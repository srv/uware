#include <ros/ros.h>

#include "frame_to_frame.h"

namespace uware
{
  FrameToFrame::FrameToFrame() : first_(true), prev_cloud_(new PointCloud) { }

  void FrameToFrame::compute()
  {
    // Create directories
    if (!createDirs()) return;

    // Read poses
    Utils::readPoses(params_.indir + "/" + ODOM_FILE, cloud_poses_);

    // Read camera matrix
    cv::FileStorage fs(params_.indir + "/" + CAMERA_MATRIX_FILE, cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix_;
    fs.release();

    // Sort directory of pointclouds by name
    typedef std::vector<fs::path> vec;
    vec v;
    copy(
          fs::directory_iterator(params_.indir + "/" + PC_DIR),
          fs::directory_iterator(),
          back_inserter(v)
        );

    sort(v.begin(), v.end());
    vec::const_iterator it(v.begin());

    // Iterate over all pointclouds
    while (it!=v.end())
    {
      // Check if the directory entry is an directory.
      if (!fs::is_directory(*it))
      {
        // Get the filename
        string filename = it->filename().string();
        string filepath = params_.indir + "/" + PC_DIR + "/" + filename;
        int lastindex = filename.find_last_of(".");
        string rawname = filename.substr(0, lastindex);
        ROS_INFO_STREAM("[Reconstruction]: Processing cloud: " << filename);

        // Search current pointcloud into the cloud poses
        uint id = getCloudPoseId(rawname, cloud_poses_);
        if (id < 0)
        {
          ROS_ERROR_STREAM("[Reconstruction]: Impossible to find the id into the cloud_poses_ array for file: " << filename);
          break;
        }

        // Load the pointcloud
        PointCloud::Ptr in_cloud(new PointCloud);
        if (pcl::io::loadPCDFile<Point> (filepath, *in_cloud) == -1)
        {
          ROS_WARN_STREAM("[Reconstruction]: Couldn't read the file: " << filename);
          it++;
          continue;
        }

        // First iteration
        if (first_)
        {
          pcl::copyPointCloud(*in_cloud, *prev_cloud_);

          vector<uint> salient_indices;
          Utils::getCloudSalientIndices(prev_cloud_, salient_indices, params_.z_diff_th);
          prev_cloud_salient_ids_ = salient_indices.size() * 100.0 / prev_cloud_->size();
          prev_rawname_ = rawname;

          first_ = false;
          it++;
          continue;
        }

        // Visual registration
        tf::Transform output;


        // TODO: Use initial guest!!!

        bool valid_sim3 = calcSim3(id, output);

        // 3D Registration
        vector<uint> salient_indices;
        bool valid_icp = registration(id, prev_cloud_, in_cloud, salient_indices);

        // Copy
        pcl::copyPointCloud(*in_cloud, *prev_cloud_);
        prev_cloud_salient_ids_ = salient_indices.size() * 100.0 / prev_cloud_->size();
        prev_rawname_ = rawname;

      }
      // Next directory entry
      it++;

    }

  }

  bool FrameToFrame::createDirs()
  {
    if (!Utils::createDir(params_.outdir))
      return false;

    if (!Utils::createDir(params_.outdir + "/" + PAIRED_CLOUDS_DIR))
      return false;

    return true;
  }

  int FrameToFrame::getCloudPoseId(string rawname, vector< pair<string, tf::Transform> > cloud_poses_)
  {
    for (uint i=0; i<cloud_poses_.size(); i++)
    {
      if (cloud_poses_[i].first == rawname)
        return i;
    }
    return -1;
  }

  bool FrameToFrame::pairAlign(PointCloud::Ptr src,
                               PointCloud::Ptr tgt,
                               tf::Transform &output)
  {
    PointCloud::Ptr aligned (new PointCloud);
    IterativeClosestPoint icp;
    icp.setMaxCorrespondenceDistance(0.07);
    icp.setRANSACOutlierRejectionThreshold(0.005);
    icp.setTransformationEpsilon(0.00001);
    icp.setEuclideanFitnessEpsilon(0.0001);
    icp.setMaximumIterations(200);
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.align(*aligned);
    double score = icp.getFitnessScore();

    if (params_.show_icp_score)
      ROS_INFO_STREAM("[Reconstruction]: ICP fitness score: " << score);

    // The transform
    output = Utils::matrix4fToTf(icp.getFinalTransformation());
    if (params_.show_icp_tf)
    {
      ROS_INFO_STREAM("[Reconstruction]: ICP correction: " << output.getOrigin().x() <<
                                                      ", " << output.getOrigin().y() <<
                                                      ", " << output.getOrigin().z());
    }

    // Return valid or not
    return ( icp.hasConverged() && (score < params_.max_icp_fitness_score) );
  }

  bool FrameToFrame::registration(int id, PointCloud::Ptr prev_cloud, PointCloud::Ptr curr_cloud, vector<uint>& salient_indices)
  {
    // Determine if current and previous pointcloud are useful for registration
    salient_indices.clear();
    Utils::getCloudSalientIndices(curr_cloud, salient_indices, params_.z_diff_th);
    double curr_salient_ids = salient_indices.size() * 100.0 / curr_cloud->size();

    if (params_.show_salient_ids)
      ROS_INFO_STREAM("[Reconstruction]: Salient indices for " << cloud_poses_[id].first << ".pcd is: " << curr_salient_ids << "\%");

    bool valid_icp = false;
    if (prev_cloud_salient_ids_ > params_.min_salient_ids &&
        curr_salient_ids > params_.min_salient_ids)
    {
      // Frame to frame registration
      ROS_INFO_STREAM("[Reconstruction]: " << cloud_poses_[id].first << ".pcd is useful for registration.");
      ROS_INFO("[Reconstruction]: Aligning clouds...");

      // Transform current cloud according to slam pose
      tf::Transform tf_prev = cloud_poses_[id-1].second;
      tf::Transform tf_curr = cloud_poses_[id].second;
      tf::Transform tf_01 = tf_curr.inverse() * tf_prev;
      Eigen::Affine3d tf_01_eigen;
      transformTFToEigen(tf_01, tf_01_eigen);
      PointCloud::Ptr prev_cloud_moved(new PointCloud);
      pcl::transformPointCloud(*prev_cloud_, *prev_cloud_moved, tf_01_eigen);

      // Pair align
      tf::Transform correction;
      valid_icp = pairAlign(prev_cloud_moved, curr_cloud, correction);
      if (valid_icp && params_.save_icp_clouds)
      {
        // Make the new directory
        stringstream ss;
        ss << setfill('0') << setw(6) << (id-1) << "_to_" << setfill('0') << setw(6) << (id);
        string paired_dir = params_.outdir + "/" + PAIRED_CLOUDS_DIR + "/" + ss.str();
        if (Utils::createDir(paired_dir))
        {
          // Move and save
          Eigen::Affine3d correction_eigen;
          transformTFToEigen(correction, correction_eigen);
          pcl::transformPointCloud(*prev_cloud_moved, *prev_cloud_moved, correction_eigen);

          string pc0_filename = paired_dir + "/" + cloud_poses_[id].first + ".pcd";
          pcl::io::savePCDFileBinary(pc0_filename, *curr_cloud);
          string pc1_filename = paired_dir + "/" + prev_rawname_ + ".pcd";
          pcl::io::savePCDFileBinary(pc1_filename, *prev_cloud_moved);
        }
      }
    }
    return valid_icp;
  }

  bool FrameToFrame::calcSim3(int id, tf::Transform& output)
  {
    output.setIdentity();

    // Load descriptors
    cv::FileStorage prev_fs, curr_fs;
    stringstream id_prev, id_curr;
    id_prev << setfill('0') << setw(6) << id-1;
    id_curr << setfill('0') << setw(6) << id;
    string prev_img_data = params_.indir + "/" + IMG_DIR + "/" + id_prev.str() + ".yaml";
    string curr_img_data = params_.indir + "/" + IMG_DIR + "/" + id_curr.str() + ".yaml";
    prev_fs.open(prev_img_data, cv::FileStorage::READ);
    if (!prev_fs.isOpened()) return false;
    curr_fs.open(curr_img_data, cv::FileStorage::READ);
    if (!curr_fs.isOpened()) return false;
    cv::Mat prev_desc, curr_desc;
    vector<cv::KeyPoint> prev_kp;
    vector<cv::Point3d> curr_wp;
    cv::FileNode prev_kp_node = prev_fs["l_kp"];
    read(prev_kp_node, prev_kp);
    prev_fs["l_desc"]       >> prev_desc;
    curr_fs["l_desc"]       >> curr_desc;
    curr_fs["world_points"] >> curr_wp;
    prev_fs.release();
    curr_fs.release();

    if (params_.show_num_of_kp)
      ROS_INFO_STREAM("[Reconstruction]: Current kps: " << curr_desc.rows << ". Previous kps: " << prev_desc.rows);
    if (prev_desc.rows < params_.min_desc_matches || curr_desc.rows < params_.min_desc_matches)
    {
      if (params_.show_generic_logs)
        ROS_INFO_STREAM("[Reconstruction]: Low number of keypoints: " <<
                        prev_desc.rows << "(previous) and " << curr_desc.rows <<
                        "(current). Threshold (min_desc_matches): " << params_.min_desc_matches);
      return false;
    }
    else
    {
      // Descriptor matching
      vector<cv::DMatch> matches;
      Utils::ratioMatching(prev_desc, curr_desc, params_.desc_matching_th, matches);
      if ((int)matches.size() < params_.min_desc_matches)
      {
        if (params_.show_generic_logs)
          ROS_INFO_STREAM("[Reconstruction]: Not enough matches for Sim3: " <<
                          matches.size() << " (threshold: " << params_.min_desc_matches << ").");
        return false;
      }
      else
      {
        // Build the matches vector
        vector<cv::Point2f> matched_kp_prev;
        vector<cv::Point3f> matched_3d_curr;
        for(uint i=0; i<matches.size(); i++)
        {
          matched_kp_prev.push_back(prev_kp[matches[i].queryIdx].pt);
          matched_3d_curr.push_back(curr_wp[matches[i].trainIdx]);
        }

        // Estimate the motion
        vector<int> inliers;
        cv::Mat rvec, tvec;
        cv::solvePnPRansac(matched_3d_curr, matched_kp_prev, camera_matrix_,
                           cv::Mat(), rvec, tvec, false,
                           100, params_.reproj_err, 100, inliers);

        if (params_.show_generic_logs)
          ROS_INFO_STREAM("[Reconstruction]: Sim3 inliers: " << inliers.size());

        if ((int)inliers.size() >= params_.min_inliers)
        {
          output = Utils::buildTransformation(rvec, tvec);
          return true;
        }
      }
    }

    return false;
  }


} // namespace