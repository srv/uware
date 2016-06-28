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

        // Try visual registration



        // Registration
        vector<uint> salient_indices;
        bool valid_icp = registration(id, prev_cloud_, in_cloud, salient_indices);
        if (!valid_icp)
        {
          // No 3D information for registration, try visual registration
        }

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


} // namespace