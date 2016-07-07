#include <ros/ros.h>

#include "registration.h"

namespace uware
{
  Registration::Registration() { }

  bool Registration::pairAlign(PointCloud::Ptr src,
                               PointCloud::Ptr tgt,
                               double& score,
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
    score = icp.getFitnessScore();

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

    bool valid = ( icp.hasConverged() && (score < params_.max_icp_fitness_score) );
    if (!valid)
      score = -1.0;

    // Return valid or not
    return valid;
  }

  bool Registration::regClouds(PointCloud::Ptr src_cloud,
                               PointCloud::Ptr tgt_cloud,
                               double& score,
                               tf::Transform& est_movement)
  {
    // Transform current cloud according to slam pose
    Eigen::Affine3d tf_01_eigen;
    transformTFToEigen(est_movement.inverse(), tf_01_eigen);
    PointCloud::Ptr src_cloud_moved(new PointCloud);
    pcl::transformPointCloud(*src_cloud, *src_cloud_moved, tf_01_eigen);

    // Pair align
    tf::Transform correction;
    bool valid_icp = pairAlign(src_cloud_moved, tgt_cloud, score, correction);
    if (valid_icp)
      est_movement = (est_movement.inverse() * correction).inverse();

    return valid_icp;
  }

  bool Registration::regImages(cv::Mat src_desc,
                               cv::Mat tgt_desc,
                               vector<cv::KeyPoint> src_kp,
                               vector<cv::Point3d> tgt_wp,
                               cv::Mat camera_matrix,
                               int& inliers_num,
                               tf::Transform& est_movement)
  {
    inliers_num = 0;

    if (params_.show_num_of_kp)
    {
      ROS_INFO_STREAM("[Reconstruction]: Source kps: " << src_desc.rows << ". Target kps: " << tgt_desc.rows);
    }
    if (src_desc.rows < params_.min_desc_matches || tgt_desc.rows < params_.min_desc_matches)
    {
      if (params_.show_generic_logs)
      {
        ROS_INFO_STREAM("[Reconstruction]: Low number of keypoints: " <<
                        src_desc.rows << "(source) and " << tgt_desc.rows <<
                        "(target). Threshold (min_desc_matches): " << params_.min_desc_matches);
      }
      return false;
    }
    else
    {
      // Descriptor matching
      vector<cv::DMatch> matches;
      Utils::ratioMatching(src_desc, tgt_desc, params_.desc_matching_th, matches);
      if ((int)matches.size() < params_.min_desc_matches)
      {
        if (params_.show_generic_logs)
        {
          ROS_INFO_STREAM("[Reconstruction]: Not enough matches for Sim3: " <<
                          matches.size() << " (threshold: " << params_.min_desc_matches << ").");
        }
        return false;
      }
      else
      {
        // Build the matches vector
        vector<cv::Point2f> matched_kp_src;
        vector<cv::Point3f> matched_3d_tgt;
        for(uint i=0; i<matches.size(); i++)
        {
          matched_kp_src.push_back(src_kp[matches[i].queryIdx].pt);
          matched_3d_tgt.push_back(tgt_wp[matches[i].trainIdx]);
        }

        // Set initial guest
        cv::Mat rvec, tvec;
        Utils::tf2SolvePnP(est_movement, rvec, tvec);

        // Estimate the motion
        vector<int> inliers;
        cv::solvePnPRansac(matched_3d_tgt, matched_kp_src, camera_matrix,
                           cv::Mat(), rvec, tvec, true,
                           100, params_.reproj_err, 100, inliers);

        if (params_.show_generic_logs)
        {
          ROS_INFO_STREAM("[Reconstruction]: Sim3 inliers: " << inliers.size());
        }

        if ((int)inliers.size() >= params_.min_inliers)
        {
          inliers_num = (int)inliers.size();
          est_movement = Utils::solvePnP2tf(rvec, tvec);
          return true;
        }
      }
    }

    return false;
  }

  void Registration::pipeline(const uint& id_src,
                              const uint& id_tgt,
                              const tf::Transform& initial_guest,
                              bool& valid_sim3,
                              bool& valid_icp,
                              int& sim3_inliers,
                              double& icp_score,
                              const cv::Mat& camera_matrix,
                              tf::Transform& out)
  {
    // Init edge information
    out = initial_guest;
    valid_sim3 = false;
    valid_icp = false;
    sim3_inliers = 0;
    icp_score = -1.0;

    // Convert ids to string
    string id_src_str = Utils::id2str(id_src);
    string id_tgt_str = Utils::id2str(id_tgt);

    // Visual registration
    // Read data
    cv::Mat src_desc;
    cv::Mat tgt_desc;
    vector<cv::KeyPoint> src_kp;
    vector<cv::Point3d> tgt_wp;
    readImgData(id_src_str, id_tgt_str, src_desc, tgt_desc, src_kp, tgt_wp);

    // Register images
    tf::Transform tf_sim3 = out;
    valid_sim3 = regImages(src_desc,
                           tgt_desc,
                           src_kp,
                           tgt_wp,
                           camera_matrix,
                           sim3_inliers,
                           tf_sim3);
    if (valid_sim3)
    {
      double err = Utils::tfDist(out, tf_sim3);
      if (err < params_.max_reg_err)
      {
        out = tf_sim3;
      }
      else
      {
        ROS_WARN_STREAM("[Registration]: Error between odometry and sim3 too large: " <<
                        err << "m. (Allowed: " << params_.max_reg_err << ").");
      }
    }

    // Read pointclouds
    string filepath_src = params_.indir + "/" + PC_DIR + "/" + id_src_str + ".pcd";
    string filepath_tgt = params_.indir + "/" + PC_DIR + "/" + id_tgt_str + ".pcd";
    PointCloud::Ptr src_cloud(new PointCloud);
    PointCloud::Ptr tgt_cloud(new PointCloud);
    if (pcl::io::loadPCDFile<Point> (filepath_src, *src_cloud) == -1)
    {
      ROS_ERROR_STREAM("[Registration]: Pointcloud " << filepath_src << " does not exists!");
      return;
    }
    if (pcl::io::loadPCDFile<Point> (filepath_tgt, *tgt_cloud) == -1)
    {
      ROS_ERROR_STREAM("[Registration]: Pointcloud " << filepath_tgt << " does not exists!");
      return;
    }


    // 3D Registration
    // Determine if current and previous pointcloud are useful for registration
    vector<uint> src_salient_indices, tgt_salient_indices;
    Utils::getCloudSalientIndices(src_cloud, src_salient_indices, params_.z_diff_th);
    Utils::getCloudSalientIndices(tgt_cloud, tgt_salient_indices, params_.z_diff_th);
    double src_salient_ids = src_salient_indices.size() * 100.0 / src_cloud->size();
    double tgt_salient_ids = tgt_salient_indices.size() * 100.0 / tgt_cloud->size();
    if (params_.show_generic_logs)
    {
      ROS_INFO_STREAM("[Reconstruction]: Salient indices for " << id_src_str <<
                      ".pcd is: " << src_salient_ids << "\%");
      ROS_INFO_STREAM("[Reconstruction]: Salient indices for " << id_tgt_str <<
                      ".pcd is: " << tgt_salient_ids << "\%");
    }

    if (src_salient_ids > params_.min_salient_ids &&
        tgt_salient_ids > params_.min_salient_ids)
    {
      ROS_INFO("[Reconstruction]: Aligning clouds...");

      tf::Transform tf_icp = out;
      valid_icp = regClouds(src_cloud, tgt_cloud, icp_score, tf_icp);
      if (valid_icp)
      {
        double err = Utils::tfDist(out, tf_icp);
        if (err < params_.max_reg_err)
        {
          out = tf_icp;

          if (params_.save_icp_clouds)
          {
            // Make the new directory
            string dir_str = id_src_str + "_to_" + id_tgt_str;
            string paired_dir = params_.outdir + "/" + PAIRED_CLOUDS_DIR + "/" + dir_str;
            if (Utils::createDir(paired_dir))
            {
              // Move and save
              Eigen::Affine3d src2tgt_eigen;
              transformTFToEigen(out.inverse(), src2tgt_eigen);
              PointCloud::Ptr src_cloud_moved(new PointCloud);
              pcl::transformPointCloud(*src_cloud, *src_cloud_moved, src2tgt_eigen);
              string pc_src_filename = paired_dir + "/" + id_src_str + ".pcd";
              pcl::io::savePCDFileBinary(pc_src_filename, *src_cloud_moved);
              string pc_tgt_filename = paired_dir + "/" + id_tgt_str + ".pcd";
              pcl::io::savePCDFileBinary(pc_tgt_filename, *tgt_cloud);
            }
          }
        }
        else
        {
          ROS_WARN_STREAM("[Registration]: Error between odometry and icp too large: " <<
                          err << "m. (Allowed: " << params_.max_reg_err << ").");
        }
      }
    }
  }

  void Registration::readImgData(const string& id_src,
                                 const string& id_tgt,
                                 cv::Mat& desc_src,
                                 cv::Mat& desc_tgt,
                                 vector<cv::KeyPoint>& kp_src,
                                 vector<cv::Point3d>& wp_tgt)
  {
    cv::FileStorage fs_src, fs_tgt;
    string src_img_data = params_.indir + "/" + IMG_DIR + "/" + id_src + ".yaml";
    string tgt_img_data = params_.indir + "/" + IMG_DIR + "/" + id_tgt + ".yaml";
    fs_src.open(src_img_data, cv::FileStorage::READ);
    if (!fs_src.isOpened()) return;
    fs_tgt.open(tgt_img_data, cv::FileStorage::READ);
    if (!fs_tgt.isOpened()) return;
    cv::FileNode src_kp_node = fs_src["l_kp"];
    read(src_kp_node, kp_src);
    fs_src["l_desc"]       >> desc_src;
    fs_tgt["l_desc"]       >> desc_tgt;
    fs_tgt["world_points"] >> wp_tgt;
    fs_src.release();
    fs_tgt.release();
  }


} // namespace