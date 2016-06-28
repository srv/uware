#include <ros/ros.h>

#include "preprocess.h"

namespace uware
{

  PreProcess::PreProcess() : initialization_(true), first_(true), id_(0)
  {
    // ORB parameters
    int     n_features    = 2000;
    float   scale_factor  = 1.2;
    int     n_levels      = 8;
    int     ini_th_fast   = 20;
    int     min_th_fast   = 7;
    l_ORB_extractor_ = new orb_utils::ORBextractor(n_features, scale_factor, n_levels, ini_th_fast, min_th_fast);
    r_ORB_extractor_ = new orb_utils::ORBextractor(n_features, scale_factor, n_levels, ini_th_fast, min_th_fast);
  }


  void PreProcess::callback(
      const nav_msgs::Odometry::ConstPtr& odom_msg,
      const nav_msgs::Odometry::ConstPtr& map_msg,
      const sensor_msgs::ImageConstPtr& l_img_msg,
      const sensor_msgs::ImageConstPtr& r_img_msg,
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg,
      const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    // First iteration
    if (initialization_)
    {
      // Transformation between odometry and camera
      if (!getOdom2CameraTf(odom_msg->child_frame_id, l_img_msg->header.frame_id, odom2camera_))
      {
        ROS_WARN("[PreProcess]: Impossible to transform odometry to camera frame.");
        return;
      }
      ROS_INFO("[PreProcess]: TF between odom frame id and camera frame id obtained.");

      // Create operational directories
      if (!createDirs()) return;

      // Get the camera parameters
      int binning_x = l_info_msg->binning_x;
      int binning_y = l_info_msg->binning_y;
      const cv::Mat P(3,4, CV_64FC1, const_cast<double*>(l_info_msg->P.data()));
      camera_matrix_ = P.colRange(cv::Range(0,3)).clone();

      // Are the images scaled?
      if (binning_x > 1 || binning_y > 1)
      {
        camera_matrix_.at<double>(0,0) = camera_matrix_.at<double>(0,0) / binning_x;
        camera_matrix_.at<double>(0,2) = camera_matrix_.at<double>(0,2) / binning_x;
        camera_matrix_.at<double>(1,1) = camera_matrix_.at<double>(1,1) / binning_y;
        camera_matrix_.at<double>(1,2) = camera_matrix_.at<double>(1,2) / binning_y;
      }

      // Store camera matrix
      cv::FileStorage fs(params_.outdir + "/" + CAMERA_MATRIX_FILE, cv::FileStorage::WRITE);
      write(fs, "camera_matrix", camera_matrix_);
      fs.release();
      ROS_INFO("[PreProcess]: Camera matrix stored.");

      // Stereo baseline times fx
      image_geometry::StereoCameraModel stereo_camera_model;
      stereo_camera_model.fromCameraInfo(*l_info_msg, *r_info_msg);
      baseline_ = (float)stereo_camera_model.baseline()*camera_matrix_.at<float>(0,0);

      // System initialized
      initialization_ = false;
    }

    // Convert the pointcloud
    PointCloud::Ptr pcl_cloud(new PointCloud);
    fromROSMsg(*cloud_msg, *pcl_cloud);
    pcl_cloud = filterCloud(pcl_cloud);

    // Store only useful pointclouds
    if (pcl_cloud->size() < (uint)params_.min_cloud_size)
    {
      ROS_INFO_STREAM("[PreProcess]: Low quality pointcloud (size: " <<
                      pcl_cloud->size() << ", min_cloud_size: " <<
                      params_.min_cloud_size << "). Skipping...");
      return;
    }

    // Convert odometry to camera frame
    tf::Transform odom = odom2Tf(*odom_msg) * odom2camera_;
    tf::Transform map  = odom2Tf(*map_msg) * odom2camera_;

    // Compute distance
    double dist;
    if (first_)
    {
      dist = params_.store_distance + 0.1;
      first_ = false;
    }
    else
      dist = calcDist(prev_odom_, odom, params_.use_2d_distance);

    // Store odometry, images and pointclouds when required
    if (dist > params_.store_distance)
    {
      // Stamp
      double stamp = l_img_msg->header.stamp.toSec();

      // Store cloud
      stringstream ss;
      ss << setfill('0') << setw(6) << id_;
      string pc_filename = params_.outdir + "/" + PC_DIR + "/" + ss.str() + ".pcd";
      pcl::io::savePCDFileBinary(pc_filename, *pcl_cloud);

      // Store odometry
      storeOdometry(ODOM_FILE, id_, stamp, odom);
      storeOdometry(OMAP_FILE,  id_, stamp, map);

      // --------------------------------
      tf::Matrix3x3 obase = odom.getBasis();
      tf::Matrix3x3 mbase = map.getBasis();
      double oyaw, myaw, dummy_1, dummy_2;
      obase.getRPY(dummy_1, dummy_2, oyaw);
      mbase.getRPY(dummy_1, dummy_2, myaw);
      cv::Mat HO = (cv::Mat_<double>(3,3) << cos(oyaw), -sin(oyaw), odom.getOrigin().x(), sin(oyaw),  cos(oyaw), odom.getOrigin().y(), 0, 0, 1.0);
      cv::Mat HM = (cv::Mat_<double>(3,3) << cos(myaw), -sin(myaw), map.getOrigin().x(), sin(myaw),  cos(myaw), map.getOrigin().y(), 0, 0, 1.0);

      cv::FileStorage fs(params_.outdir + "/homographies/" + ss.str() + ".yaml", cv::FileStorage::WRITE);
      write(fs, "filename", "/tmp/mosaicing/images/" + ss.str() + ".jpg");
      write(fs, "HO", HO);
      write(fs, "HM", HM);
      fs.release();
      // --------------------------------

      // Store image information
      cv::Mat l_img, r_img;
      imgMsgToMat(*l_img_msg, *r_img_msg, l_img, r_img);
      int kp_size = storeImages(l_img, r_img, stamp);

      // Update
      prev_odom_ = odom;
      id_++;

      // Log
      ROS_INFO_STREAM("\n[PreProcess]: Storing pointcloud #" << id_);
      ROS_INFO_STREAM("              Dist to prev: " << dist << " m.");
      ROS_INFO_STREAM("              Cloud size: " << pcl_cloud->size());
      ROS_INFO_STREAM("              Kp size: " << kp_size);
    }

  }

  bool PreProcess::createDirs()
  {
    if (!Utils::createDir(params_.outdir))
      return false;

    if (!Utils::createDir(params_.outdir + "/homographies"))
      return false;

    if (!Utils::createDir(params_.outdir + "/" + PC_DIR))
      return false;

    if (!Utils::createDir(params_.outdir + "/" + IMG_DIR))
      return false;

    // Remove files
    string cm_file   = params_.outdir + "/" + CAMERA_MATRIX_FILE;
    string odom_file = params_.outdir + "/" + ODOM_FILE;
    string map_file  = params_.outdir + "/" + OMAP_FILE;
    remove(cm_file.c_str());
    remove(odom_file.c_str());
    remove(map_file.c_str());

    return true;
  }

  double PreProcess::calcDist(tf::Transform a, tf::Transform b, bool use_2d_distance)
  {
    tf::Vector3 d = a.getOrigin() - b.getOrigin();
    if (use_2d_distance)
      return sqrt(d.x()*d.x() + d.y()*d.y());
    else
      return sqrt(d.x()*d.x() + d.y()*d.y() + d.z()*d.z());
  }

  void PreProcess::storeOdometry(string filename, int id, double stamp, tf::Transform odometry)
  {
    string odom_file = params_.outdir + "/" + filename;
    fstream f_odom(odom_file.c_str(), ios::out | ios::app);

    // Convert id to 6 digits
    stringstream ss;
    ss << setfill('0') << setw(6) << id;

    f_odom << fixed <<
    setprecision(6) <<
    ss.str() << "," <<
    stamp << "," <<

    odometry.getOrigin().x() << "," <<
    odometry.getOrigin().y() << "," <<
    odometry.getOrigin().z() << "," <<
    odometry.getRotation().x() << "," <<
    odometry.getRotation().y() << "," <<
    odometry.getRotation().z() << "," <<
    odometry.getRotation().w() <<  endl;

    f_odom.close();
  }

  int PreProcess::storeImages(cv::Mat l_img, cv::Mat r_img, double stamp)
  {
    // Extract ORB
    orb_utils::Frame frame = orb_utils::Frame(l_img, r_img, stamp,
                                              l_ORB_extractor_,
                                              r_ORB_extractor_,
                                              camera_matrix_,
                                              baseline_);
    std::vector<cv::KeyPoint> l_kp = frame.mvKeys;
    std::vector<cv::KeyPoint> r_kp = frame.mvKeysRight;
    cv::Mat l_orb_desc             = frame.mDescriptors;
    cv::Mat r_orb_desc             = frame.mDescriptorsRight;

    // Extract SIFT
    cv::initModule_nonfree();
    cv::Mat l_sift_desc, r_sift_desc;
    cv::Ptr<cv::DescriptorExtractor> cv_extractor;
    cv_extractor = cv::DescriptorExtractor::create("SIFT");
    cv_extractor->compute(l_img, l_kp, l_sift_desc);
    cv_extractor->compute(r_img, r_kp, r_sift_desc);

    // Store
    stringstream ss;
    ss << setfill('0') << setw(6) << id_;
    cv::FileStorage fs(params_.outdir + "/" + IMG_DIR + "/" + ss.str() + ".yaml", cv::FileStorage::WRITE);
    write(fs, "l_kp", l_kp);
    write(fs, "r_kp", r_kp);
    write(fs, "l_orb_desc", l_orb_desc);
    write(fs, "r_orb_desc", r_orb_desc);
    write(fs, "l_sift_desc", l_sift_desc);
    write(fs, "r_sift_desc", r_sift_desc);
    fs.release();

    cv::imwrite(params_.outdir + "/" + IMG_DIR + "/" + ss.str() + ".jpg", l_img);

    return l_kp.size();
  }

  bool PreProcess::imgMsgToMat(sensor_msgs::Image l_img_msg,
                               sensor_msgs::Image r_img_msg,
                               cv::Mat &l_img, cv::Mat &r_img)
  {
    // Convert message to cv::Mat
    cv_bridge::CvImagePtr l_img_ptr, r_img_ptr;
    try
    {
      l_img_ptr = cv_bridge::toCvCopy(l_img_msg, enc::BGR8);
      r_img_ptr = cv_bridge::toCvCopy(r_img_msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("[PreProcess]: cv_bridge exception: %s", e.what());
      return false;
    }

    // Set the images
    l_img = l_img_ptr->image;
    r_img = r_img_ptr->image;
    return true;
  }

  tf::Transform PreProcess::odom2Tf(nav_msgs::Odometry odom_msg)
  {
    // Get the data
    double tx = odom_msg.pose.pose.position.x;
    double ty = odom_msg.pose.pose.position.y;
    double tz = odom_msg.pose.pose.position.z;

    double qx = odom_msg.pose.pose.orientation.x;
    double qy = odom_msg.pose.pose.orientation.y;
    double qz = odom_msg.pose.pose.orientation.z;
    double qw = odom_msg.pose.pose.orientation.w;

    // Sanity check
    if(qx == 0.0 && qy == 0.0 && qz == 0.0 && qw == 1.0)
    {
      tf::Transform odom;
      odom.setIdentity();
      return odom;
    }
    else
    {
      tf::Vector3 tf_trans(tx, ty, tz);
      tf::Quaternion tf_q (qx, qy, qz, qw);
      tf::Transform odom(tf_q, tf_trans);
      return odom;
    }
  }

  bool PreProcess::getOdom2CameraTf(string odom_frame_id,
      string camera_frame_id,
      tf::StampedTransform &transform)
  {
    // Init the transform
    transform.setIdentity();

    try
    {
      // Extract the transform
      tf_listener_.lookupTransform(odom_frame_id,
          camera_frame_id,
          ros::Time(0),
          transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("%s", ex.what());
      return false;
    }
    return true;
  }

  PointCloud::Ptr PreProcess::filterCloud(PointCloud::Ptr in_cloud)
  {
    // Remove nans
    vector<int> indicies;
    PointCloud::Ptr cloud(new PointCloud);
    pcl::removeNaNFromPointCloud(*in_cloud, *cloud, indicies);

    // Voxel grid filter (used as x-y surface extraction. Note that leaf in z is very big)
    pcl::ApproximateVoxelGrid<Point> grid;
    grid.setLeafSize(params_.voxel_resolution, params_.voxel_resolution, params_.voxel_resolution);
    grid.setDownsampleAllData(true);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);

    return cloud;
  }

}
