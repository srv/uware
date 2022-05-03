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
    ROS_INFO("[PreProcess]: init Preprocess node.");
  }

  //const sensor_msgs::PointCloud2ConstPtr& cloud_msg
  void PreProcess::callback(
      // const nav_msgs::Odometry::ConstPtr& odom_msg,
      // const nav_msgs::Odometry::ConstPtr& map_msg,
      const sensor_msgs::ImageConstPtr& l_img_msg,
      const sensor_msgs::ImageConstPtr& r_img_msg,
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg,
      const sensor_msgs::RangeConstPtr& altitude_msg,
      const cola2_msgs::NavSts::ConstPtr& navstatus_msg)
  {
    double altitud = altitude_msg->range;
    // float latitud = navstatus_msg->global_position.latitude; //// lat/long of Turbot base link
    // float longitud = navstatus_msg->global_position.longitude;
    float north = navstatus_msg->position.north; //// NED pose of Turbot base link, in translation and rotation, with respect to NED origin.
    float east = navstatus_msg->position.east; 
    float depth = navstatus_msg->position.depth;
    float roll = navstatus_msg->orientation.roll;
    float pitch = navstatus_msg->orientation.pitch;
    float yaw = navstatus_msg->orientation.yaw;
    std_msgs::Header headerNav = navstatus_msg->header;
    float originlat = navstatus_msg->origin.latitude; //// NED lat/lon origin of trajectory 
    float originlong = navstatus_msg->origin.longitude;
    int height = l_info_msg->height;  
    int width = l_info_msg->width;
    
    double FOV_x = 34.73 * ((2*M_PI)/360.0); //// images field of view. The constants have been obtained experimentaly and depend on each lens. These values are for the Turbot stereo camera Camaleon.  
    double FOV_y = 26.84 * ((2*M_PI)/360.0); //// remember that the field of view does not change with the image resolution. 
                                             ////  dimensions of FOV in x and y in metric units, dpeending on the flying height   
    double x_dim_img = 2 * altitud * tan(FOV_x/2); //#width 
    double y_dim_img = 2 * altitud * tan(FOV_y/2); //#height

    /// get the Turbot velocity. 
    float body_velocityX = navstatus_msg->body_velocity.x;
    float body_velocityY = navstatus_msg->body_velocity.y;
    float body_velocityZ = navstatus_msg->body_velocity.z;

    // ROS_INFO_STREAM("[PreProcess]: latitude and longitude of base link " << latitud <<" // " <<  longitud );
    
    // First iteration
    ROS_INFO("[PreProcess]: Processing the Inputs Callback.");
    if (initialization_)
    {
      // Transformation between turbot base link and camera frame (left optical). odom2camera gives the transform between the Turbot base link and the left optical
      if (!getOdom2CameraTf(params_.base_link_frame_id, l_img_msg->header.frame_id, odom2camera_))
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
      fs << "camera_matrix " << camera_matrix_;
      //write(fs, "camera_matrix", camera_matrix_);
      fs.release();
      ROS_INFO("[PreProcess]: Camera matrix stored.");

      // Stereo baseline times fx
      stereo_camera_model_.fromCameraInfo(*l_info_msg, *r_info_msg);
      baseline_ = (double)stereo_camera_model_.baseline()*camera_matrix_.at<double>(0,0);

      // NED initialization
      ned_ = new Ned(originlat, originlong, 0.0); // initialize NED origin

      // Check max_altitude
      ROS_INFO_STREAM("Max altitude (NODE): " << params_.max_altitude) ;

      storeNavSts(NAVSTS_FILE, 0, 0, 0, 0, 0, initialization_) ;
      storeLatLonCorners(LATLONCORNERS_FILE, 0, 0, 
                        0, 0, 0, 
                        0, 0, 0, 
                        0, 0, 0,
                        0, 0, 0,
                        0, 0, 0,
                        0, 0, 0,
                        initialization_);

      // System initialized
      initialization_ = false;
      
    }

    // Convert the pointcloud
    /* fbf commentted on 18/01/2021 uncomment of PC storage
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
    }*/

    // Convert odometry (map and odom) to camera frame (left optical). In this way, we have the motion of the lens instead of the robot motion.   
    // tf::Transform odom = odom2Tf(*odom_msg) * odom2camera_;
    // tf::Transform map  = odom2Tf(*map_msg) * odom2camera_;

    /* Transform NED data obtained with respect to the turbot base_link to the left camera optical. Then, convert from NED to Geodesical to obtain the lat/long of
    the image instead of lat/log of the turbot baselink */

    nav_msgs::Odometry odometry_tmp; // create temporal odometry message
    
    odometry_tmp.header = headerNav; // its header will be the same as the navsts header
    
    //set the position of the temp. odometry message from the NED position
    odometry_tmp.pose.pose.position.x = north;
    odometry_tmp.pose.pose.position.y = east;
    odometry_tmp.pose.pose.position.z = depth;
    // create a new quaternion 
    tf::Quaternion odom_quat; /// create a new quaternion type "tf"
    odom_quat.setRPY(roll, pitch, yaw); // set the quaternion from the roll,pitch and yaw of the NavSts NED orientation
    odom_quat.normalize(); // normalize in such a way the norm of the quaternion is 1
    geometry_msgs::Quaternion odom_quat_tmp; /// set the quaternion of the temp. odom. to this quaternion

    tf::quaternionTFToMsg(odom_quat, odom_quat_tmp); // transform the tf::quaternion into a geometry message Quaternion
    odometry_tmp.pose.pose.orientation = odom_quat_tmp; // put it into the orientation of the temporal odometry message
    //set the velocity of the temp. odom. message with the vel. of the NavSts.
    odometry_tmp.child_frame_id = params_.base_link_frame_id;
    odometry_tmp.twist.twist.linear.x = body_velocityX;
    odometry_tmp.twist.twist.linear.y = body_velocityY;
    odometry_tmp.twist.twist.linear.z = body_velocityZ;

    /*image corners*/  
    double W = (x_dim_img/2), H = (y_dim_img/2); /// get the horizontal and vertical metric view dimensions
    // double angle = atan(W/H); /// get the orientation of the upper right corner of the view with respect to its center. 
    double angle = atan(H/W);
    
    double roll1 = 0, pitch1 = 0, yaw1 = angle; 
    //transform (tf format) between view (image to space) center to the 4 corners. 
    tf::Transform uprightCorner_tf = data2Tf(W, H, roll1, pitch1, yaw1);
    tf::Transform upleftCorner_tf = data2Tf(-W, H, roll1, pitch1, -yaw1);
    tf::Transform downrightCorner_tf = data2Tf(W, -H, roll1, pitch1, (M_PI-yaw1));
    tf::Transform downleftCorner_tf = data2Tf(-W, -H, roll1, pitch1, (M_PI+yaw1));
    tf::Transform upmiddle_tf = data2Tf(0, H, roll1, pitch1, 0) ;

    /// transform the odometry tmp message, which contains the NED Turbot base Link pose to the left optical frame. The result is the NED pose of the left optical with respect to the NED origin. 

    tf::Transform cameraMap_tf = odom2Tf(odometry_tmp) * odom2camera_; // product of NED base link pose and transform base link to left optical = transform NED origin to the left optical. 

    // transform NED base link pose to left optical (which can be assumed as being the image center) 
    // and tafterwards transform from left optical (image center) to corners. The results are the transforms from NED origin to all view corners

    tf::Transform uprightMap_tf = cameraMap_tf * uprightCorner_tf; 
    tf::Transform upleftMap_tf = cameraMap_tf * upleftCorner_tf;
    tf::Transform downrightMap_tf = cameraMap_tf * downrightCorner_tf; 
    tf::Transform downleftMap_tf = cameraMap_tf * downleftCorner_tf;
    tf::Transform upmiddleMap_tf = cameraMap_tf * upmiddle_tf ;

    // Center and corners Lat Lons
    LatLonAltitude uprightLatLon = ned_tf2Geo(uprightMap_tf) ;
    LatLonAltitude upleftLatlon = ned_tf2Geo(upleftMap_tf) ;
    LatLonAltitude downrightLatlon = ned_tf2Geo(downrightMap_tf) ;
    LatLonAltitude downleftLatlon = ned_tf2Geo(downleftMap_tf) ;
    LatLonAltitude centerLatlon = ned_tf2Geo(cameraMap_tf) ;
    LatLonAltitude upMiddleLatlon = ned_tf2Geo(upmiddleMap_tf) ;

    ROS_INFO_STREAM("[PreProcess]: Geodesic data corresponding to the NED pose of the left optical. Lat: " << centerLatlon.lat <<" Lon: " <<  centerLatlon.lon << " Altitude: " << centerLatlon.h);

    // Compute distance
    double dist;
    if (first_)
    {
      dist = params_.store_distance + 0.1;
      first_ = false;
    }
    else
    {
      dist = calcDist(cameraMap_tf_prev_, cameraMap_tf, params_.use_2d_distance);
    }

    ROS_INFO_STREAM("              Altitude: " << altitud << " m.") ;
    // Store odometry, images and pointclouds when required
    if (dist > params_.store_distance)
    {
      if (altitud < params_.max_altitude){

        // Stamp
        double stamp = l_img_msg->header.stamp.toSec();

        // Store cloud 
        /*fbf commentted on 18/01/2021 uncomment of PC storage
        string pc_filename = params_.outdir + "/" + PC_DIR + "/" + Utils::id2str(id_) + ".pcd";
        pcl::io::savePCDFileBinary(pc_filename, *pcl_cloud);*/

        // Store odometries and the Geodesic Data of view center and 4 corners. 
        // storeOdometry(ODOM_FILE, id_, stamp, odom);
        storeOdometry(OMAP_FILE,  id_, stamp, cameraMap_tf);

        storeNavSts(NAVSTS_FILE, id_, stamp, centerLatlon.lat, centerLatlon.lon, altitud, initialization_) ;

        storeLatLonCorners(LATLONCORNERS_FILE, id_, stamp, 
                          centerLatlon.lat, centerLatlon.lon, centerLatlon.h, 
                          uprightLatLon.lat, uprightLatLon.lon, uprightLatLon.h, 
                          upleftLatlon.lat, upleftLatlon.lon, upleftLatlon.h,
                          downrightLatlon.lat, downrightLatlon.lon, downrightLatlon.h,
                          downleftLatlon.lat, downleftLatlon.lon, downleftLatlon.h,
                          upMiddleLatlon.lat, upMiddleLatlon.lon, upMiddleLatlon.h,
                          initialization_);
        

        // --------------------------------
        // tf::Matrix3x3 obase = odom.getBasis();
        tf::Matrix3x3 mbase = cameraMap_tf.getBasis();
        double oyaw, myaw, dummy_1, dummy_2;
        // obase.getRPY(dummy_1, dummy_2, oyaw);
        mbase.getRPY(dummy_1, dummy_2, myaw);
        ROS_INFO("[PreProcess]: Register in CSV file.");
        /// store the yaml file with odometries for later mosaicing 
        // cv::Mat HO = (cv::Mat_<double>(3,3) << cos(oyaw), -sin(oyaw), odom.getOrigin().x(), sin(oyaw),  cos(oyaw), odom.getOrigin().y(), 0, 0, 1.0);
        cv::Mat HM = (cv::Mat_<double>(3,3) << cos(myaw), -sin(myaw), cameraMap_tf.getOrigin().x(), sin(myaw),  cos(myaw), cameraMap_tf.getOrigin().y(), 0, 0, 1.0);
        cv::FileStorage fs(params_.outdir + "/homographies/" + Utils::id2str(id_) + ".yaml", cv::FileStorage::WRITE);
        write(fs, "filename", params_.outdir + "/images/" + Utils::id2str(id_) + ".jpg");
        // write(fs, "HO", HO);
        write(fs, "HM", HM);
        write(fs, "ALT", altitud);
        write(fs, "LAT", centerLatlon.lat);
        write(fs, "LONG", centerLatlon.lon);

        // write(fs, "LAT", latitud);
        // write(fs, "LONG", longitud);

        fs.release();
        // --------------------------------

        // Store image information
        cv::Mat l_img, r_img;
        imgMsgToMat(*l_img_msg, *r_img_msg, l_img, r_img);
        int kp_size = storeImages(l_img, r_img, stamp);
        ROS_INFO_STREAM("              Saving image") ;
        ROS_INFO_STREAM("              Kp size: " << kp_size) ;

      }

      // Update
      cameraMap_tf_prev_ = cameraMap_tf;
      id_++;

      // Log
      /*fbf commentted on 18/01/2021 uncomment of PC storage
      ROS_INFO_STREAM("\n[PreProcess]: Storing pointcloud #" << id_);*/
      ROS_INFO_STREAM("              Dist to prev: " << dist << " m.");
      // ROS_INFO_STREAM("              Cloud size: " << pcl_cloud->size());
      // ROS_INFO_STREAM("              Kp size: " << kp_size);
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
    string navsts_file  = params_.outdir + "/" + NAVSTS_FILE;
    string latloncorner_file  = params_.outdir + "/" + LATLONCORNERS_FILE;
    remove(cm_file.c_str());
    remove(odom_file.c_str());
    remove(map_file.c_str());
    remove(navsts_file.c_str());
    remove(latloncorner_file.c_str());

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

    f_odom << fixed <<
    setprecision(10) <<
    Utils::id2str(id) << "," <<
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


  void PreProcess::storeLatLonCorners(string filename, int id, double stamp, 
                                      float lat, float lon, float h, 
                                      float latiupright, float lonupright, float hupright, 
                                      float latupleft, float lonupleft, float hupleft, 
                                      float latdownright, float londownright, float hdownright, 
                                      float latdownleft, float londownleft, float hdownleft, 
                                      float latmiddleup, float lonmiddleup, float hmiddleup,
                                      bool init)
  {
    string latlon_file = params_.outdir + "/" + filename;
    fstream f_latlon_file(latlon_file.c_str(), ios::out | ios::app);

    if (init == true){

      f_latlon_file << "#id" << "," << "stamp" << ","
      << "latitude" << "," << "longitude" << "," << "altitude" << ","
      << "uprightlat" << "," << "uprightlon" << "," << "uprighth" << ","
      << "upleftlat" << "," << "upleftlon" << "," << "uplefth" << ","
      << "downrightlat" << "," << "downrightlon" << "," << "downrighth" << ","
      << "downleftlat" << "," << "downleftlon" << "," << "downlefth" << ","
      << "middleuplat" << "," << "middleuplon" << "," << "middleuph" << "\n" ;

    } else {

      f_latlon_file << fixed <<
      setprecision(20) <<
      Utils::id2str(id) << "," <<
      stamp << "," <<

      lat << "," <<
      lon << "," << 
      h << "," << 

      latiupright << "," <<
      lonupright << "," << 
      hupright << "," << 

      latupleft << "," <<
      lonupleft << "," << 
      hupleft << "," << 

      latdownright << "," <<
      londownright << "," << 
      hdownright << "," <<

      latdownleft << "," <<
      londownleft << "," << 
      hdownleft << "," << 
      
      latmiddleup << "," <<
      lonmiddleup << "," <<
      hmiddleup << "," << endl;

    }
    f_latlon_file.close();
  }

  void PreProcess::storeNavSts(string filename, int id, double stamp, float lat, float lon, float h, bool init)
  {
    string navstatus_file = params_.outdir + "/" + filename;
    fstream f_navsts(navstatus_file.c_str(), ios::out | ios::app);

    if (init == true) {

      f_navsts << "#img_name" << "," << "latitude" << "," << "longitude" << "," << "altitude" << "," << "stamp" << "\n" ;
      
    } else {

      f_navsts << fixed << setprecision(20) << Utils::id2str(id) + ".jpg" << "," <<
      lat << "," << lon << "," << h << "," << stamp << "," << "\n" ;

    }

    f_navsts.close() ;
  }


  int PreProcess::storeImages(cv::Mat l_img, cv::Mat r_img, double stamp)
  {
    // Extract ORB // commented by fbf 21/01/2021. Uncomment to store yaml file with image key points. 
    /*orb_utils::Frame frame = orb_utils::Frame(l_img, r_img, stamp,
                                              l_ORB_extractor_,
                                              r_ORB_extractor_,
                                              camera_matrix_,
                                              baseline_);
    vector<cv::KeyPoint> l_kp, r_kp;
    cv::Mat l_desc, r_desc;
    vector<cv::Point3d> world_points;
    frame.GetLeftRightMatchings(l_kp, r_kp, l_desc, r_desc, world_points, stereo_camera_model_, params_.epipolar_th);

    // Store 
    cv::FileStorage fs(params_.outdir + "/" + IMG_DIR + "/" + Utils::id2str(id_) + ".yaml", cv::FileStorage::WRITE);
    write(fs, "l_kp", l_kp);
    write(fs, "r_kp", r_kp);
    write(fs, "l_desc", l_desc);
    write(fs, "r_desc", r_desc);
    write(fs, "world_points", world_points);

    fs.release();
*/
    cv::imwrite(params_.outdir + "/" + IMG_DIR + "/" + Utils::id2str(id_) + ".jpg", l_img);
    return 1;
   
// return l_kp.size();
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


  tf::Transform PreProcess::data2Tf(double  W, double  Y, double roll1, double pitch1, double yaw1){
      tf::Quaternion odom_quat2; 
      odom_quat2.setRPY(roll1, pitch1, yaw1); // set the quaternion from the roll,pitch and yaw of the NavSts NED orientation
      odom_quat2.normalize();
      geometry_msgs::Quaternion odom_quat_tmp3; 
      tf::quaternionTFToMsg(odom_quat2, odom_quat_tmp3);
      
  
      tf::Vector3 tf_translation(W, Y, 0);
      tf::Quaternion tf_orientation(odom_quat_tmp3.x, odom_quat_tmp3.y, odom_quat_tmp3.z, odom_quat_tmp3.w);
      tf::Transform uprightcor(tf_orientation, tf_translation);
      return uprightcor; 
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


  nav_msgs::Odometry PreProcess::Tf2odom(tf::Transform Tf)
  {
    // Get the data

    tf::Quaternion q_in = Tf.getRotation(); // get transform orientation in quaternion
    tf::Vector3 t_in = Tf.getOrigin(); // get the transform translation in a Vector3

    nav_msgs::Odometry odometry;  

    odometry.pose.pose.position.x=t_in.getX(); // get x, y and z translation, NED position
    odometry.pose.pose.position.y=t_in.getY();
    odometry.pose.pose.position.z=t_in.getZ();
    geometry_msgs::Quaternion odom_quat_tmp2; 
    ROS_INFO_STREAM("[PreProcess]: NED odometry transformed into left optical. North: " << odometry.pose.pose.position.x <<" East: " <<  odometry.pose.pose.position.y );
    tf::quaternionTFToMsg(q_in, odom_quat_tmp2); // convert the tf quaternion into a geometry message quaternion 
    odometry.pose.pose.orientation=odom_quat_tmp2; // assign this quaternion to the odo. orientation
    return odometry;

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


  PreProcess::LatLonAltitude PreProcess::ned_tf2Geo(tf::Transform nav_tf)
  {
    LatLonAltitude s ;
    nav_msgs::Odometry odom = Tf2odom(nav_tf) ;

    ned_->ned2Geodetic(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, s.lat, s.lon, s.h) ;
    
    return s ;
  }

/* fbf commentted on 18/01/2021 uncomment of PC storage
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
  }*/

}
