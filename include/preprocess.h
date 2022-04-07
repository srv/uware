#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>
//#include <geometry_msgs/Quaternion>
#include <cola2_msgs/NavSts.h>
#include <cola2_msgs/NED.h>
#include <cola2_msgs/RPY.h>
#include <cola2_msgs/DecimalLatLon.h>
#include <cola2_msgs/VehicleStatus.h>
#include <ned_tools/ned.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <opencv2/opencv.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <eigen3/Eigen/Dense>


#include <orb_utils/Frame.h>
#include <orb_utils/ORBextractor.h>

#include "constants.h"
#include "utils.h"

using namespace std;
using namespace boost;
namespace fs  = filesystem;
namespace enc = sensor_msgs::image_encodings;

typedef pcl::PointXYZRGB          Point;
typedef pcl::PointCloud<Point>    PointCloud;

namespace uware
{

class PreProcess
{

public:

  struct Params
  {
    string outdir;            //!> Output directory
    int min_cloud_size;       //!> Minimum cloud size (number of points)
    double store_distance;    //!> Distance at which pointclouds are stored (m)
    bool use_2d_distance;     //!> Use 2D distance to calculate when store a new pointcloud
    double voxel_resolution;  //!> Voxel filter cloud resolution (m)
    double epipolar_th;       //!> Epipolar threshold for the stereo matching
    double max_altitude ;     //!> Under this altitude the program save information   

    // Default settings
    Params () {
      outdir            = "";
      min_cloud_size    = 100;
      store_distance    = 0.5;
      use_2d_distance   = false;
      voxel_resolution  = 0.01;
      epipolar_th       = 1.5;
      max_altitude      = 4.5 ;
    }
  };

  struct LatLonAltitude
  {
    double lat;
    double lon;
    double h ; 

    LatLonAltitude()
    {
      lat = 0 ;
      lon = 0 ;
      h = 0 ;
    }

  };
  
  /** \brief Set class params
   * \param the parameters struct
   */
  inline void setParams(const Params& params){params_ = params;}

  /** \brief Class constructor
   */
  PreProcess();

  /** \brief Messages callback. This function is called when synchronized odometry and image
   * message are received.
   * \param odom_msg ros odometry message of type nav_msgs::Odometry
   * \param l_img left stereo image message of type sensor_msgs::Image
   * \param r_img right stereo image message of type sensor_msgs::Image
   * \param l_info left stereo info message of type sensor_msgs::CameraInfo
   * \param r_info right stereo info message of type sensor_msgs::CameraInfo
   * \param pointcloud
   */// const sensor_msgs::PointCloud2ConstPtr& cloud_msg
  void callback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                const nav_msgs::Odometry::ConstPtr& map_msg,
                const sensor_msgs::ImageConstPtr& l_img_msg,
                const sensor_msgs::ImageConstPtr& r_img_msg,
                const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                const sensor_msgs::CameraInfoConstPtr& r_info_msg,
                const sensor_msgs::RangeConstPtr& altitude_msg,
                const cola2_msgs::NavSts::ConstPtr& navstatus_msg);


protected:

  /** \brief Create operational directories
   * @return true if all directories have been created
   */
  bool createDirs();

  /** \brief Calculate distance between poses
   * @return the euclidean distance
   * \param pose a
   * \param pose b
   * \param if true, only x and y axes are used to compute the distance
   */
  double calcDist(tf::Transform a, tf::Transform b, bool use_2d_distance);

  /** \brief Store odometry into file
   * @return
   * \param filename
   * \param measure id
   * \param timestamp
   * \param odometry message
   */
  void storeOdometry(string filename, int id, double stamp, tf::Transform odometry);

  void storeNavSts(string filename, int id, double stamp, float lat, float lon, float h, float latiupright, float lonupright, float hupright, float latupleft, float lonupleft, float hupleft, float latdownright, float londownright, float hdownright, float latdownleft, float londownleft, float hdownleft);

  void storeLatLonImages(string filename, int id, float lat, float lon, float h, bool init) ; 

  /** \brief Store image data into file
   * @return number of left keypoints
   * \param left image
   * \param right image
   * \param timestamp
   */
  int storeImages(cv::Mat l_img, cv::Mat r_img, double stamp);

  /** \brief Convert ros image to cv image
   * @return true if conversion was successfully
   * \param left input image
   * \param right input image
   * \param left output image
   * \param right output image
   */
  bool imgMsgToMat(sensor_msgs::Image l_img_msg,
                   sensor_msgs::Image r_img_msg,
                   cv::Mat &l_img, cv::Mat &r_img);

 /** \brief Convert the odometry message to TF
   * @return the transform
   * \param Odometry message
   */
  tf::Transform odom2Tf(nav_msgs::Odometry odom_msg);
  tf::Transform data2Tf(double  W, double  Y, double roll1, double pitch1, double yaw1);


  nav_msgs::Odometry Tf2odom(tf::Transform Tf);

  /** \brief Get the transform between odometry frame and camera frame
   * @return true if valid transform, false otherwise
   * \param Odometry frame id
   * \param Camera frame id
   * \param Output transform
   */
  bool getOdom2CameraTf(string odom_frame_id,
                        string camera_frame_id,
                        tf::StampedTransform &transform);

  LatLonAltitude ned2Geo(tf::Transform nav_tf) ;

  /** \brief Filters a pointcloud
   * @return filtered cloud
   * \param input cloud
   */
  /* fbf commentted on 18/01/2021 uncomment of PC storage
  PointCloud::Ptr filterCloud(PointCloud::Ptr in_cloud); */


private:

  Params params_; //!> Stores parameters.

  tf::StampedTransform odom2camera_; //!> Transformation between robot odometry frame and camera frame.

  tf::TransformListener tf_listener_; //!> Listen for tf between robot and camera.

  bool initialization_; //!> Initialization

  bool first_; //!> First iteration

  uint id_; //!> Frame id

  tf::Transform prev_odom_; //!> Previous odometry

  // ORB extractors
  orb_utils::ORBextractor* l_ORB_extractor_;
  orb_utils::ORBextractor* r_ORB_extractor_;

  cv::Mat camera_matrix_; //!> Camera matrix

  image_geometry::StereoCameraModel stereo_camera_model_; //!> Stereo camera model
  float baseline_; //!> Stereo baseline multiplied by fx.
                              //!< reference frame the AUV is in

  Ned* ned_;

};

} // namespace

#endif // PREPROCESS_H
