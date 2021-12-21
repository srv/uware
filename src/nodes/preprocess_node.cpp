#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include "preprocess.h"
#include "constants.h"

using namespace uware;

/** \brief Main entry point
  */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "uware_preprocess");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  // Read parameters
  PreProcess::Params params;
  string odom_topic, map_topic, altitude_topic, camera_left_topic, camera_right_topic, camera_left_info, camera_right_info, camera_topic_points2, outdir, navstatus_topic;
  nhp.param("odom_topic",       odom_topic,                 string(""));
  nhp.param("map_topic",        map_topic,                  string(""));
  nhp.param("camera_left_topic",     camera_left_topic,     string(""));
  nhp.param("camera_right_topic",     camera_right_topic,   string(""));
  nhp.param("altitude_topic",     altitude_topic,           string(""));
  nhp.param("camera_left_info",     camera_left_info,       string(""));
  nhp.param("camera_right_info",     camera_right_info,     string(""));
  nhp.param("camera_topic_points2",     camera_topic_points2,     string(""));
  nhp.param("outdir",           params.outdir,              ros::package::getPath("uware") + "/" + PREPROCESS_DIR);
  nhp.param("min_cloud_size",   params.min_cloud_size,      100);
  nhp.param("store_distance",   params.store_distance,      0.5);
  nhp.param("use_2d_distance",  params.use_2d_distance,     false);
  nhp.param("voxel_resolution", params.voxel_resolution,    0.01);
  nhp.param("epipolar_th",      params.epipolar_th,         1.5);
  nhp.param("navstatus_topic",      navstatus_topic,         string(""));


  // Init the node
  PreProcess node;
  node.setParams(params);

  // Message sync
  image_transport::ImageTransport it(nh);
  image_transport::SubscriberFilter left_sub, right_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub, right_info_sub;
  message_filters::Subscriber<sensor_msgs::Range> altitude_sub;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
  message_filters::Subscriber<nav_msgs::Odometry> map_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;
  message_filters::Subscriber<cola2_msgs::NavSts> navstatus_sub;


////sensor_msgs::PointCloud2.
  /// syncro odometries, images, info-images, altitude and the nav status
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,
                                                          nav_msgs::Odometry,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo,
                                                          sensor_msgs::CameraInfo,
                                                          sensor_msgs::Range,
                                                          cola2_msgs::NavSts> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;

  // Setup the sync
  boost::shared_ptr<Sync> sync;
  odom_sub      .subscribe(nh, odom_topic, 20);
  map_sub       .subscribe(nh, map_topic, 20);
  left_sub      .subscribe(it, camera_left_topic, 5);
  right_sub     .subscribe(it, camera_right_topic, 5);
  left_info_sub .subscribe(nh, camera_left_info,  5);
  right_info_sub.subscribe(nh, camera_right_info, 5);
  altitude_sub  .subscribe(nh, altitude_topic, 5);
  cloud_sub     .subscribe(nh, camera_topic_points2, 5);
  navstatus_sub     .subscribe(nh, navstatus_topic, 20);

  // Sync callback for all syncronized topics 
  sync.reset(new Sync(SyncPolicy(2500), odom_sub, map_sub, left_sub, right_sub, left_info_sub, right_info_sub, altitude_sub, navstatus_sub) );
  //sync.reset(new Sync(SyncPolicy(10), odom_sub, map_sub, left_sub, right_sub, left_info_sub, altitude_sub, right_info_sub, cloud_sub) );

  sync->registerCallback(bind(&PreProcess::callback, &node, _1, _2, _3, _4, _5, _6, _7, _8));

  ROS_INFO_STREAM("odom_topic " << odom_topic);
  ROS_INFO_STREAM("map_topic " << map_topic);
  ROS_INFO_STREAM("camera_left_topic " << camera_left_topic);
  ROS_INFO_STREAM("camera_right_topic " << camera_right_topic);
  ROS_INFO_STREAM("camera_left_info " << camera_left_info);
  ROS_INFO_STREAM("camera_right_info " << camera_right_info);
  ROS_INFO_STREAM("altitude_topic " << altitude_topic);
  ROS_INFO_STREAM("navstatus_topic " << navstatus_topic);
  ROS_INFO_STREAM("odom_topic " << params.outdir);

  ros::spin();
  ros::shutdown();

  return 0;
}
