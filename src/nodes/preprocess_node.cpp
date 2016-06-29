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
  string odom_topic, map_topic, camera_topic, outdir;
  nhp.param("odom_topic",       odom_topic,                 string(""));
  nhp.param("map_topic",        map_topic,                  string(""));
  nhp.param("camera_topic",     camera_topic,               string(""));
  nhp.param("outdir",           params.outdir,              ros::package::getPath("uware") + "/" + PREPROCESS_DIR);
  nhp.param("min_cloud_size",   params.min_cloud_size,      100);
  nhp.param("store_distance",   params.store_distance,      0.5);
  nhp.param("use_2d_distance",  params.use_2d_distance,     false);
  nhp.param("voxel_resolution", params.voxel_resolution,    0.02);
  nhp.param("epipolar_th",      params.epipolar_th,         1.5);


  // Init the node
  PreProcess node;
  node.setParams(params);

  // Message sync
  image_transport::ImageTransport it(nh);
  image_transport::SubscriberFilter left_sub, right_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub, right_info_sub;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
  message_filters::Subscriber<nav_msgs::Odometry> map_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,
                                                          nav_msgs::Odometry,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo,
                                                          sensor_msgs::CameraInfo,
                                                          sensor_msgs::PointCloud2> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;

  // Setup the sync
  boost::shared_ptr<Sync> sync;
  odom_sub      .subscribe(nh, odom_topic, 20);
  map_sub       .subscribe(nh, map_topic, 20);
  left_sub      .subscribe(it, camera_topic+"/left/image_rect_color", 5);
  right_sub     .subscribe(it, camera_topic+"/right/image_rect_color", 5);
  left_info_sub .subscribe(nh, camera_topic+"/left/camera_info",  5);
  right_info_sub.subscribe(nh, camera_topic+"/right/camera_info", 5);
  cloud_sub     .subscribe(nh, camera_topic+"/points2", 5);

  // Sync callback
  sync.reset(new Sync(SyncPolicy(10), odom_sub, map_sub, left_sub, right_sub, left_info_sub, right_info_sub, cloud_sub) );
  sync->registerCallback(bind(&PreProcess::callback, &node, _1, _2, _3, _4, _5, _6, _7));

  ros::spin();
  ros::shutdown();

  return 0;
}