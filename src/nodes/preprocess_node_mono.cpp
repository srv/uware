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
    string img_topic, info_topic, outdir, navstatus_topic;
    int aprox_time ;
    nhp.param("img_topic",            img_topic,                  string(""));
    nhp.param("info_topic",           info_topic,                 string(""));
    nhp.param("navstatus_topic",      navstatus_topic,            string(""));
    nhp.param("approximate_time",     aprox_time,                 20);
    nhp.param("outdir",               params.outdir,              ros::package::getPath("uware") + "/" + PREPROCESS_DIR);
    nhp.param("base_link_frame_id",   params.base_link_frame_id,  string("/turbot/base_link"));
    nhp.param("store_distance",       params.store_distance,      0.5);
    nhp.param("use_2d_distance",      params.use_2d_distance,     false);
    nhp.param("voxel_resolution",     params.voxel_resolution,    0.01);
    nhp.param("epipolar_th",          params.epipolar_th,         1.5);
    nhp.param("max_altitude",         params.max_altitude,        4.5);

    // Init the node
    PreProcess node;
    node.setParams(params);

    // Message sync
    image_transport::ImageTransport it(nh);
    image_transport::SubscriberFilter img_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
    message_filters::Subscriber<cola2_msgs::NavSts> navstatus_sub;


    ////sensor_msgs::PointCloud2.
    /// syncro odometries, images, info-images, altitude and the nav status
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                            sensor_msgs::CameraInfo,
                                                            cola2_msgs::NavSts> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;

    // Setup the sync
    boost::shared_ptr<Sync> sync;
    img_sub       .subscribe(it, img_topic, 5);
    info_sub      .subscribe(nh, info_topic,  5);
    navstatus_sub .subscribe(nh, navstatus_topic, 20);

    // Sync callback for all syncronized topics 
    sync.reset(new Sync(SyncPolicy(aprox_time), img_sub, info_sub, navstatus_sub) );
    
    sync->registerCallback(bind(&PreProcess::mono_callback, &node, _1, _2, _3));

    ROS_INFO_STREAM("img_topic: " << img_topic);
    ROS_INFO_STREAM("info_topic: " << info_topic);
    ROS_INFO_STREAM("navstatus_topic: " << navstatus_topic);
    ROS_INFO_STREAM("approximate_time: " << aprox_time) ;
    ROS_INFO_STREAM("outdir: " << params.outdir);
    ROS_INFO_STREAM("base_link_frame_id: " << params.base_link_frame_id);
    ROS_INFO_STREAM("store_distance: " << params.store_distance);
    ROS_INFO_STREAM("max_altitude: " << params.max_altitude) ;

    ros::spin();
    ros::shutdown();

    return 0;
}