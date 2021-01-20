#include <ros/ros.h>
#include <ros/package.h>

#include "constants.h"
#include "edge_info.h"
#include "pose_info.h"
#include "registration.h"
#include "frame_to_frame.h"
#include "loop_closing.h"
#include "build3d.h"

using namespace uware;

/** \brief Main entry point
  */
void storeOdometry(string filename, int id, string outdir, tf::Transform odometry);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "uware_reconstruction");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Read parameters
  FrameToFrame::Params ftf_params;
  LoopClosing::Params lc_params;
  Registration::Params reg_params;
  Build3D::Params b3d_params;
  nhp.param("preprocess_dir",     ftf_params.indir,             ros::package::getPath("uware") + "/" + PREPROCESS_DIR);
  nhp.param("outdir",             ftf_params.outdir,            ros::package::getPath("uware") + "/" + RECONSTRUCTION_DIR);
  nhp.param("start_img_name",     ftf_params.start_img_name,    -1);
  nhp.param("stop_img_name",      ftf_params.stop_img_name,     -1);
  nhp.param("show_generic_logs",  ftf_params.show_generic_logs, false);

  nhp.param("preprocess_dir",     lc_params.indir,              ros::package::getPath("uware") + "/" + PREPROCESS_DIR);
  nhp.param("discard_window",     lc_params.discard_window,     5);
  nhp.param("n_best",             lc_params.n_best,             5);

  nhp.param("preprocess_dir",     reg_params.indir,             ros::package::getPath("uware") + "/" + PREPROCESS_DIR);
  nhp.param("outdir",             reg_params.outdir,            ros::package::getPath("uware") + "/" + RECONSTRUCTION_DIR);
  nhp.param("desc_matching_th",   reg_params.desc_matching_th,  0.7);
  nhp.param("min_desc_matches",   reg_params.min_desc_matches,  50);
  nhp.param("reproj_err",         reg_params.reproj_err,        4.0);
  nhp.param("min_inliers",        reg_params.min_inliers,       20);
  nhp.param("max_reg_err",        reg_params.max_reg_err,       0.3);
  nhp.param("show_icp_score",     reg_params.show_icp_score,    false);
  nhp.param("show_icp_tf",        reg_params.show_icp_tf,       false);
  nhp.param("show_num_of_kp",     reg_params.show_num_of_kp,    false);
  nhp.param("z_diff_th",          reg_params.z_diff_th,         15.0);
  nhp.param("min_salient_ids",    reg_params.min_salient_ids,   15.0);
  nhp.param("show_generic_logs",  reg_params.show_generic_logs, false);
  nhp.param("save_icp_clouds",    reg_params.save_icp_clouds,   false);

  nhp.param("preprocess_dir",     b3d_params.indir,             ros::package::getPath("uware") + "/" + PREPROCESS_DIR);
  nhp.param("outdir",             b3d_params.outdir,            ros::package::getPath("uware") + "/" + RECONSTRUCTION_DIR);
  nhp.param("outdir",             b3d_params.voxel_resolution,  0.01);

  Registration* registration = new Registration();
  registration->setParams(reg_params);

  ROS_INFO("[Reconstruction]: -----------------------------------------------------------");
  ROS_INFO("[Reconstruction]:                    FRAME TO FRAME                          ");
  ROS_INFO("[Reconstruction]: -----------------------------------------------------------");

  // Frame to frame registration
  FrameToFrame frame_to_frame(registration);
  frame_to_frame.setParams(ftf_params);

  vector<PoseInfo> poses;
  vector<EdgeInfo> edges;
  frame_to_frame.compute(poses, edges);

  ROS_INFO("[Reconstruction]: -----------------------------------------------------------");
  ROS_INFO("[Reconstruction]:                      LOOP CLOSING                          ");
  ROS_INFO("[Reconstruction]: -----------------------------------------------------------");

  // Loop closing registration
  LoopClosing loop_closing(registration);

  loop_closing.setParams(lc_params);

  loop_closing.compute(poses, edges);// fbf 19/10/2017 next we store the matrices with all the poses in the hard disc

  // -------------------------------- //fbf 19/10/2017 odom is a TF
    ROS_INFO_STREAM("get in the loop, outdir: " << reg_params.outdir);
    Utils::createDir(reg_params.outdir + "/homographies/");
      for (uint p=0; p<poses.size(); p++)
      {
      tf::Transform odom = poses[p].pose;
      tf::Matrix3x3 obase = odom.getBasis();
      double oyaw, dummy_1, dummy_2;
      obase.getRPY(dummy_1, dummy_2, oyaw);
      cv::Mat HP = (cv::Mat_<double>(3,3) << cos(oyaw), -sin(oyaw), odom.getOrigin().x(), sin(oyaw),  cos(oyaw), odom.getOrigin().y(), 0, 0, 1.0);
      ROS_INFO("create the file in /homographies");

      cv::FileStorage fs(reg_params.outdir + "/homographies/" + Utils::id2str(p) + ".yaml", cv::FileStorage::WRITE);
      cv::write(fs, "filename", "/tmp/mosaicing/images/" + Utils::id2str(p) + ".jpg");
      cv::write(fs, "HP", HP);
      fs.release();

      storeOdometry(ODOM_FILE, p, reg_params.outdir, odom);

      // -------------------------------- end fbf 19/10/2017
       }


  ROS_INFO("[Reconstruction]: -----------------------------------------------------------");
  ROS_INFO("[Reconstruction]:                        BUILD 3D                            ");
  ROS_INFO("[Reconstruction]: -----------------------------------------------------------");
 // fbf 19/10/2017 commented the 3D reconstruction. Only interested in the optimized poses
  // Build 3D
//  Build3D build3d;
 // build3d.setParams(b3d_params);
//  build3d.build(poses);

  ROS_INFO("[Reconstruction]: -----------------------------------------------------------");
  ROS_INFO("[Reconstruction]:                        FINISHED!                           ");
  ROS_INFO("[Reconstruction]: -----------------------------------------------------------");

  ros::shutdown();

  return 0;
}

void storeOdometry(string filename, int id, string outdir,  tf::Transform odometry)
  {
    string odom_file = outdir + "/" + filename;
    fstream f_odom(odom_file.c_str(), ios::out | ios::app);

    f_odom << fixed <<
    setprecision(6) <<
    Utils::id2str(id) << "," <<
    
    odometry.getOrigin().x() << "," <<
    odometry.getOrigin().y() << "," <<
    odometry.getOrigin().z() << "," <<
    odometry.getRotation().x() << "," <<
    odometry.getRotation().y() << "," <<
    odometry.getRotation().z() << "," <<
    odometry.getRotation().w() <<  endl;

    f_odom.close();
  }