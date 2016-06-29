#include <ros/ros.h>
#include <ros/package.h>

#include "constants.h"
#include "frame_to_frame.h"

using namespace uware;

/** \brief Main entry point
  */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "uware_reconstruction");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  // Read parameters
  FrameToFrame::Params ftf_params;
  nhp.param("preprocess_dir",     ftf_params.indir,             ros::package::getPath("uware") + "/" + PREPROCESS_DIR);
  nhp.param("outdir",             ftf_params.outdir,            ros::package::getPath("uware") + "/" + RECONSTRUCTION_DIR);
  nhp.param("z_diff_th",          ftf_params.z_diff_th,         15.0);
  nhp.param("min_salient_ids",    ftf_params.min_salient_ids,   15.0);
  nhp.param("max_reg_err",        ftf_params.max_reg_err,       0.3);
  nhp.param("desc_matching_th",   ftf_params.desc_matching_th,  0.7);
  nhp.param("min_desc_matches",   ftf_params.min_desc_matches,  50);
  nhp.param("reproj_err",         ftf_params.reproj_err,        4.0);
  nhp.param("min_inliers",        ftf_params.min_inliers,       20);

  nhp.param("show_generic_logs",  ftf_params.show_generic_logs, false);
  nhp.param("show_salient_ids",   ftf_params.show_salient_ids,  false);
  nhp.param("show_icp_score",     ftf_params.show_icp_score,    false);
  nhp.param("show_icp_tf",        ftf_params.show_icp_tf,       false);
  nhp.param("save_icp_clouds",    ftf_params.save_icp_clouds,   false);
  nhp.param("show_num_of_kp",     ftf_params.show_num_of_kp,    false);


  // Frame to frame registration
  FrameToFrame frame_to_frame;
  frame_to_frame.setParams(ftf_params);
  frame_to_frame.compute();

  // Loop closing registration
  // LoopClosing loop_closing;
  // loop_closing.setParams(params);
  // loop_closing.compute();

  ros::spin();
  ros::shutdown();

  return 0;
}