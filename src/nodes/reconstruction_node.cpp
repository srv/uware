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
  nhp.param("preprocess_dir",   ftf_params.indir,             ros::package::getPath("uware") + "/" + PREPROCESS_DIR);
  nhp.param("outdir",           ftf_params.outdir,            ros::package::getPath("uware") + "/" + RECONSTRUCTION_DIR);
  nhp.param("z_diff_th",        ftf_params.z_diff_th,         15.0);
  nhp.param("min_salient_ids",  ftf_params.min_salient_ids,   15.0);

  nhp.param("show_salient_ids", ftf_params.show_salient_ids,  false);
  nhp.param("show_icp_score",   ftf_params.show_icp_score,    false);
  nhp.param("show_icp_tf",      ftf_params.show_icp_tf,       false);

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