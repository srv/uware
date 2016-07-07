#include <ros/ros.h>
#include <ros/package.h>

#include "constants.h"
#include "edge_info.h"
#include "pose_info.h"
#include "registration.h"
#include "frame_to_frame.h"
#include "loop_closing.h"
#include "graph.h"

using namespace uware;

/** \brief Main entry point
  */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "uware_reconstruction");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  // Read parameters
  Registration::Params reg_params;
  FrameToFrame::Params ftf_params;
  LoopClosing::Params lc_params;
  nhp.param("preprocess_dir",     ftf_params.indir,             ros::package::getPath("uware") + "/" + PREPROCESS_DIR);
  nhp.param("outdir",             ftf_params.outdir,            ros::package::getPath("uware") + "/" + RECONSTRUCTION_DIR);
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

  Registration* registration = new Registration();
  registration->setParams(reg_params);

  ROS_INFO("[Registration]: -----------------------------------------------------------");
  ROS_INFO("[Registration]:                    FRAME TO FRAME                          ");
  ROS_INFO("[Registration]: -----------------------------------------------------------");

  // Frame to frame registration
  FrameToFrame frame_to_frame(registration);
  frame_to_frame.setParams(ftf_params);

  vector<PoseInfo> poses;
  vector<EdgeInfo> edges;
  frame_to_frame.compute(poses, edges);

  ROS_INFO("[Registration]: -----------------------------------------------------------");
  ROS_INFO("[Registration]:                      LOOP CLOSING                          ");
  ROS_INFO("[Registration]: -----------------------------------------------------------");

  // Loop closing registration
  LoopClosing loop_closing(registration);
  loop_closing.setParams(lc_params);
  loop_closing.compute(poses, edges);

  // Graph optimization
  Graph graph;
  graph.optimize(poses, edges);


  ros::spin();
  ros::shutdown();

  return 0;
}