#include <ros/ros.h>

#include "loop_closing.h"

namespace uware
{
  LoopClosing::LoopClosing(Registration* reg) : reg_(reg) { }

  void LoopClosing::compute(vector<PoseInfo> poses, vector<EdgeInfo> edges)
  {
    // Read camera matrix
    cv::Mat camera_matrix;
    cv::FileStorage fs(params_.indir + "/" + CAMERA_MATRIX_FILE, cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs.release();

    int lc_counter = 0;
    for (uint i=0; i<poses.size(); i++)
    {
      ROS_INFO_STREAM("[Registration]: Searching LC for node " << i << " of " << poses.size());

      tf::Transform tf_curr = poses[i].pose;
      vector<uint> closest_poses = getClosestPoses(poses, i, params_.n_best, params_.discard_window, true);

      for (uint j=0; j<closest_poses.size(); j++)
      {
        // Find loop closures

        // Init
        bool valid_sim3 = false;
        bool valid_icp = false;
        int sim3_inliers = 0;
        double icp_score = 0.0;

        tf::Transform tf_cand = poses[closest_poses[j]].pose;
        tf::Transform cand2curr = tf_cand.inverse() * tf_curr;

        // Registration
        tf::Transform out;
        reg_->pipeline(closest_poses[j], i, cand2curr, valid_sim3, valid_icp, sim3_inliers, icp_score, camera_matrix, out);

        if (valid_sim3 || valid_icp)
        {
          ROS_INFO("[Registration]: ------------ LOOP CLOSING ------------");
          ROS_INFO_STREAM("[Registration]: Between: " << closest_poses[j] << " and " << i);
          ROS_INFO_STREAM("[Registration]: Sim3: " << valid_sim3 << " (inliers: " << sim3_inliers << ").");
          ROS_INFO_STREAM("[Registration]: ICP: " << valid_icp << " (score: " << icp_score << ").");
          ROS_INFO("[Registration]: --------------------------------------");

          EdgeInfo e(Utils::id2str(closest_poses[j]), Utils::id2str(i), valid_sim3, valid_icp, sim3_inliers, icp_score);
          edges.push_back(e);

          lc_counter++;
        }
      }
    }

    ROS_INFO_STREAM("[Registration]: Total loop closings: " << lc_counter);
  }

  vector<uint> LoopClosing::getClosestPoses(vector<PoseInfo> poses, uint id, int n_best, int discard_window, bool xy)
  {
    // Init
    vector< pair<uint,double> > distances;
    tf::Transform current = poses[id].pose;

    for (uint i=0; i<poses.size(); i++)
    {
      // Do not take into account the neighbors into the discard window.
      if (abs((int)i-(int)id) <= (int)discard_window)
        continue;

      double dist = Utils::tfDist(current, poses[i].pose, xy);
      distances.push_back(make_pair(i,dist));
    }

    // Sort by distance
    sort(distances.begin(), distances.end(), Utils::sortByDistance);

    vector<uint> out;
    for (uint i=0; i<(uint)n_best; i++)
      out.push_back(distances[i].first);

    return out;
  }

} // namespace