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
      ROS_INFO_STREAM("[Reconstruction]: Searching LC for node " << i << " of " << poses.size());

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

        uint src = lexical_cast<uint>(poses[closest_poses[j]].name);
        uint tgt = lexical_cast<uint>(poses[i].name);

        // Registration
        tf::Transform out;
        reg_->pipeline(src, tgt, cand2curr, valid_sim3, valid_icp, sim3_inliers, icp_score, camera_matrix, true, out);

        if (valid_sim3 || valid_icp)
        {
          ROS_INFO("[Reconstruction]: ------------ LOOP CLOSING ------------");
          ROS_INFO_STREAM("[Reconstruction]: Between: " << closest_poses[j] << " and " << i);
          ROS_INFO_STREAM("[Reconstruction]: Sim3: " << valid_sim3 << " (inliers: " << sim3_inliers << ").");
          ROS_INFO_STREAM("[Reconstruction]: ICP: " << valid_icp << " (score: " << icp_score << ").");
          ROS_INFO("[Reconstruction]: --------------------------------------");

          // Check if this edge already exists
          bool insert = true;
          string a = poses[closest_poses[j]].name;
          string b = poses[i].name;
          for (uint n=0; n<edges.size(); n++)
          {
            EdgeInfo e = edges[n];
            if ((a == e.name_a && b == e.name_b) || (b == e.name_a && a == e.name_b))
            {
              // Edge found
              if (e.valid_sim3 && valid_sim3)
              {
                if (e.sim3_inliers > sim3_inliers)
                  insert = false;
              }
              if (e.valid_icp && valid_icp)
              {
                if (e.icp_score < icp_score)
                  insert = false;
              }
            }
          }

          // Insert edge if it is better than the previous for the same nodes
          if (insert)
          {
            EdgeInfo e(a, b, valid_sim3, valid_icp, sim3_inliers, icp_score, out);
            edges.push_back(e);

            // Update the graph and continue
            graph_.optimize(poses, edges);
          }

          lc_counter++;
        }
      }
    }

    // TODO! Search loop closings with libhaloc


    ROS_INFO_STREAM("[Reconstruction]: Total loop closings: " << lc_counter);
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

    uint min_size = (uint)n_best;
    if (min_size > distances.size())
      min_size = distances.size();

    vector<uint> out;
    for (uint i=0; i<min_size; i++)
      out.push_back(distances[i].first);

    return out;
  }

} // namespace