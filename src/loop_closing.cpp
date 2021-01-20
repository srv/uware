#include <ros/ros.h>

#include "loop_closing.h"

namespace uware
{
  LoopClosing::LoopClosing(Registration* reg) : reg_(reg) { }

  void LoopClosing::compute(vector<PoseInfo> poses, vector<EdgeInfo> edges)
  {
    // Read camera matrix
    cv::Mat camera_matrix;
    ROS_INFO_STREAM("params indir:" << params_.indir << "/" << CAMERA_MATRIX_FILE);
    cv::FileStorage fs(params_.indir + "/" + CAMERA_MATRIX_FILE, cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs.release();
    ROS_INFO_STREAM("camera matrix readed" << poses.size());
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
    ROS_INFO_STREAM("[Reconstruction]: bucle acabat");
    // Store optimized poses
    string vertices_file  = params_.outdir + "/" + OPTIMIZED_POSES;
    string edges_file     = params_.outdir + "/" + OPTIMIZED_EDGES;
    fstream f_vertices(vertices_file.c_str(), ios::out | ios::trunc);
    fstream f_edges(edges_file.c_str(), ios::out | ios::trunc);
    f_vertices << "% timestamp, frame id, x, y, z, qx, qy, qz, qw" << endl;
    for (uint i=0; i<poses.size(); i++)
    {
      // Get stamp
      double stamp = getFrameStamp(poses[i].name);

      f_vertices << fixed <<
      setprecision(6) <<
      stamp << "," <<
      boost::lexical_cast<int>(poses[i].name) << "," <<
      poses[i].pose.getOrigin().x() << "," <<
      poses[i].pose.getOrigin().y() << "," <<
      poses[i].pose.getOrigin().z() << "," <<
      poses[i].pose.getRotation().x() << "," <<
      poses[i].pose.getRotation().y() << "," <<
      poses[i].pose.getRotation().z() << "," <<
      poses[i].pose.getRotation().w() <<  endl;
    }
    f_vertices.close();
    f_edges << "% name_a, name_b, valid_sim3, valid_icp, sim3_inliers, icp_score" << endl;
    for (uint i=0; i<edges.size(); i++)
    {
      f_edges << fixed <<
      setprecision(6) <<
      edges[i].name_a << "," <<
      edges[i].name_b << "," <<
      edges[i].valid_sim3 << "," <<
      edges[i].valid_icp << "," <<
      edges[i].sim3_inliers << "," <<
      edges[i].icp_score <<  endl;
    }
    f_edges.close();

    // Store homographies
    /*Utils::createDir("/home/xesc/workspace/ros/src/uware/reconstruction/homographies");
    for (uint i=0; i<poses.size(); i++)
    {
      tf::Matrix3x3 mbase = poses[i].pose.getBasis();
      double yaw, dummy_1, dummy_2;
      mbase.getRPY(dummy_1, dummy_2, yaw);
      cv::Mat H = (cv::Mat_<double>(3,3) << cos(yaw), -sin(yaw), poses[i].pose.getOrigin().x(), sin(yaw),  cos(yaw), poses[i].pose.getOrigin().y(), 0, 0, 1.0);
      cv::FileStorage fs2("/home/xesc/workspace/ros/src/uware/reconstruction/homographies/" + Utils::id2str(i) + ".yaml", cv::FileStorage::WRITE);
      cv::write(fs2, "filename", "/tmp/mosaicing/images/" + Utils::id2str(i) + ".jpg");
      cv::write(fs2, "HP", H);
      fs2.release();
    }*/

    ROS_INFO_STREAM("[Reconstruction]: Total loop closings: " << lc_counter);
  }

  double LoopClosing::getFrameStamp(string name)
  {
    string file = params_.indir + "/" + ODOM_FILE;

    // Get the pointcloud poses file
    ifstream poses_file(file.c_str());
    string line;
    while (getline(poses_file, line))
    {
      int i = 0;
      double stamp = -1.0;
      string cloud_name, value;
      istringstream ss(line);
      while(getline(ss, value, ','))
      {
        if (i == 0)
          cloud_name = value;
        else if (i == 1)
        {
          stamp = boost::lexical_cast<double>(value);
          if (cloud_name == name)
            return stamp;
        }
        i++;
      }
    }

    return -1.0;
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