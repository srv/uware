#include <ros/ros.h>

#include "frame_to_frame.h"

namespace uware
{
  FrameToFrame::FrameToFrame(Registration* reg) : first_(true), reg_(reg) { }

  void FrameToFrame::compute(vector<PoseInfo>& poses, vector<EdgeInfo>& edges)
  {
    // Create directories
    if (!createDirs()) return;

    // Read poses
    Utils::readPoses(params_.indir + "/" + ODOM_FILE, odom_cloud_poses_);
    Utils::readPoses(params_.indir + "/" + OMAP_FILE, map_cloud_poses_);

    // Read camera matrix
    cv::Mat camera_matrix;
    cv::FileStorage fs(params_.indir + "/" + CAMERA_MATRIX_FILE, cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs.release();

    // Sort directory of pointclouds by name
    typedef std::vector<fs::path> vec;
    vec v;
    copy(
          fs::directory_iterator(params_.indir + "/" + PC_DIR),
          fs::directory_iterator(),
          back_inserter(v)
        );

    sort(v.begin(), v.end());
    vec::const_iterator it(v.begin());

    // Iterate over all pointclouds
    while (it!=v.end())
    {
      // Check if the directory entry is an directory.
      if (!fs::is_directory(*it))
      {
        // Get the filename
        string filename = it->filename().string();
        string filepath = params_.indir + "/" + PC_DIR + "/" + filename;
        int lastindex = filename.find_last_of(".");
        string rawname = filename.substr(0, lastindex);

        // Handle start/stop limits
        if (params_.start_img_name >= 0)
        {
          int current = lexical_cast<int>(rawname);
          if (current < params_.start_img_name)
          {
            it++;
            continue;
          }
        }
        if (params_.stop_img_name >= 0)
        {
          int current = lexical_cast<int>(rawname);
          if (current > params_.stop_img_name)
          {
            it++;
            continue;
          }
        }

        ROS_INFO_STREAM("[Reconstruction]: Processing cloud: " << filename);

        // Search current pointcloud into the cloud poses
        uint id = getCloudPoseId(rawname, odom_cloud_poses_);
        if (id < 0)
        {
          ROS_ERROR_STREAM("[Reconstruction]: Impossible to find the id into the odom_cloud_poses_ array for file: " << filename);
          break;
        }

        // First iteration
        if (first_)
        {
          prev_pose_ = map_cloud_poses_[id].second;
          PoseInfo p(rawname, prev_pose_);
          poses_result_.push_back(p);
          saveResult();

          first_ = false;
          it++;
          continue;
        }

        // Init
        bool valid_sim3 = false;
        bool valid_icp = false;
        int sim3_inliers = 0;
        double icp_score = 0.0;


        // Estimated movement by odometry
        tf::Transform tf_prev = odom_cloud_poses_[id-1].second;
        tf::Transform tf_curr = odom_cloud_poses_[id].second;
        tf::Transform prev2curr = tf_prev.inverse() * tf_curr;

        // Registration
        tf::Transform out;
        reg_->pipeline(id-1, id, prev2curr, valid_sim3, valid_icp, sim3_inliers, icp_score, camera_matrix, false, out);

        // Switch between registration and map tf
        tf::Transform curr_pose;
        if (valid_sim3 || valid_icp)
         curr_pose = prev_pose_ * out;
        else
          curr_pose = map_cloud_poses_[id].second;

        // Save the result
        EdgeInfo e(Utils::id2str(id-1), rawname, valid_sim3, valid_icp, sim3_inliers, icp_score, prev2curr);
        edges_result_.push_back(e);
        PoseInfo p(rawname, curr_pose);
        poses_result_.push_back(p);
        saveResult();

        // Copy
        prev_pose_ = curr_pose;

      }
      // Next directory entry
      it++;
    }
    poses = poses_result_;
    edges = edges_result_;
  }

  bool FrameToFrame::createDirs()
  {
    if (!Utils::createDir(params_.outdir))
      return false;

    if (!Utils::createDir(params_.outdir + "/" + PAIRED_CLOUDS_DIR))
      return false;

    // Remove files
    string poses_file   = params_.outdir + "/" + F2F_POSES;
    remove(poses_file.c_str());
    string edges_file   = params_.outdir + "/" + F2F_EDGES;
    remove(edges_file.c_str());

    return true;
  }

  int FrameToFrame::getCloudPoseId(string rawname, vector< pair<string, tf::Transform> > odom_cloud_poses_)
  {
    for (uint i=0; i<odom_cloud_poses_.size(); i++)
    {
      if (odom_cloud_poses_[i].first == rawname)
        return i;
    }
    return -1;
  }

  void FrameToFrame::saveResult()
  {
    if (poses_result_.size()>0)
    {
      int idx = (int)poses_result_.size()-1;
      string result_file = params_.outdir + "/" + F2F_POSES;
      fstream f_res(result_file.c_str(), ios::out | ios::app);

      f_res << fixed <<
      setprecision(6) <<
      poses_result_[idx].name << "," <<
      poses_result_[idx].pose.getOrigin().x() << "," <<
      poses_result_[idx].pose.getOrigin().y() << "," <<
      poses_result_[idx].pose.getOrigin().z() << "," <<
      poses_result_[idx].pose.getRotation().x() << "," <<
      poses_result_[idx].pose.getRotation().y() << "," <<
      poses_result_[idx].pose.getRotation().z() << "," <<
      poses_result_[idx].pose.getRotation().w() <<  endl;

      f_res.close();
    }

    if (edges_result_.size()>0)
    {
      int idx = (int)edges_result_.size()-1;
      string result_file = params_.outdir + "/" + F2F_EDGES;
      fstream f_res(result_file.c_str(), ios::out | ios::app);

      f_res << fixed <<
      setprecision(6) <<
      edges_result_[idx].name_a << "," <<
      edges_result_[idx].name_b << "," <<
      edges_result_[idx].valid_sim3 << "," <<
      edges_result_[idx].valid_icp << "," <<
      edges_result_[idx].sim3_inliers << "," <<
      edges_result_[idx].icp_score <<  endl;

      f_res.close();
    }
  }


} // namespace