#include <ros/ros.h>

#include "frame_to_frame.h"

namespace uware
{
  FrameToFrame::FrameToFrame() : first_(true), prev_cloud_(new PointCloud) { }

  void FrameToFrame::compute()
  {

    // Create the output directory
    string output_dir = params_.outdir;
    if (fs::is_directory(output_dir))
      fs::remove_all(output_dir);
    fs::path dir1(output_dir);
    if (!fs::create_directory(dir1))
    {
      ROS_ERROR("[Reconstruction]: Impossible to create the output directory.");
    }

    // Read poses
    vector< pair<string, tf::Transform> > cloud_poses;
    Utils::readPoses(params_.indir + "/" + OMAP_FILE, cloud_poses);

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
        ROS_INFO_STREAM("[Reconstruction]: Processing cloud: " << filename);

        // Search current pointcloud into the cloud poses
        uint id = getCloudPoseId(rawname, cloud_poses);
        if (id < 0)
        {
          ROS_ERROR_STREAM("[Reconstruction]: Impossible to find the id into the cloud_poses array for file: " << filename);
          break;
        }

        // Load the pointcloud
        PointCloud::Ptr in_cloud(new PointCloud);
        if (pcl::io::loadPCDFile<Point> (filepath, *in_cloud) == -1)
        {
          ROS_WARN_STREAM("[Reconstruction]: Couldn't read the file: " << filename);
          it++;
          continue;
        }

        // First iteration
        if (first_)
        {
          pcl::copyPointCloud(*in_cloud, *prev_cloud_);

          vector<uint> salient_indices;
          Utils::getCloudSalientIndices(prev_cloud_, salient_indices, params_.z_diff_th);
          prev_cloud_salient_ids_ = salient_indices.size() * 100.0 / prev_cloud_->size();

          first_ = false;
          it++;
          continue;
        }

        // Determine if current and previous pointcloud are useful for registration
        vector<uint> salient_indices;
        Utils::getCloudSalientIndices(in_cloud, salient_indices, params_.z_diff_th);
        double curr_salient_ids = salient_indices.size() * 100.0 / in_cloud->size();

        if (params_.show_salient_ids)
          ROS_INFO_STREAM("[Reconstruction]: Salient indices for " << filename << " is: " << curr_salient_ids << "\%");

        if (prev_cloud_salient_ids_ > params_.min_salient_ids &&
            curr_salient_ids > params_.min_salient_ids)
        {
          // Frame to frame registration
          ROS_INFO_STREAM("[Reconstruction]: " << filename << " is useful for registration.");
          ROS_INFO("[Reconstruction]: Aligning clouds...");

          // Transform current cloud according to slam pose
          tf::Transform tf_prev = cloud_poses[id-1].second;
          tf::Transform tf_curr = cloud_poses[id].second;
          tf::Transform tf_01 = tf_prev.inverse() * tf_curr;
          Eigen::Affine3d tf_01_eigen;
          transformTFToEigen(tf_01, tf_01_eigen);
          PointCloud::Ptr prev_cloud_moved(new PointCloud);
          pcl::transformPointCloud(*prev_cloud_, *prev_cloud_moved, tf_01_eigen);

          // Pair align
          tf::Transform correction;
          pairAlign(prev_cloud_moved, in_cloud, correction);

        }
        else
        {
          // No 3D information for registration, try visual registration
        }

        // Copy
        pcl::copyPointCloud(*in_cloud, *prev_cloud_);
        prev_cloud_salient_ids_ = curr_salient_ids;

      }
      // Next directory entry
      it++;

    }

  }

  int FrameToFrame::getCloudPoseId(string rawname, vector< pair<string, tf::Transform> > cloud_poses)
  {
    for (uint i=0; i<cloud_poses.size(); i++)
    {
      if (cloud_poses[i].first == rawname)
        return i;
    }
    return -1;
  }

  bool FrameToFrame::pairAlign(PointCloud::Ptr src,
                               PointCloud::Ptr tgt,
                               tf::Transform &output)
  {
    PointCloud::Ptr aligned (new PointCloud);
    IterativeClosestPoint icp;
    icp.setMaxCorrespondenceDistance(0.07);
    icp.setRANSACOutlierRejectionThreshold(0.005);
    icp.setTransformationEpsilon(0.00001);
    icp.setEuclideanFitnessEpsilon(0.0001);
    icp.setMaximumIterations(200);
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.align(*aligned);
    double score = icp.getFitnessScore();

    if (params_.show_icp_score)
      ROS_INFO_STREAM("[Reconstruction]: ICP fitness score: " << score);

    // The transform
    output = Utils::matrix4fToTf(icp.getFinalTransformation());
    if (params_.show_icp_tf)
    {
      ROS_INFO_STREAM("[Reconstruction]: ICP correction: " << output.getOrigin().x() <<
                                                      ", " << output.getOrigin().y() <<
                                                      ", " << output.getOrigin().z());
    }

    // Return valid or not
    return ( icp.hasConverged() && (score < params_.max_icp_fitness_score) );
  }


} // namespace