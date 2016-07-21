#include <ros/ros.h>

#include "build3d.h"

namespace uware
{
  Build3D::Build3D() : first_(true) { }

  void Build3D::build(const vector<PoseInfo>& poses)
  {
    PointCloud::Ptr acc(new PointCloud);

    for (uint i=0; i<poses.size(); i++)
    {
      string rawname = poses[i].name;
      tf::Transform pose = poses[i].pose;

      ROS_INFO_STREAM("[Reconstruction]: Appending cloud " << rawname);

      // Load the pointcloud
      string filepath_src = params_.indir + "/" + PC_DIR + "/" + rawname + ".pcd";
      PointCloud::Ptr in_cloud(new PointCloud);
      if (pcl::io::loadPCDFile<Point> (filepath_src, *in_cloud) == -1)
      {
        ROS_ERROR_STREAM("[Reconstruction]: Pointcloud " << filepath_src << " does not exists!");
        return;
      }

      // Transform pointcloud
      Eigen::Affine3d pose_eigen;
      transformTFToEigen(pose, pose_eigen);
      pcl::transformPointCloud(*in_cloud, *in_cloud, pose_eigen);

      if (first_)
      {
        pcl::copyPointCloud(*in_cloud, *acc);
        first_ = false;
      }
      else
      {
        // Append the clouds
        *acc += *in_cloud;
      }
    }

    ROS_INFO("[Reconstruction]: Filtering output cloud");

    // Filter the cloud
    acc = filterCloud(acc);

    ROS_INFO("[Reconstruction]: Saving output cloud");

    // Save the cloud
    string pc_filename = params_.outdir + "/reconstruction.pcd";
    pcl::io::savePCDFileBinary(pc_filename, *acc);

  }

  PointCloud::Ptr Build3D::filterCloud(PointCloud::Ptr in_cloud)
  {
    // Remove nans
    vector<int> indicies;
    PointCloud::Ptr cloud(new PointCloud);
    pcl::removeNaNFromPointCloud(*in_cloud, *cloud, indicies);

    // Voxel grid filter (used as x-y surface extraction. Note that leaf in z is very big)
    pcl::ApproximateVoxelGrid<Point> grid;
    grid.setLeafSize(params_.voxel_resolution, params_.voxel_resolution, params_.voxel_resolution);
    grid.setDownsampleAllData(true);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);

    return cloud;
  }

} // namespace