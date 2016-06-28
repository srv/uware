#ifndef UTILS
#define UTILS

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>

#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

using namespace boost;
namespace fs  = filesystem;

namespace uware
{

class Utils
{

public:

  // Definitions
  typedef pcl::PointXYZRGB        Point;
  typedef pcl::PointCloud<Point>  PointCloud;

  /** \brief Create a directory
   * @return true if directory was created successfully
   * \param directory path
   */
  static bool createDir(string dir_name)
  {
    if (fs::is_directory(dir_name))
      fs::remove_all(dir_name);
    fs::path dir1(dir_name);
    if (!fs::create_directory(dir1))
    {
      ROS_ERROR_STREAM("[PreProcess]: Impossible to create the directory: " << dir_name);
      return false;
    }
    return true;
  }


  /** \brief Get cloud salient indices
   * @return
   * \param input cloud
   * \param output vector with salient indices
   * \param zeta difference, respect to the z mean, to consider a point to be salient (in percentage)
   */
  static void getCloudSalientIndices(PointCloud::Ptr cloud, vector<uint>& salient_indices, double z_diff_th)
  {
    vector<double> zs;
    salient_indices.clear();
    for (uint i=0; i<cloud->size(); i++)
      zs.push_back(cloud->points[i].z);

    double mean_z = accumulate(zs.begin(), zs.end(), 0.0) / zs.size();
    for (uint i=0; i< zs.size(); i++)
    {
      // Compute zeta difference in percentage
      double diff_percentage = 100.0 - std::min(mean_z, zs[i]) * 100.0 / std::max(mean_z, zs[i]);
      if (diff_percentage > z_diff_th && fabs(mean_z - zs[i]) > 0.1)
        salient_indices.push_back(i);
    }
  }

  /** \brief convert a Eigen::Matrix4f to tf::transform
   * @return tf::transform matrix
   * \param in of type Eigen::Matrix4f
   */
  static tf::Transform matrix4fToTf(Eigen::Matrix4f in)
  {
    tf::Vector3 t_out;
    t_out.setValue(static_cast<double>(in(0,3)),static_cast<double>(in(1,3)),static_cast<double>(in(2,3)));

    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(in(0,0)), static_cast<double>(in(0,1)), static_cast<double>(in(0,2)),
                  static_cast<double>(in(1,0)), static_cast<double>(in(1,1)), static_cast<double>(in(1,2)),
                  static_cast<double>(in(2,0)), static_cast<double>(in(2,1)), static_cast<double>(in(2,2)));

    tf::Quaternion q_out;
    tf3d.getRotation(q_out);
    tf::Transform out(q_out, t_out);
    return out;
  }

  /** \brief Read the poses from a csv file
   * @return
   * \param filename
   * \param output vector with poses
   */
  static void readPoses(string poses_file_str, vector< pair<string, tf::Transform> > &poses)
  {
    // Init
    poses.clear();

    // Get the pointcloud poses file
    int line_counter = 0;
    tf::Transform zero_pose;
    ifstream poses_file(poses_file_str.c_str());
    string line;
    while (getline(poses_file, line))
    {
      int i = 0;
      string cloud_name, value;
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      double qx = 0.0;
      double qy = 0.0;
      double qz = 0.0;
      double qw = 0.0;
      istringstream ss(line);
      while(getline(ss, value, ','))
      {
        if (i == 0)
          cloud_name = value;
        else if (i == 2)
          x = boost::lexical_cast<double>(value);
        else if (i == 3)
          y = boost::lexical_cast<double>(value);
        else if (i == 4)
          z = boost::lexical_cast<double>(value);
        else if (i == 5)
          qx = boost::lexical_cast<double>(value);
        else if (i == 6)
          qy = boost::lexical_cast<double>(value);
        else if (i == 7)
          qz = boost::lexical_cast<double>(value);
        else if (i == 8)
          qw = boost::lexical_cast<double>(value);
        i++;
      }
      // Build the tf
      tf::Vector3 t(x, y, z);
      tf::Quaternion q(qx, qy, qz, qw);
      tf::Transform transf(q, t);

      // Save
      poses.push_back(make_pair(cloud_name, transf));
      line_counter++;
    }
  }


};

} // namespace

#endif // UTILS