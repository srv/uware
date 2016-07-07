#ifndef UTILS
#define UTILS

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>

#include <opencv2/opencv.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

using namespace std;
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

  /** \brief Ratio matching between descriptors
   * \param Descriptors of image 1
   * \param Descriptors of image 2
   * \param ratio value (0.6/0.9)
   * \param output matching
   */
  static void ratioMatching(cv::Mat desc_1, cv::Mat desc_2, double ratio, vector<cv::DMatch> &matches)
  {
    matches.clear();
    if (desc_1.rows < 10 || desc_2.rows < 10) return;

    cv::Mat match_mask;
    const int knn = 2;
    cv::Ptr<cv::DescriptorMatcher> descriptor_matcher;
    descriptor_matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    vector<vector<cv::DMatch> > knn_matches;
    descriptor_matcher->knnMatch(desc_1, desc_2, knn_matches, knn, match_mask);
    for (uint m=0; m<knn_matches.size(); m++)
    {
      if (knn_matches[m].size() < 2) continue;
      if (knn_matches[m][0].distance <= knn_matches[m][1].distance * ratio)
        matches.push_back(knn_matches[m][0]);
    }
  }

  static tf::Transform solvePnP2tf(cv::Mat rvec, cv::Mat tvec)
  {
    if (rvec.empty() || tvec.empty())
      return tf::Transform();

    tf::Vector3 axis(rvec.at<double>(0, 0),
                     rvec.at<double>(1, 0),
                     rvec.at<double>(2, 0));
    double angle = norm(rvec);
    tf::Quaternion quaternion(axis, angle);

    tf::Vector3 translation(tvec.at<double>(0, 0), tvec.at<double>(1, 0),
        tvec.at<double>(2, 0));

    return tf::Transform(quaternion, translation);
  }

  static void tf2SolvePnP(tf::Transform in, cv::Mat& rvec, cv::Mat& tvec)
  {
    // Convert rotation
    tf::Matrix3x3 rot = in.getBasis();
    cv::Mat tmp_r(3,3, CV_64FC1);
    rvec = cv::Mat(3,1, CV_64FC1);
    tmp_r.at<double>(0,0) = rot[0][0];
    tmp_r.at<double>(0,1) = rot[0][1];
    tmp_r.at<double>(0,2) = rot[0][2];
    tmp_r.at<double>(1,0) = rot[1][0];
    tmp_r.at<double>(1,1) = rot[1][1];
    tmp_r.at<double>(1,2) = rot[1][2];
    tmp_r.at<double>(2,0) = rot[2][0];
    tmp_r.at<double>(2,1) = rot[2][1];
    tmp_r.at<double>(2,2) = rot[2][2];
    cv::Rodrigues(tmp_r, rvec);

    // Convert translation vector
    tvec = cv::Mat(3,1, CV_64FC1);
    tvec.at<double>(0, 0) = in.getOrigin().x();
    tvec.at<double>(1, 0) = in.getOrigin().y();
    tvec.at<double>(2, 0) = in.getOrigin().z();
  }

  static double tfDist(tf::Transform a, tf::Transform b, bool xy=false)
  {
    if (xy)
    {
      return sqrt((a.getOrigin().x() - b.getOrigin().x()) * (a.getOrigin().x() - b.getOrigin().x()) +
                  (a.getOrigin().y() - b.getOrigin().y()) * (a.getOrigin().y() - b.getOrigin().y()));
    }
    else
    {
      return sqrt((a.getOrigin().x() - b.getOrigin().x()) * (a.getOrigin().x() - b.getOrigin().x()) +
                  (a.getOrigin().y() - b.getOrigin().y()) * (a.getOrigin().y() - b.getOrigin().y()) +
                  (a.getOrigin().z() - b.getOrigin().z()) * (a.getOrigin().z() - b.getOrigin().z()));
    }
  }

  /** \brief Sort 2 pairs by size
   * @return true if pair 1 is smaller than pair 2
   * \param pair 1
   * \param pair 2
   */
  static bool sortByDistance(const pair<uint, double> a, const pair<uint, double> b)
  {
    return (a.second < b.second);
  }

  /** \brief Convert id to string
   * @return 6 digits id string
   * \param input id
   */
  static string id2str(int id)
  {
    stringstream id_ss;
    id_ss << setfill('0') << setw(6) << id;
    return id_ss.str();
  }


};

} // namespace

#endif // UTILS