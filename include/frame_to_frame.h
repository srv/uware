#ifndef FRAMETOFRAME_H
#define FRAMETOFRAME_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "constants.h"
#include "utils.h"

using namespace std;
using namespace boost;
namespace fs  = filesystem;

typedef pcl::PointXYZRGB          Point;
typedef pcl::PointCloud<Point>    PointCloud;
typedef pcl::IterativeClosestPoint<Point, Point> IterativeClosestPoint;

namespace uware
{

class FrameToFrame
{

public:

  struct Params
  {
    string outdir;                  //!> Output directory
    string indir;                   //!> Input directory (the output of pre-process)
    double z_diff_th;               //!> The z difference threshold to consider the existence of 3D structure (in percentage respect to the mean z)
    double min_salient_ids;         //!> Minimum percentage of salient indices to consider a pointcloud valid for registration
    double max_icp_fitness_score;   //!> Maximum icp fitness score to consider a valid icp registration
    double max_reg_err;             //!> Maximum registration error between odometry and ICP or Sim3 (euclidean distance in meters)

    // Debug
    bool show_salient_ids;          //!> Show the salient indices percentage
    bool show_icp_score;            //!> Show the icp fitness score
    bool show_icp_tf;               //!> Show the icp output transformation
    bool save_icp_clouds;           //!> Save paired clouds

    // Default settings
    Params () {
      outdir                  = "";
      indir                   = "";
      z_diff_th               = 15.0;
      min_salient_ids         = 15.0;
      max_icp_fitness_score   = 0.1;
      max_reg_err             = 0.3;

      show_salient_ids        = false;
      show_icp_score          = false;
      show_icp_tf             = false;
      save_icp_clouds         = false;
    }
  };

  /** \brief Set class params
   * \param the parameters struct
   */
  inline void setParams(const Params& params){params_ = params;}

  /** \brief Class constructor
   */
  FrameToFrame();

  /** \brief Compute the frame to frame tf
   */
  void compute();

protected:

  /** \brief Create operational directories
   * @return true if all directories have been created
   */
  bool createDirs();

  /** \brief Get a cloud pose id
   * @return index of this cloud into the cloud poses vector
   * \param input cloud filename (without extension)
   * \param input vector of cloud poses
   */
  int getCloudPoseId(string rawname, vector< pair<string, tf::Transform> > cloud_poses);

  /** \brief Pointcloud to pointcloud ICP registration
   * @return true if icp is valid
   * \param input source cloud
   * \param input target cloud
   * \param output transformation to align the pointclouds
   */
  bool pairAlign(PointCloud::Ptr src, PointCloud::Ptr tgt, tf::Transform &output);

  /** \brief Performs all the registration
   * @return true if registration is valid
   * \param id
   * \param previous pointcloud
   * \param current pointcloud
   * \param output vector of salient indices
   */
  bool registration(int id, PointCloud::Ptr prev_cloud, PointCloud::Ptr curr_cloud, vector<uint>& salient_indices);


private:

  Params params_; //!> Stores parameters.

  bool first_; //!> First iteration

  vector< pair<string, tf::Transform> > cloud_poses_; //!> Array of cloud of poses

  PointCloud::Ptr prev_cloud_; //!> Previous pointcloud
  double prev_cloud_salient_ids_; //!> Percentage of salient indices on the previous cloud
  string prev_rawname_; //!> Previous rawname

};

} // namespace

#endif // FRAMETOFRAME_H