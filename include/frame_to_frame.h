#ifndef FRAMETOFRAME_H
#define FRAMETOFRAME_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <opencv2/opencv.hpp>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>


#include "constants.h"
#include "edge_info.h"
#include "pose_info.h"
#include "utils.h"
#include "registration.h"

using namespace std;
using namespace boost;
namespace fs  = filesystem;

typedef pcl::PointXYZRGB          Point;
typedef pcl::PointCloud<Point>    PointCloud;

namespace uware
{

class FrameToFrame
{

public:

  struct Params
  {
    string outdir;                  //!> Output directory
    string indir;                   //!> Input directory (the output of pre-process)

    // Debug
    bool show_generic_logs;         //!> Show generic logs

    // Default settings
    Params () {
      outdir                  = "";
      indir                   = "";

      show_generic_logs       = false;
    }
  };

  /** \brief Set class params
   * \param the parameters struct
   */
  inline void setParams(const Params& params){params_ = params;}

  /** \brief Class constructor
   * \param Pointer to a registration object
   */
  FrameToFrame(Registration* reg);

  /** \brief Compute the frame to frame tf
   * \param Output vector with the corrected cloud poses
   */
  void compute(vector<PoseInfo>& poses, vector<EdgeInfo>& edges);

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

  /** \brief Save result to file
   */
  void saveResult();

private:

  Params params_; //!> Stores parameters.

  bool first_; //!> First iteration

  vector< pair<string, tf::Transform> > odom_cloud_poses_; //!> Array of odom cloud of poses

  vector< pair<string, tf::Transform> > map_cloud_poses_; //!> Array of map cloud of poses

  tf::Transform prev_pose_; //!> Previous estimated robot pose

  vector<PoseInfo> poses_result_; //!> Output vector of corrected cloud poses

  vector<EdgeInfo> edges_result_; //!> Edges information

  Registration* reg_; //!> Registration class

};

} // namespace

#endif // FRAMETOFRAME_H