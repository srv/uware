#ifndef BUILD3D_H
#define BUILD3D_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include "constants.h"
#include "pose_info.h"
#include "utils.h"

using namespace std;

typedef pcl::PointXYZRGB          Point;
typedef pcl::PointCloud<Point>    PointCloud;

namespace uware
{

class Build3D
{

public:

  struct Params
  {
    string indir;                   //!> Input directory (the output of pre-process)
    string outdir;                  //!> Input directory (the output of pre-process)
    double voxel_resolution;        //!> Voxel filter cloud resolution (m)

    // Default settings
    Params () {
      indir                   = "";
      outdir                  = "";
      voxel_resolution        = 0.01;
    }
  };

  /** \brief Set class params
   * \param the parameters struct
   */
  inline void setParams(const Params& params){params_ = params;}

  /** \brief Class constructor
   */
  Build3D();

  /** \brief Build the 3D
   * \param the vector of poses
   */
  void build(const vector<PoseInfo>& poses);

protected:

  /** \brief Filters a pointcloud
   * @return filtered cloud
   * \param input cloud
   */
  PointCloud::Ptr filterCloud(PointCloud::Ptr in_cloud);


private:

  Params params_; //!> Stores parameters.

  bool first_; //!> First iteration

};

} // namespace

#endif // BUILD3D_H