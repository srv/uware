#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include "constants.h"
#include "edge_info.h"
#include "pose_info.h"
#include "utils.h"
#include "registration.h"

using namespace std;

typedef pcl::PointXYZRGB          Point;
typedef pcl::PointCloud<Point>    PointCloud;

namespace uware
{

class LoopClosing
{

public:

  struct Params
  {
    string indir;                   //!> Input directory (the output of pre-process)
    int discard_window;             //!> Number of previous and consecutive nodes to discard in the loop closing search
    int n_best;                     //!> Number of candidates to take to try the loop closing

    // Default settings
    Params () {
      indir                   = "";
      discard_window          = 5;
      n_best                  = 5;
    }
  };

  /** \brief Set class params
   * \param the parameters struct
   */
  inline void setParams(const Params& params){params_ = params;}

  /** \brief Class constructor
   */
  LoopClosing(Registration* reg);

  /** \brief Compute the loop closings
   * \param the vector of poses
   * \param the vector of edges between the poses
   */
  void compute(vector<PoseInfo> poses, vector<EdgeInfo> edges);


protected:

  vector<uint> getClosestPoses(vector<PoseInfo> poses, uint id, int n_best, int discard_window, bool xy);

private:

  Params params_; //!> Stores parameters.

  Registration* reg_; //!> Registration class

};

} // namespace

#endif // LOOPCLOSING_H