#ifndef GRAPH_H
#define GRAPH_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include <boost/lexical_cast.hpp>

#include "edge_info.h"
#include "pose_info.h"
#include "utils.h"

using namespace std;
using namespace boost;

namespace uware
{

class Graph
{

public:

  /** \brief Class constructor
   */
  Graph();

  /** \brief Optimize graph
   * \param the vector of poses
   * \param the vector of edges between the poses
   */
  void optimize(vector<PoseInfo>& poses, vector<EdgeInfo>& edges);

protected:

  int searchIdx(string name);

private:

  g2o::SparseOptimizer graph_optimizer_; //!> G2O graph optimizer

  vector< pair<int,string> > vertex_names_; //!> Table of vertex names

};

} // namespace

#endif // GRAPH_H