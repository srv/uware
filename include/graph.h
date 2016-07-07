#ifndef GRAPH_H
#define GRAPH_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "constants.h"
#include "edge_info.h"
#include "pose_info.h"
#include "utils.h"

using namespace std;

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
  void optimize(vector<PoseInfo> poses, vector<EdgeInfo> edges);


protected:


private:

};

} // namespace

#endif // GRAPH_H