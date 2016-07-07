#ifndef EDGEINFO_H
#define EDGEINFO_H

#include <ros/ros.h>

using namespace std;

namespace uware
{
  struct EdgeInfo
  {
    string name_a;          //!> Name of node a
    string name_b;          //!> Name of node b
    bool valid_sim3;        //!> True if valid Sim3
    bool valid_icp;         //!> True if valid icp
    int sim3_inliers;       //!> Number of Sim3 inliers between nodes (0 if no Sim3)
    double icp_score;       //!> ICP fitness score (-1 if no icp)

    EdgeInfo() {
      name_a = "";
      name_b = "";
      valid_sim3 = false;
      valid_icp = false;
      sim3_inliers = 0;
      icp_score = -1.0;
    }
    EdgeInfo(string a, string b, bool ok_sim3, bool ok_icp, int inliers, double score) {
      name_a = a;
      name_b = b;
      valid_sim3 = ok_sim3;
      valid_icp = ok_icp;
      sim3_inliers = inliers;
      icp_score = score;
    }
  };

} // namespace

#endif // EDGEINFO_H