#ifndef POSEINFO_H
#define POSEINFO_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

using namespace std;

namespace uware
{
  struct PoseInfo
  {
    string name;            //!> Name of node
    tf::Transform pose;     //!> Corrected pose

    PoseInfo(string name_in, tf::Transform pose_in) {
      name = name_in;
      pose = pose_in;
    }
  };

} // namespace

#endif // POSEINFO_H