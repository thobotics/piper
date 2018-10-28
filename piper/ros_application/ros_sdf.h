#ifndef ROS_SDF_H_
#define ROS_SDF_H_

#include <map>
#include <string>

#include <ros/ros.h>

namespace piper {
class RosSDF
{
public:
  /// Default constructor
  RosSDF(){}
  /**
   *  STEAP: simultaneous trajectory estimation and planning
   *
   *  @param nh node handle for namespace
   **/
  RosSDF(ros::NodeHandle nh);
};

}

#endif
