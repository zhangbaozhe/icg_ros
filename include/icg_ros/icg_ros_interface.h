//
// Created by baozhe on 22-9-20.
//

#ifndef ICG_ROS_ICG_ROS_INCLUDE_ICG_ROS_ICG_ROS_INTERFACE_H_
#define ICG_ROS_ICG_ROS_INCLUDE_ICG_ROS_ICG_ROS_INTERFACE_H_

#include <iostream>
#include <icg/common.h>
#include <ros/ros.h>

class ICG_ROS
{
 public:

  ICG_ROS(ros::NodeHandle &nh)
    : nh_{nh}
  {

  }

 private:
  ros::NodeHandle nh_;
};

#endif //ICG_ROS_ICG_ROS_INCLUDE_ICG_ROS_ICG_ROS_INTERFACE_H_
