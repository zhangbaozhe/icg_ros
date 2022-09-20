//
// Created by baozhe on 22-9-20.
//

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <icg_ros/ros_camera.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "icg_test_node");

  ros::NodeHandle color_camera_nh;
  ros::NodeHandle depth_camera_nh;

  icg_ros::RosColorCamera color_interface(
      "color_interface",
      "/camera/color/image_raw",
      "/camera/color/camera_info",
      color_camera_nh);
  icg_ros::RosDepthCamera depth_interface(
      "depth_interface",
      "/camera/aligned_depth_to_color/image_raw",
      "/camera/aligned_depth_to_color/camera_info",
      depth_camera_nh);

  color_interface.SetUp();
  depth_interface.SetUp();

  ros::Rate rate(60);

  while (ros::ok()) {
    rate.sleep();
    ROS_INFO("HELLO");
    color_interface.UpdateImage(true);
    depth_interface.UpdateImage(true);
//    cv::imshow("color", color_interface.image());
    cv::imshow("depth", depth_interface.image());
    cv::waitKey(1);
  }

  return 0;

}