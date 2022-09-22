//
// Created by baozhe on 22-9-20.
//

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <icg_ros/ros_camera.h>
#include <icg_ros/icg_ros_interface.h>

#include <gflags/gflags.h>

DEFINE_string(config_dir, "", "The directory of the configuration files");



int main(int argc, char **argv)
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  const std::string config_dir = FLAGS_config_dir;

  ros::init(argc, argv, "icg_test_node");

  ros::NodeHandle nh;

  icg_ros::ICG_ROS_Config config;
  config.config_dir = config_dir;
  config.body_names.push_back("triangle");
  config.tracker_name = "tracker";
  config.renderer_geometry_name = "renderer_geometry";
  config.color_camera_name = "color_interface";
  config.color_camera_topic = "/camera/color/image_raw";
  config.color_camera_info_topic = "/camera/color/camera_info";
  config.depth_camera_name = "depth_interface";
  config.depth_camera_topic = "/camera/aligned_depth_to_color/image_raw";
  config.depth_camera_info_topic = "/camera/aligned_depth_to_color/camera_info";
  config.color_depth_renderer_name = "color_depth_renderer";
  config.depth_depth_renderer_name = "depth_depth_renderer";

  icg_ros::ICG_ROS interface(nh, config);



//  icg_ros::RosColorCamera color_interface(
//      "color_interface",
//      "/camera/color/image_raw",
//      "/camera/color/camera_info",
//      color_camera_nh);
//  icg_ros::RosDepthCamera depth_interface(
//      "depth_interface",
//      "/camera/aligned_depth_to_color/image_raw",
//      "/camera/aligned_depth_to_color/camera_info",
//      depth_camera_nh);
//
//  color_interface.SetUp();
//  depth_interface.SetUp();

  ros::Rate rate(30);

  int iteration = 0;
  while (ros::ok()) {
    interface.RunTrackerProcessOneFrame(true, true, iteration);
    iteration++;
    rate.sleep();
  }

  return 0;

}