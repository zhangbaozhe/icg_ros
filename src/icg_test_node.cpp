//
// Created by baozhe on 22-9-20.
//

#include "icg_ros/ros_camera.h"
#include "icg_ros/icg_ros_interface.h"

#include <iostream>
#include <string>

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>


#include <gflags/gflags.h>

DEFINE_string(config_dir, "", "The directory of the configuration files");
DEFINE_string(camera_frame, "", "The camera frame in which object is tracked");

int main(int argc, char **argv)
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  const std::string config_dir = FLAGS_config_dir;
  const std::string camera_frame = FLAGS_camera_frame;

  ros::init(argc, argv, "icg_test_node");

  ros::NodeHandle interface_nh;
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

  icg_ros::ICG_ROS interface(interface_nh, config);
  auto tracker_ptr = interface.GetTrackerPtr();
  auto tracking_sub = nh.subscribe<std_msgs::Bool>("start_tracking", 10,
      [&](const std_msgs::BoolConstPtr &msg)
      {
        tracker_ptr->ExecuteDetection(msg->data);
      });
  auto pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);

  ros::Rate rate(60);

  int iteration = 0;
  while (ros::ok()) {
    interface.RunTrackerProcessOneFrame(iteration);
    icg::Transform3fA temp_transform = tracker_ptr->body_ptrs()[0]->body2world_pose();
    Eigen::Quaternionf rotation(temp_transform.matrix().block<3, 3>(0, 0));
    Eigen::Vector3f trans(temp_transform.matrix().block<3, 1>(0, 3));
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "camera";
    pose.header.stamp = ros::Time::now();  
    pose.pose.orientation.w = rotation.w();
    pose.pose.orientation.x = rotation.x();
    pose.pose.orientation.y = rotation.y();
    pose.pose.orientation.z = rotation.z();
    pose.pose.position.x = trans.x();
    pose.pose.position.y = trans.y();
    pose.pose.position.z = trans.z();
    pose_publisher.publish(pose);
    iteration++;
    ros::spinOnce();
    rate.sleep();
  }
  ros::shutdown();

  return 0;

}