//
// Created by baozhe on 22-9-20.
//

#include <iostream>
#include <string>

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <icg_ros/ros_camera.h>
#include <icg_ros/icg_ros_interface.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

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
  auto pose_publisher = nh.advertise<std_msgs::Float64MultiArray>("pose", 10);
  static tf2_ros::TransformBroadcaster br;

  ros::Rate rate(60);

  int iteration = 0;
  while (ros::ok()) {
    interface.RunTrackerProcessOneFrame(iteration);
    icg::Transform3fA temp_transform = tracker_ptr->body_ptrs()[0]->body2world_pose();
    Eigen::Affine3d eigen_affine_transform;
    eigen_affine_transform.matrix() = temp_transform.matrix().cast<double>();
    std::cerr << "" << eigen_affine_transform.matrix() << std::endl;

    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped = tf2::eigenToTransform(eigen_affine_transform);

    tf_stamped.header.frame_id = camera_frame;
    tf_stamped.child_frame_id = "tracked_obj";
    tf_stamped.header.stamp = ros::Time::now();
    br.sendTransform(tf_stamped);

    iteration++;
    ros::spinOnce();
    rate.sleep();
  }
  ros::shutdown();

  return 0;

}