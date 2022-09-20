//
// Created by baozhe on 22-9-20.
//

#ifndef ICG_ROS_ICG_ROS_INCLUDE_ICG_ROS_ROS_CAMERA_H_
#define ICG_ROS_ICG_ROS_INCLUDE_ICG_ROS_ROS_CAMERA_H_

#include <boost/shared_ptr.hpp>

#include <icg/common.h>
#include <icg/camera.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

namespace icg_ros
{

class RosColorCamera : public icg::ColorCamera
{
 public:
  RosColorCamera(const std::string &name,
                 const std::string &color_image_topic,
                 const std::string &camera_info_topic,
                 ros::NodeHandle &nh);

  bool SetUp() override;

  bool UpdateImage(bool synchronized) override;

 private:
  void ImageCallback(const sensor_msgs::ImageConstPtr &color);
//  void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info);
  void RosCamera2Intrinsics(const sensor_msgs::CameraInfo &camera_info,
                            icg::Intrinsics &intrinsics);
  ros::NodeHandle nh_;
  ros::CallbackQueue callback_queue_;
  std::string color_image_topic_;
  std::string camera_info_topic_;
  ros::Subscriber color_image_sub_;
};


class RosDepthCamera : public icg::DepthCamera
{
 public:
  RosDepthCamera(const std::string &name,
                 const std::string &depth_image_topic,
                 const std::string &camera_info_topic,
                 ros::NodeHandle &nh);

  bool SetUp() override;

  bool UpdateImage(bool synchronized) override;

 private:
  void ImageCallback(const sensor_msgs::ImageConstPtr &depth);
//  void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info);
  void RosCamera2Intrinsics(const sensor_msgs::CameraInfo &camera_info,
                            icg::Intrinsics &intrinsics);
  ros::NodeHandle nh_;
  ros::CallbackQueue callback_queue_;
  std::string depth_image_topic_;
  std::string camera_info_topic_;
  ros::Subscriber depth_image_sub_;
};


}

#endif //ICG_ROS_ICG_ROS_INCLUDE_ICG_ROS_ROS_CAMERA_H_
