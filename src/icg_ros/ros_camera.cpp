//
// Created by baozhe on 22-9-20.
//

#include <icg_ros/ros_camera.h>
#include <cv_bridge/cv_bridge.h>

namespace icg_ros
{

RosColorCamera::RosColorCamera(const std::string &name,
                               const std::string &color_image_topic,
                               const std::string &camera_info_topic,
                               ros::NodeHandle &nh)
                               : ColorCamera{name},
                                 color_image_topic_{color_image_topic},
                                 camera_info_topic_{camera_info_topic},
                                 nh_{nh}
{
  nh_.setCallbackQueue(&callback_queue_);

  // if we do not have this subscription in advance connected,
  // the following ros::topic::waitForMessage will register one
  // each time when it is called, which is slow
  color_image_sub_ = nh_.subscribe<sensor_msgs::Image>(color_image_topic_, 1,
                                                       &RosColorCamera::ImageCallback, this);
}

void RosColorCamera::RosCamera2Intrinsics(const sensor_msgs::CameraInfo &camera_info, icg::Intrinsics &intrinsics)
{
  intrinsics.fu = camera_info.K[0];
  intrinsics.fv = camera_info.K[4];
  intrinsics.ppu = camera_info.K[2];
  intrinsics.ppv = camera_info.K[5];
  intrinsics.width = camera_info.width;
  intrinsics.height = camera_info.height;
}

void RosColorCamera::ImageCallback(const sensor_msgs::ImageConstPtr &color)
{
//  image_ = cv_bridge::toCvShare(*color, nullptr, color->encoding)->image.clone();
}


bool RosColorCamera::SetUp()
{
  // if the camera info arrives, set_up_ will become true,
  // if it does not arrive, we wait
  set_up_ = false;
  while (!set_up_) {
    auto temp_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
        camera_info_topic_, ros::Duration(5.0));
    if (temp_ptr != nullptr) {
      set_up_ = true;
      RosCamera2Intrinsics(*temp_ptr, intrinsics_);
    }
  }
  return UpdateImage(true);
}

bool RosColorCamera::UpdateImage(bool synchronized)
{
  if (!set_up_ || !ros::ok()) {
    std::cerr << "Set up ros color camera " << name_ << " first"
              << std::endl;
    return false;
  }
  auto image_msg = ros::topic::waitForMessage<sensor_msgs::Image>(color_image_topic_, nh_);
  if (image_msg == nullptr)
    return false;
  image_ = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
  if (image_.empty()) {
    std::cerr << "Could not update image from the topic " << color_image_topic_ << std::endl;
    return false;
  }
  return true;
}

RosDepthCamera::RosDepthCamera(const std::string &name,
                               const std::string &depth_image_topic,
                               const std::string &camera_info_topic,
                               ros::NodeHandle &nh)
    : DepthCamera{name},
      depth_image_topic_{depth_image_topic},
      camera_info_topic_{camera_info_topic},
      nh_{nh}
{
  nh_.setCallbackQueue(&callback_queue_);

  // if we do not have this subscription in advance connected,
  // the following ros::topic::waitForMessage will register one
  // each time when it is called, which is slow
  depth_image_sub_ = nh_.subscribe<sensor_msgs::Image>(depth_image_topic_, 1,
                                                       &RosDepthCamera::ImageCallback, this);
}

void RosDepthCamera::RosCamera2Intrinsics(const sensor_msgs::CameraInfo &camera_info, icg::Intrinsics &intrinsics)
{
  intrinsics.fu = camera_info.K[0];
  intrinsics.fv = camera_info.K[4];
  intrinsics.ppu = camera_info.K[2];
  intrinsics.ppv = camera_info.K[5];
  intrinsics.width = camera_info.width;
  intrinsics.height = camera_info.height;
}

void RosDepthCamera::ImageCallback(const sensor_msgs::ImageConstPtr &depth)
{
//  image_ = cv_bridge::toCvShare(*depth, nullptr, depth->encoding)->image.clone();
}


bool RosDepthCamera::SetUp()
{
  // if the camera info arrives, set_up_ will become true,
  // if it does not arrive, we wait
  set_up_ = false;
  while (!set_up_) {
    auto temp_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
        camera_info_topic_, ros::Duration(5.0));
    if (temp_ptr != nullptr) {
      set_up_ = true;
      RosCamera2Intrinsics(*temp_ptr, intrinsics_);
    }
  }
  return UpdateImage(true);
}

bool RosDepthCamera::UpdateImage(bool synchronized)
{
  if (!set_up_ || !ros::ok()) {
    std::cerr << "Set up ros color camera " << name_ << " first"
              << std::endl;
    return false;
  }
  auto image_msg = ros::topic::waitForMessage<sensor_msgs::Image>(depth_image_topic_, nh_);
  if (image_msg == nullptr)
    return false;
  image_ = cv_bridge::toCvCopy(image_msg, "16UC1")->image;
  if (image_.empty()) {
    std::cerr << "Could not update image from the topic " << depth_image_topic_ << std::endl;
    return false;
  }
  return true;
}
}