//
// Created by baozhe on 22-9-20.
//

#ifndef ICG_ROS_ICG_ROS_INCLUDE_ICG_ROS_ICG_ROS_INTERFACE_H_
#define ICG_ROS_ICG_ROS_INCLUDE_ICG_ROS_ICG_ROS_INTERFACE_H_

#include <iostream>
#include <utility>
#include <vector>
#include <string>

#include <filesystem/filesystem.h>
#include <icg/basic_depth_renderer.h>
#include <icg/body.h>
#include <icg/common.h>
#include <icg/depth_modality.h>
#include <icg/depth_model.h>
#include <icg/normal_viewer.h>
#include <icg/region_modality.h>
#include <icg/region_model.h>
#include <icg/renderer_geometry.h>
#include <icg/static_detector.h>
#include <icg/tracker.h>

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <memory>
#include <icg_ros/ros_camera.h>

namespace icg_ros
{

struct ICG_ROS_Config
{
  std::filesystem::path config_dir;
  std::vector<std::string> body_names;
  std::string tracker_name;
  std::string renderer_geometry_name;
  std::string color_camera_name;
  std::string color_camera_topic;
  std::string color_camera_info_topic;
  std::string depth_camera_name;
  std::string depth_camera_topic;
  std::string depth_camera_info_topic;
  std::string color_viewer_name;
  std::string depth_viewer_name;
  std::string color_depth_renderer_name;
  std::string depth_depth_renderer_name;
  std::string detector_suffix = "detector";
  std::string region_model_suffix = "region_model";
  std::string depth_model_suffix = "depth_model";
  std::string region_modality_suffix = "region_modality";
  std::string depth_modality_suffix = "depth_modality";
  std::string optimizer_suffix = "optimizer";
};

class ICG_ROS
{
 public:

  ICG_ROS(ros::NodeHandle &nh,
          const ICG_ROS_Config &config);

  inline std::shared_ptr<icg::Tracker> &GetTrackerPtr() { return tracker_ptr_; }

  bool RunTrackerProcessOneFrame(int iteration);

 private:
  ros::NodeHandle nh_;
  ICG_ROS_Config config_;
  std::shared_ptr<icg::Tracker> tracker_ptr_;


};
}

#endif //ICG_ROS_ICG_ROS_INCLUDE_ICG_ROS_ICG_ROS_INTERFACE_H_
