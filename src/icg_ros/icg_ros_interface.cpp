//
// Created by baozhe on 22-9-20.
//

#include "icg_ros/icg_ros_interface.h"

namespace icg_ros
{


ICG_ROS::ICG_ROS(ros::NodeHandle &nh, const icg_ros::ICG_ROS_Config &config)
    : nh_{nh}, config_{config}
{
  constexpr bool kUseDepthViewer = true;
  constexpr bool kMeasureOcclusions = true;
  constexpr bool kModelOcclusions = false;
  constexpr bool kVisualizePoseResult = false;
  constexpr bool kSaveImages = false;
  tracker_ptr_ = std::make_shared<icg::Tracker>(config_.tracker_name);
  renderer_geometry_ptr_ = std::make_shared<icg::RendererGeometry>(config_.renderer_geometry_name);
  color_camera_ptr_ = std::make_shared<RosColorCamera>(config_.color_camera_name,
                                                       config_.color_camera_topic,
                                                       config_.color_camera_info_topic,
                                                       nh_);
  depth_camera_ptr_ = std::make_shared<RosDepthCamera>(config_.depth_camera_name,
                                                       config_.depth_camera_topic,
                                                       config_.depth_camera_info_topic,
                                                       nh_);
  color_viewer_ptr_ = std::make_shared<icg::NormalColorViewer>(config_.color_viewer_name,
                                                               color_camera_ptr_,
                                                               renderer_geometry_ptr_);
  // TODO:
  // if (kSaveImages) color_viewer_ptr_->StartSavingImages(save_directory, "bmp");

  tracker_ptr_->AddViewer(color_viewer_ptr_);
  if (kUseDepthViewer) {
    depth_viewer_ptr_ = std::make_shared<icg::NormalDepthViewer>(config_.depth_viewer_name,
                                                                 depth_camera_ptr_,
                                                                 renderer_geometry_ptr_,
                                                                 0.3f, 2.0f); // min and max depth
    // TODO:
    // if (kSaveImages) depth_viewer_ptr_->StartSavingImages(save_directory, "bmp");
    tracker_ptr_->AddViewer(depth_viewer_ptr_);
  } else {
    depth_viewer_ptr_ = nullptr;
  }

  // set up depth renderer
  color_depth_renderer_ptr_ = std::make_shared<icg::FocusedBasicDepthRenderer>(
      config_.color_depth_renderer_name,
      renderer_geometry_ptr_,
      color_camera_ptr_);
  depth_depth_renderer_ptr_ = std::make_shared<icg::FocusedBasicDepthRenderer>(
      config_.depth_depth_renderer_name,
      renderer_geometry_ptr_,
      depth_camera_ptr_);

  for (const auto body_name : config_.body_names) {
    // set up body
    std::filesystem::path metafile_path{config_.config_dir / (body_name + ".yaml")};
    auto body_ptr{std::make_shared<icg::Body>(body_name, metafile_path)};
    renderer_geometry_ptr_->AddBody(body_ptr);
    color_depth_renderer_ptr_->AddReferencedBody(body_ptr);
    depth_depth_renderer_ptr_->AddReferencedBody(body_ptr);

    // Set up detector
    std::filesystem::path detector_path{config_.config_dir /
        (body_name + "_detector.yaml")};
    auto detector_ptr{std::make_shared<icg::StaticDetector>(
        body_name + "_detector", detector_path, body_ptr)};
    tracker_ptr_->AddDetector(detector_ptr);

    // Set up models
    auto region_model_ptr{std::make_shared<icg::RegionModel>(
        body_name + "_region_model", body_ptr,
        config_.config_dir / (body_name + "_region_model.bin"))};
    auto depth_model_ptr{std::make_shared<icg::DepthModel>(
        body_name + "_depth_model", body_ptr,
        config_.config_dir / (body_name + "_depth_model.bin"))};

    // Set up modalities
    auto region_modality_ptr{std::make_shared<icg::RegionModality>(
        body_name + "_region_modality", body_ptr, color_camera_ptr_,
        region_model_ptr)};
    auto depth_modality_ptr{std::make_shared<icg::DepthModality>(
        body_name + "_depth_modality", body_ptr, depth_camera_ptr_,
        depth_model_ptr)};
    if (kVisualizePoseResult) {
      region_modality_ptr->set_visualize_pose_result(true);
    }
    if (kMeasureOcclusions) {
      region_modality_ptr->MeasureOcclusions(depth_camera_ptr_);
      depth_modality_ptr->MeasureOcclusions();
    }
    if (kModelOcclusions) {
      region_modality_ptr->ModelOcclusions(color_depth_renderer_ptr_);
      depth_modality_ptr->ModelOcclusions(depth_depth_renderer_ptr_);
    }

    // Set up optimizer
    auto body1_optimizer_ptr{
        std::make_shared<icg::Optimizer>(body_name + "_optimizer")};
    body1_optimizer_ptr->AddModality(region_modality_ptr);
    body1_optimizer_ptr->AddModality(depth_modality_ptr);
    tracker_ptr_->AddOptimizer(body1_optimizer_ptr);
  }

  if (!tracker_ptr_->SetUp()) {
    std::cerr << "CANNOT SETUP. EXIT..." << std::endl;
    if (!ros::isShuttingDown()) {
      ros::shutdown();
    }
    exit(1);
  }
}

bool ICG_ROS::RunTrackerProcessOneFrame(bool execution_detection, bool start_tracking, int iteration)
{
  if (!tracker_ptr_->set_up()) {
    std::cerr << "Set up tracker " << tracker_ptr_->name() << " first" << std::endl;
    return false;
  }

  tracker_ptr_->tracking_started_ = false;
  tracker_ptr_->quit_tracker_process_ = false;
  tracker_ptr_->execute_detection_ = execution_detection;
  tracker_ptr_->start_tracking_ = start_tracking;

  auto begin{std::chrono::high_resolution_clock::now()};
  if (!tracker_ptr_->UpdateCameras(tracker_ptr_->execute_detection_)) return false;
  if (tracker_ptr_->execute_detection_) {
    if (!tracker_ptr_->ExecuteDetectionCycle(iteration)) return false;
    tracker_ptr_->tracking_started_ = false;
    tracker_ptr_->execute_detection_ = false;
  }
  if (tracker_ptr_->start_tracking_) {
    if (!tracker_ptr_->StartModalities(iteration)) return false;
    tracker_ptr_->tracking_started_ = true;
    tracker_ptr_->start_tracking_ = false;
  }
  if (tracker_ptr_->tracking_started_) {
    if (!tracker_ptr_->ExecuteTrackingCycle(iteration)) return false;
  }
  if (!tracker_ptr_->UpdateViewers(iteration)) return false;
  if (tracker_ptr_->quit_tracker_process_) return true;
  if (!tracker_ptr_->synchronize_cameras_) tracker_ptr_->WaitUntilCycleEnds(begin);

  return true;
}


}

