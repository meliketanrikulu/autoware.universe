// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pointcloud_preprocessor/distortion_corrector/distortion_corrector.hpp"

#include "pointcloud_preprocessor/distortion_corrector/undistorter.hpp"
#include "tier4_autoware_utils/math/trigonometry.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

// delete this
#include <chrono>
#include <deque>
#include <optional>
#include <string>
#include <utility>

namespace pointcloud_preprocessor
{
/** @brief Constructor. */
DistortionCorrectorComponent::DistortionCorrectorComponent(const rclcpp::NodeOptions & options)
: Node("distortion_corrector_node", options)
{
  // initialize debug tool
  {
    using autoware::universe_utils::DebugPublisher;
    using autoware::universe_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "distortion_corrector");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  // Parameter
  time_stamp_field_name_ = declare_parameter("time_stamp_field_name", "time_stamp");
  use_imu_ = declare_parameter("use_imu", true);
  use_3d_distortion_correction_ = declare_parameter("use_3d_distortion_correction", true);

  // Publisher
  undistorted_points_pub_ =
    this->create_publisher<PointCloud2>("~/output/pointcloud", rclcpp::SensorDataQoS());

  // Subscriber
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "~/input/twist", 10,
    std::bind(
      &DistortionCorrectorComponent::onTwistWithCovarianceStamped, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "~/input/imu", 10,
    std::bind(&DistortionCorrectorComponent::onImu, this, std::placeholders::_1));
  input_points_sub_ = this->create_subscription<PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&DistortionCorrectorComponent::onPointCloud, this, std::placeholders::_1));

  // Setup the undistortor

  if (use_3d_distortion_correction_) {
    undistorter_ = std::make_unique<Undistorter3D>(tf2_buffer);
  } else {
    undistorter_ = std::make_unique<Undistorter2D>(tf2_buffer);
  }
}

void DistortionCorrectorComponent::onTwistWithCovarianceStamped(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg)
{
  undistorter_->processTwistMessage(twist_msg);
}

void DistortionCorrectorComponent::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  if (!use_imu_) {
    return;
  }

  undistorter_->processIMUMessage(base_link_frame_, imu_msg);
}

void DistortionCorrectorComponent::onPointCloud(PointCloud2::UniquePtr pointcloud_msg)
{
  stop_watch_ptr_->toc("processing_time", true);
  const auto points_sub_count = undistorted_points_pub_->get_subscription_count() +
                                undistorted_points_pub_->get_intra_process_subscription_count();

  if (points_sub_count < 1) {
    return;
  }

  undistorter_->setPointCloudTransform(base_link_frame_, pointcloud_msg->header.frame_id);

  undistorter_->initialize();
  undistorter_->undistortPointCloud(use_imu_, *pointcloud_msg);

  if (debug_publisher_) {
    auto pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (this->get_clock()->now() - pointcloud_msg->header.stamp).nanoseconds()))
        .count();
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }

  undistorted_points_pub_->publish(std::move(pointcloud_msg));

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::DistortionCorrectorComponent)
