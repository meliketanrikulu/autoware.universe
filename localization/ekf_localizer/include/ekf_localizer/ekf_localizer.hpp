// Copyright 2018-2019 Autoware Foundation
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

#ifndef EKF_LOCALIZER__EKF_LOCALIZER_HPP_
#define EKF_LOCALIZER__EKF_LOCALIZER_HPP_

#include "ekf_localizer/aged_object_queue.hpp"
#include "ekf_localizer/ekf_module.hpp"
#include "ekf_localizer/hyper_parameters.hpp"
#include "ekf_localizer/warning.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/logger_level_configure.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tier4_debug_msgs/msg/float64_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <deque>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <vector>

class Simple1DFilter
{
public:
  Simple1DFilter()
  {
    initialized_ = false;
    x_ = 0;
    dev_ = 1e9;
    proc_dev_x_c_ = 0.0;
  };
  void init(const double init_obs, const double obs_dev, const rclcpp::Time & time)
  {
    x_ = init_obs;
    dev_ = obs_dev;
    latest_time_ = time;
    initialized_ = true;
  };
  void update(const double obs, const double obs_dev, const rclcpp::Time & time)
  {
    if (!initialized_) {
      init(obs, obs_dev, time);
      return;
    }

    // Prediction step (current stddev_)
    double dt = (time - latest_time_).seconds();
    double proc_dev_x_d = proc_dev_x_c_ * dt * dt;
    dev_ = dev_ + proc_dev_x_d;

    // Update step
    double kalman_gain = dev_ / (dev_ + obs_dev);
    x_ = x_ + kalman_gain * (obs - x_);
    dev_ = (1 - kalman_gain) * dev_;

    latest_time_ = time;
  };
  void set_proc_dev(const double proc_dev) { proc_dev_x_c_ = proc_dev; }
  [[nodiscard]] double get_x() const { return x_; }

private:
  bool initialized_;
  double x_;
  double dev_;
  double proc_dev_x_c_;
  rclcpp::Time latest_time_;
};

class EKFLocalizer : public rclcpp::Node
{
public:
  explicit EKFLocalizer(const rclcpp::NodeOptions & options);

private:
  const std::shared_ptr<Warning> warning_;

  //!< @brief ekf estimated pose publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  //!< @brief estimated ekf pose with covariance publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
  //!< @brief estimated ekf odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  //!< @brief ekf estimated twist publisher
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  //!< @brief ekf estimated twist with covariance publisher
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_cov_;
  //!< @brief ekf estimated yaw bias publisher
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64Stamped>::SharedPtr pub_yaw_bias_;
  //!< @brief ekf estimated yaw bias publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_biased_pose_;
  //!< @brief ekf estimated yaw bias publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_biased_pose_cov_;
  //!< @brief diagnostics publisher
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  //!< @brief initial pose subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
  //!< @brief measurement pose with covariance subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_with_cov_;
  //!< @brief measurement twist with covariance subscriber
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    sub_twist_with_cov_;
  //!< @brief imu subscriber
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  //!< @brief time for ekf calculation callback
  rclcpp::TimerBase::SharedPtr timer_control_;
  //!< @brief last predict time
  std::shared_ptr<const rclcpp::Time> last_predict_time_;
  //!< @brief trigger_node service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_trigger_node_;

  //!< @brief timer to send transform
  rclcpp::TimerBase::SharedPtr timer_tf_;
  //!< @brief tf broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  //!< @brief logger configure module
  std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;

  //!< @brief  extended kalman filter instance.
  std::unique_ptr<EKFModule> ekf_module_;
  Simple1DFilter z_filter_;
  Simple1DFilter roll_filter_;
  Simple1DFilter pitch_filter_;

  const HyperParameters params_;

  double ekf_dt_;

  //////////////////////////////////////////////////////////////////////////////

  /* process noise variance for discrete model */
  double proc_cov_yaw_d_;  //!< @brief  discrete yaw process noise
  double proc_cov_vx_d_;   //!< @brief  discrete process noise in d_vx=0
  double proc_cov_wz_d_;   //!< @brief  discrete process noise in d_wz=0

  bool is_activated_;

  double imu_dt_;
  bool start = 0;
  rclcpp::Time last_imu_time_ = rclcpp::Time(0, 0);
  std::deque<sensor_msgs::msg::Imu> imu_msg_deque_;
  sensor_msgs::msg::Imu find_closest_imu_msg(const rclcpp::Time & current_time);

  builtin_interfaces::msg::Time pose_time_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  struct RollPitchHeightStruct
  {
    double roll;
    double pitch;
    double z;
  };

  RollPitchHeightStruct get_roll_pitch_height(const rclcpp::Time current_time);

  rclcpp::Time prev_time_;
  double imu_counter = 0;
  //////////////////////////////////////////////////////////////////////////////

  EKFDiagnosticInfo pose_diag_info_;
  EKFDiagnosticInfo twist_diag_info_;

  AgedObjectQueue<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> pose_queue_;
  AgedObjectQueue<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> twist_queue_;

  /**
   * @brief computes update & prediction of EKF for each ekf_dt_[s] time
   */
  void timer_callback();

  /**
   * @brief publish tf for tf_rate [Hz]
   */
  void timer_tf_callback();

  /**
   * @brief set pose with covariance measurement
   */
  void callback_pose_with_covariance(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief set twist with covariance measurement
   */
  void callback_twist_with_covariance(
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  /**
   * @brief get imu data
   */
  void callback_imu(sensor_msgs::msg::Imu::SharedPtr msg);
  /**
   * @brief set initial_pose to current EKF pose
   */
  void callback_initial_pose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief update predict frequency
   */
  void update_predict_frequency(const rclcpp::Time & current_time);

  /**
   * @brief get transform from frame_id
   */
  bool get_transform_from_tf(
    std::string parent_frame, std::string child_frame,
    geometry_msgs::msg::TransformStamped & transform);

  /**
   * @brief publish current EKF estimation result
   */
  void publish_estimate_result(
    const geometry_msgs::msg::PoseStamped & current_ekf_pose,
    const geometry_msgs::msg::PoseStamped & current_biased_ekf_pose,
    const geometry_msgs::msg::TwistStamped & current_ekf_twist);

  /**
   * @brief publish diagnostics message
   */
  void publish_diagnostics(
    const geometry_msgs::msg::PoseStamped & current_ekf_pose, const rclcpp::Time & current_time);

  /**
   * @brief update simple 1d filter
   */
  void update_simple_1d_filters(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose, const size_t smoothing_step);

  /**
   * @brief initialize simple 1d filter
   */
  void init_simple_1d_filters(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

  /**
   * @brief trigger node
   */
  void service_trigger_node(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  autoware::universe_utils::StopWatch<std::chrono::milliseconds> stop_watch_;

  friend class EKFLocalizerTestSuite;  // for test code
};
#endif  // EKF_LOCALIZER__EKF_LOCALIZER_HPP_
