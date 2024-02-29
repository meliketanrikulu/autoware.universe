// Copyright 2024 The Autoware Foundation
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

#include "include/aw_pose_covariance_modifier_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

AWPoseCovarianceModifierNode::AWPoseCovarianceModifierNode() : Node("AWPoseCovarianceModifierNode")
{
  trusted_pose_with_cov_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "input_trusted_pose_with_cov_topic", 10000,
      std::bind(
        &AWPoseCovarianceModifierNode::trusted_pose_with_cov_callback, this,
        std::placeholders::_1));

  new_pose_estimator_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "output_pose_with_covariance_topic", 10);

  debug_pose_with_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/aw_pose_covariance_modifier/debug/pose_with_cov_stamped",10);

  client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/localization/pose_estimator/ndt_scan_matcher/set_parameters");



  while(startNDTCovModifier != true && rclcpp::ok()){
      startNDTCovModifier = AWPoseCovarianceModifierNode::callNDTCovarianceModifier();
  }
  if (startNDTCovModifier == 1) {
    RCLCPP_INFO(get_logger(), "NDT pose covariance modifier activated ...");
  }
  else{
      startNDTCovModifier = AWPoseCovarianceModifierNode::callNDTCovarianceModifier();
  }
}
std::string futureStatusToString(std::future_status status) {
    switch (status) {
        case std::future_status::ready:
            return "ready";
        case std::future_status::timeout:
            return "timeout";
        case std::future_status::deferred:
            return "deferred";
        default:
            return "unknown";
    }
}
bool AWPoseCovarianceModifierNode::callNDTCovarianceModifier() {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    rcl_interfaces::msg::Parameter parameter;
    parameter.name = "aw_pose_covariance_modifier.enable";
    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    parameter.value.bool_value = true;

    request->parameters.push_back(parameter);

    auto future_result = client_->async_send_request(request);

    auto result_status = future_result.wait_for(std::chrono::seconds(10));
    std::string status_str = futureStatusToString(result_status);
    RCLCPP_INFO(this->get_logger(), "Result Status: %s", status_str.c_str());

        if (result_status == std::future_status::ready) {

            auto response = future_result.get();

            if (response && response->results.data()) {
                RCLCPP_INFO(this->get_logger(), "Parametreler başarıyla ayarlandı.");
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Parametre ayarlanırken bir hata oluştu.");
                return false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "İstek belirtilen süre içinde tamamlanmadı.");
            return false;
        }

}

void AWPoseCovarianceModifierNode::trusted_pose_with_cov_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose_estimator_pose = *msg;

  trusted_pose_rmse_ = (std::sqrt(pose_estimator_pose.pose.covariance[0]) +
                        std::sqrt(pose_estimator_pose.pose.covariance[7])) /
                       2;
  trusted_pose_yaw_rmse_in_degrees_ =
    std::sqrt(pose_estimator_pose.pose.covariance[35]) * 180 / M_PI;

  if (trusted_pose_rmse_ > 0.25) {
    RCLCPP_INFO(
      this->get_logger(),
      "Trusted Pose RMSE is under the threshold. It will not be used as a pose source.");
  } else {
    if (trusted_pose_yaw_rmse_in_degrees_ >= 0.3) {
      pose_estimator_pose.pose.covariance[35] = 1000000;
    }

        if (trusted_pose_yaw_rmse_in_degrees_ >= 0.3){
            pose_estimator_pose.pose.covariance[35] = 1000000;
        }

        new_pose_estimator_pub_->publish(pose_estimator_pose);

    }

}
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AWPoseCovarianceModifierNode>());
  rclcpp::shutdown();
  return 0;
}
