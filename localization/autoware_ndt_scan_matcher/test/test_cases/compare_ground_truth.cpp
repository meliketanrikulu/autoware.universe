// Copyright 2023 Autoware Foundation
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

#ifndef TEST_CASES__COMPARE_GROUND_TRUTH_HPP_
#define TEST_CASES__COMPARE_GROUND_TRUTH_HPP_

#include "../test_fixture.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rcl_yaml_param_parser/parser.h>
#include "../rosbag_parser.hpp"
#include "../pcd_loader.hpp"
#include <memory>
#include <string>
#include <vector>



TEST_F(TestNDTScanMatcher, compare_ground_truth)  // NOLINT
{
  //---------//
  // Arrange //
  //---------//
  std::thread t1([&]() {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node_);
    exec.spin();
  });
  std::thread t2([&]() { rclcpp::spin(pcd_loader_); });


  //-----//
  // Act //
  //-----//
  // (1) trigger initial pose estimation
  EXPECT_TRUE(trigger_node_client_->send_trigger_node(true));


  auto rosbag_parser_node_ = std::make_shared<RosbagParserNode>(
    "/home/melike/rosbags/urban_shared_data/only_localization_test/sliced_bag/sliced_bag_0.db3");


  // (2) publish LiDAR point cloud
  // const sensor_msgs::msg::PointCloud2 input_cloud = make_default_sensor_pcd();
  const sensor_msgs::msg::PointCloud2 input_cloud = *rosbag_parser_node_->get_pc();
  RCLCPP_INFO_STREAM(node_->get_logger(), "xx sensor cloud size: " << input_cloud.width);
  sensor_pcd_publisher_->publish_pcd(input_cloud);

  // (3) send initial pose
  // const geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg =
  //   make_pose(/* x = */ 100.0, /* y = */ 100.0);
  const geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg = *rosbag_parser_node_->get_pose_with_cov();
  const geometry_msgs::msg::Pose result_pose =
    initialpose_client_->send_initialpose(initial_pose_msg).pose.pose;

  input_pose_with_covariance_publisher_->publish(*rosbag_parser_node_->get_pose_with_cov());

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov;
  result_pose_with_cov.pose.pose = result_pose;
  result_pose_with_cov.header.frame_id= "map";
  output_pose_with_covariance_publisher_->publish(result_pose_with_cov);

  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr ndt_output_msg;

  ndt_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
"/ndt_pose_with_covariance", 1, [&](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  RCLCPP_INFO_STREAM(node_->get_logger(), "xx ndt_pose: " << msg->pose.pose.position.x << ", "
                                                        << msg->pose.pose.position.y << ", "
                                                        << msg->pose.pose.position.z);
  ndt_output_msg = msg;
  });
  int counter = 0;
  while(counter < 200){
    std::cout<<"counter: "<<counter<<std::endl;
    ekf_pose_with_covariance_publisher_->publish(*rosbag_parser_node_->get_pose_with_cov());
    ekf_pose_with_covariance_publisher_->publish(*rosbag_parser_node_->get_pose_with_cov());
    sensor_pcd_publisher_->publish_pcd(*rosbag_parser_node_->get_pc());
    counter++;

  }

  //--------//
  // Assert //
  //--------//
  RCLCPP_INFO_STREAM(
    node_->get_logger(), std::fixed << "result_pose: " << result_pose.position.x << ", "
                                    << result_pose.position.y << ", " << result_pose.position.z);
  EXPECT_NEAR(ndt_output_msg->pose.pose.position.x, initial_pose_msg.pose.pose.position.x, 2.0);
  EXPECT_NEAR(ndt_output_msg->pose.pose.position.y, initial_pose_msg.pose.pose.position.y, 2.0);
  EXPECT_NEAR(ndt_output_msg->pose.pose.position.z, initial_pose_msg.pose.pose.position.z, 2.0);

  rclcpp::shutdown();
  t1.join();
  t2.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

#endif  // TEST_CASES__COMPARE_GROUND_TRUTH_HPP_
