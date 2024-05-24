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

#include "pointcloud_preprocessor/distortion_corrector/undistorter.hpp"

#include "tier4_autoware_utils/math/trigonometry.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include <deque>
#include <optional>
#include <string>
#include <utility>

namespace pointcloud_preprocessor
{

template <class Derived>
void Undistorter<Derived>::processTwistMessage(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header = twist_msg->header;
  msg.twist = twist_msg->twist.twist;
  twist_queue.push_back(msg);

  while (!twist_queue.empty()) {
    // for replay rosbag
    if (rclcpp::Time(twist_queue.front().header.stamp) > rclcpp::Time(twist_msg->header.stamp)) {
      twist_queue.pop_front();
    } else if (  // NOLINT
      rclcpp::Time(twist_queue.front().header.stamp) <
      rclcpp::Time(twist_msg->header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
      twist_queue.pop_front();
    }
    break;
  }
}

template <class Derived>
void Undistorter<Derived>::processIMUMessage(
  const std::string & base_link_frame, const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  geometry_msgs::msg::TransformStamped::SharedPtr geometry_imu_to_base_link_ptr =
    std::make_shared<geometry_msgs::msg::TransformStamped>();
  getIMUTransformation(base_link_frame, imu_msg->header.frame_id, geometry_imu_to_base_link_ptr);
  storeIMUToQueue(imu_msg, geometry_imu_to_base_link_ptr);
}

template <class Derived>
void Undistorter<Derived>::getIMUTransformation(
  const std::string & base_link_frame, const std::string & imu_frame,
  geometry_msgs::msg::TransformStamped::SharedPtr geometry_imu_to_base_link_ptr)
{
  if (is_imu_transfrom_exist) {
    return;
  }

  tf2::Transform tf2_imu_to_base_link;
  if (base_link_frame == imu_frame) {
    tf2_imu_to_base_link.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_imu_to_base_link.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    is_imu_transfrom_exist = true;
  } else {
    try {
      const auto transform_msg =
        tf2_buffer.lookupTransform(base_link_frame, imu_frame, tf2::TimePointZero);
      tf2::convert(transform_msg.transform, tf2_imu_to_base_link);
      is_imu_transfrom_exist = true;
    } catch (const tf2::TransformException & ex) {
      // RCLCPP_WARN(get_logger(), "%s", ex.what());
      // RCLCPP_ERROR(
      //   get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

      tf2_imu_to_base_link.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
      tf2_imu_to_base_link.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    }
  }

  geometry_imu_to_base_link_ptr->transform.rotation =
    tf2::toMsg(tf2_imu_to_base_link.getRotation());
}

template <class Derived>
void Undistorter<Derived>::storeIMUToQueue(
  const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
  geometry_msgs::msg::TransformStamped::SharedPtr geometry_imu_to_base_link_ptr)
{
  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.vector = imu_msg->angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, *geometry_imu_to_base_link_ptr);
  transformed_angular_velocity.header = imu_msg->header;
  angular_velocity_queue.push_back(transformed_angular_velocity);

  while (!angular_velocity_queue.empty()) {
    // for replay rosbag
    if (
      rclcpp::Time(angular_velocity_queue.front().header.stamp) >
      rclcpp::Time(imu_msg->header.stamp)) {
      angular_velocity_queue.pop_front();
    } else if (  // NOLINT
      rclcpp::Time(angular_velocity_queue.front().header.stamp) <
      rclcpp::Time(imu_msg->header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
      angular_velocity_queue.pop_front();
    }
    break;
  }
}

template <class Derived>
void Undistorter<Derived>::getIteratorOfTwistAndIMU(
  bool use_imu, double first_point_time_stamp_sec,
  std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu)
{
  it_twist = std::lower_bound(
    std::begin(twist_queue), std::end(twist_queue), first_point_time_stamp_sec,
    [](const geometry_msgs::msg::TwistStamped & x, const double t) {
      return rclcpp::Time(x.header.stamp).seconds() < t;
    });
  it_twist = it_twist == std::end(twist_queue) ? std::end(twist_queue) - 1 : it_twist;

  if (use_imu && !angular_velocity_queue.empty()) {
    it_imu = std::lower_bound(
      std::begin(angular_velocity_queue), std::end(angular_velocity_queue),
      first_point_time_stamp_sec, [](const geometry_msgs::msg::Vector3Stamped & x, const double t) {
        return rclcpp::Time(x.header.stamp).seconds() < t;
      });
    it_imu =
      it_imu == std::end(angular_velocity_queue) ? std::end(angular_velocity_queue) - 1 : it_imu;
  }
}

template <class Derived>
bool Undistorter<Derived>::isInputValid(sensor_msgs::msg::PointCloud2 & pointcloud)
{
  if (pointcloud.data.empty() || twist_queue.empty()) {
    // RCLCPP_WARN_STREAM_THROTTLE(
    //   get_logger(), *get_clock(), 10000 /* ms */, "input pointcloud or twist_queue_ is empty.");
    return false;
  }

  auto time_stamp_field_it = std::find_if(
    std::cbegin(pointcloud.fields), std::cend(pointcloud.fields),
    [](const sensor_msgs::msg::PointField & field) { return field.name == "time_stamp"; });
  if (time_stamp_field_it == pointcloud.fields.cend()) {
    // RCLCPP_WARN_STREAM_THROTTLE(
    //   get_logger(), *get_clock(), 10000 /* ms */,
    //   "Required field time stamp doesn't exist in the point cloud.");
    return false;
  }
  return true;
}

template <class Derived>
void Undistorter<Derived>::undistortPointCloud(
  bool use_imu, sensor_msgs::msg::PointCloud2 & pointcloud)
{
  if (!isInputValid(pointcloud)) return;

  sensor_msgs::PointCloud2Iterator<float> it_x(pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(pointcloud, "z");
  sensor_msgs::PointCloud2ConstIterator<double> it_time_stamp(pointcloud, "time_stamp");

  double prev_time_stamp_sec{*it_time_stamp};
  const double first_point_time_stamp_sec{*it_time_stamp};

  std::deque<geometry_msgs::msg::TwistStamped>::iterator it_twist;
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator it_imu;
  getIteratorOfTwistAndIMU(use_imu, first_point_time_stamp_sec, it_twist, it_imu);

  // For performance, do not instantiate `rclcpp::Time` inside of the for-loop
  double twist_stamp = rclcpp::Time(it_twist->header.stamp).seconds();
  double imu_stamp{0.0};
  if (use_imu && !angular_velocity_queue.empty()) {
    imu_stamp = rclcpp::Time(it_imu->header.stamp).seconds();
  }

  // If there is a point in a pointlcoud that cannot be associated, record it to issue a warning
  bool is_twist_time_stamp_too_late = false;
  bool is_imu_time_stamp_is_too_late = false;
  bool is_twist_valid = true;
  bool is_imu_valid = true;

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_time_stamp) {
    is_twist_valid = true;
    is_imu_valid = true;

    // Get closest twist information
    while (it_twist != std::end(twist_queue) - 1 && *it_time_stamp > twist_stamp) {
      ++it_twist;
      twist_stamp = rclcpp::Time(it_twist->header.stamp).seconds();
    }
    if (std::abs(*it_time_stamp - twist_stamp) > 0.1) {
      is_twist_time_stamp_too_late = true;
      is_twist_valid = false;
    }

    // Get closest IMU information
    if (use_imu && !angular_velocity_queue.empty()) {
      while (it_imu != std::end(angular_velocity_queue) - 1 && *it_time_stamp > imu_stamp) {
        ++it_imu;
        imu_stamp = rclcpp::Time(it_imu->header.stamp).seconds();
      }

      if (std::abs(*it_time_stamp - imu_stamp) > 0.1) {
        is_imu_time_stamp_is_too_late = true;
        is_imu_valid = false;
      }
    } else {
      is_imu_valid = false;
    }

    float time_offset = static_cast<float>(*it_time_stamp - prev_time_stamp_sec);

    // std::cout << "before undistortPoint" << std::endl;
    // std::cout << "it_x: " << *it_x << " it_y: " << *it_y << " it_z: " << *it_z << std::endl;

    // Undistorted a single point based on the strategy
    // auto start = std::chrono::high_resolution_clock::now();
    undistortPoint(it_x, it_y, it_z, it_twist, it_imu, time_offset, is_twist_valid, is_imu_valid);
    // std::cout << "after undistortPoint" << std::endl;
    // std::cout << "it_x: " << *it_x << " it_y: " << *it_y << " it_z: " << *it_z << std::endl;
    // std::cout << "//////////////////\n" << std::endl;
    prev_time_stamp_sec = *it_time_stamp;

    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> duration = end - start;
    // std::cout << "Function execution time: " << duration.count() << " seconds" << std::endl;
  }

  warnIfTimestampsTooLate(is_twist_time_stamp_too_late, is_imu_time_stamp_is_too_late);
}

template <class Derived>
void Undistorter<Derived>::warnIfTimestampsTooLate(
  bool is_twist_time_stamp_too_late, bool is_imu_time_stamp_is_too_late)
{
  if (is_twist_time_stamp_too_late) {
    // RCLCPP_WARN_STREAM_THROTTLE(
    //   get_logger(), *get_clock(), 10000 /* ms */,
    //   "twist time_stamp is too late. Could not interpolate.");
    std::cout << "twist time_stamp is too late. Could not interpolate." << std::endl;
  }

  if (is_imu_time_stamp_is_too_late) {
    // RCLCPP_WARN_STREAM_THROTTLE(
    //   get_logger(), *get_clock(), 10000 /* ms */,
    //   "imu time_stamp is too late. Could not interpolate.");
    std::cout << "imu time_stamp is too late. Could not interpolate." << std::endl;
  }
}

void Undistorter2D::initialize()
{
  x = 0.0f;
  y = 0.0f;
  theta = 0.0f;
}

void Undistorter3D::initialize()
{
  prev_transformation_matrix = Eigen::Matrix4f::Identity();
}

void Undistorter2D::setPointCloudTransform(
  const std::string & base_link_frame, const std::string & lidar_frame)
{
  if (is_pointcloud_transfrom_exist) {
    return;
  }

  if (base_link_frame == lidar_frame) {
    tf2_lidar_to_base_link.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_lidar_to_base_link.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    tf2_base_link_to_lidar = tf2_lidar_to_base_link;
    is_pointcloud_transfrom_exist = true;
  } else {
    try {
      const auto transform_msg =
        tf2_buffer.lookupTransform(base_link_frame, lidar_frame, tf2::TimePointZero);
      tf2::convert(transform_msg.transform, tf2_lidar_to_base_link);
      tf2_base_link_to_lidar = tf2_lidar_to_base_link.inverse();
      is_pointcloud_transfrom_exist = true;
      is_pointcloud_transform_needed = true;
    } catch (const tf2::TransformException & ex) {
      // RCLCPP_WARN(get_logger(), "%s", ex.what());
      // RCLCPP_ERROR(
      //   get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

      tf2_lidar_to_base_link.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
      tf2_lidar_to_base_link.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
      tf2_base_link_to_lidar = tf2_lidar_to_base_link;
    }
  }
}

void Undistorter3D::setPointCloudTransform(
  const std::string & base_link_frame, const std::string & lidar_frame)
{
  if (is_pointcloud_transfrom_exist) {
    return;
  }

  if (base_link_frame == lidar_frame) {
    eigen_lidar_to_base_link = Eigen::Matrix4f::Identity();
    eigen_base_link_to_lidar = Eigen::Matrix4f::Identity();
    is_pointcloud_transfrom_exist = true;
  }

  try {
    const auto transform_msg =
      tf2_buffer.lookupTransform(base_link_frame, lidar_frame, tf2::TimePointZero);
    eigen_lidar_to_base_link =
      tf2::transformToEigen(transform_msg.transform).matrix().cast<float>();
    eigen_base_link_to_lidar = eigen_lidar_to_base_link.inverse();
    is_pointcloud_transfrom_exist = true;
    is_pointcloud_transform_needed = true;
  } catch (const tf2::TransformException & ex) {
    // RCLCPP_WARN(get_logger(), "%s", ex.what());
    // RCLCPP_ERROR(
    //   get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());
    eigen_lidar_to_base_link = Eigen::Matrix4f::Identity();
    eigen_base_link_to_lidar = Eigen::Matrix4f::Identity();
  }
}

void Undistorter2D::implementation(
  sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
  sensor_msgs::PointCloud2Iterator<float> & it_z,
  std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, float & time_offset,
  bool & is_twist_valid, bool & is_imu_valid)
{
  // Initialize linear velocity and angular velocity
  float v{0.0f}, w{0.0f};
  if (is_twist_valid) {
    v = static_cast<float>(it_twist->twist.linear.x);
    w = static_cast<float>(it_twist->twist.angular.z);
  }
  if (is_imu_valid) {
    w = static_cast<float>(it_imu->vector.z);
  }

  // Undistort point
  point_tf.setValue(*it_x, *it_y, *it_z);

  if (is_pointcloud_transform_needed) {
    point_tf = tf2_lidar_to_base_link * point_tf;
  }
  theta += w * time_offset;
  baselink_quat.setValue(
    0, 0, tier4_autoware_utils::sin(theta * 0.5f),
    tier4_autoware_utils::cos(theta * 0.5f));  // baselink_quat.setRPY(0.0, 0.0, theta);
  const float dis = v * time_offset;
  x += dis * tier4_autoware_utils::cos(theta);
  y += dis * tier4_autoware_utils::sin(theta);

  baselink_tf_odom.setOrigin(tf2::Vector3(x, y, 0.0));
  baselink_tf_odom.setRotation(baselink_quat);

  undistorted_point_tf = baselink_tf_odom * point_tf;

  if (is_pointcloud_transform_needed) {
    undistorted_point_tf = tf2_base_link_to_lidar * undistorted_point_tf;
  }

  *it_x = static_cast<float>(undistorted_point_tf.getX());
  *it_y = static_cast<float>(undistorted_point_tf.getY());
  *it_z = static_cast<float>(undistorted_point_tf.getZ());
}

void Undistorter3D::implementation(
  sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
  sensor_msgs::PointCloud2Iterator<float> & it_z,
  std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, float & time_offset,
  bool & is_twist_valid, bool & is_imu_valid)
{
  // Initialize linear velocity and angular velocity
  float v_x_{0.0f}, v_y_{0.0f}, v_z_{0.0f}, w_x_{0.0f}, w_y_{0.0f}, w_z_{0.0f};
  if (is_twist_valid) {
    v_x_ = static_cast<float>(it_twist->twist.linear.x);
    v_y_ = static_cast<float>(it_twist->twist.linear.y);
    v_z_ = static_cast<float>(it_twist->twist.linear.z);
    w_x_ = static_cast<float>(it_twist->twist.angular.x);
    w_y_ = static_cast<float>(it_twist->twist.angular.y);
    w_z_ = static_cast<float>(it_twist->twist.angular.z);
  }
  if (is_imu_valid) {
    w_x_ = static_cast<float>(it_imu->vector.x);
    w_y_ = static_cast<float>(it_imu->vector.y);
    w_z_ = static_cast<float>(it_imu->vector.z);
  }

  // Undistort point
  point_eigen << *it_x, *it_y, *it_z, 1.0;
  if (is_pointcloud_transform_needed) {
    point_eigen = eigen_lidar_to_base_link * point_eigen;
  }

  Sophus::SE3f::Tangent twist(v_x_, v_y_, v_z_, w_x_, w_y_, w_z_);
  twist = twist * time_offset;
  transformation_matrix = Sophus::SE3f::exp(twist).matrix();
  transformation_matrix = transformation_matrix * prev_transformation_matrix;
  undistorted_point_eigen = transformation_matrix * point_eigen;

  if (is_pointcloud_transform_needed) {
    undistorted_point_eigen = eigen_base_link_to_lidar * undistorted_point_eigen;
  }
  *it_x = undistorted_point_eigen[0];
  *it_y = undistorted_point_eigen[1];
  *it_z = undistorted_point_eigen[2];

  prev_transformation_matrix = transformation_matrix;
}

}  // namespace pointcloud_preprocessor
