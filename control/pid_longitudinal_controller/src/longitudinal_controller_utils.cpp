// Copyright 2018-2021 Tier IV, Inc. All rights reserved.
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

#include "pid_longitudinal_controller/longitudinal_controller_utils.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include <experimental/optional>  // NOLINT

#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif

#include <algorithm>
#include <limits>

namespace autoware::motion::control::pid_longitudinal_controller
{
namespace longitudinal_utils
{

bool isValidTrajectory(const Trajectory & traj)
{
  for (const auto & p : traj.points) {
    if (
      !isfinite(p.pose.position.x) || !isfinite(p.pose.position.y) ||
      !isfinite(p.pose.position.z) || !isfinite(p.pose.orientation.w) ||
      !isfinite(p.pose.orientation.x) || !isfinite(p.pose.orientation.y) ||
      !isfinite(p.pose.orientation.z) || !isfinite(p.longitudinal_velocity_mps) ||
      !isfinite(p.lateral_velocity_mps) || !isfinite(p.acceleration_mps2) ||
      !isfinite(p.heading_rate_rps)) {
      return false;
    }
  }

  // when trajectory is empty
  if (traj.points.empty()) {
    return false;
  }

  return true;
}

double calcStopDistance(
  const Pose & current_pose, const Trajectory & traj, const double max_dist, const double max_yaw)
{
  const size_t seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    traj.points, current_pose, max_dist, max_yaw);
  const auto stop_idx_opt = motion_utils::searchZeroVelocityIndex(traj.points);
  const size_t end_idx = stop_idx_opt ? *stop_idx_opt : traj.points.size() - 1;
  //  if (end_idx == traj.points.size() - 1 && 1 >= int(end_idx - seg_idx)) {
  //    // Check the ego in front of the last point of trajectory or not
  //    const auto yaw = tier4_autoware_utils::getRPY(current_pose).z;
  //    const Eigen::Vector2d base_pose_vec(std::cos(yaw), std::sin(yaw));
  //    const Eigen::Vector2d vehicle_to_end_vec(
  //      traj.points.back().pose.position.x - current_pose.position.x,
  //      traj.points.back().pose.position.y - current_pose.position.y);
  //    if (base_pose_vec.dot(vehicle_to_end_vec) < 0.0) {
  //      return -1.0 *
  //             tier4_autoware_utils::calcDistance3d(current_pose, traj.points.at(end_idx).pose);
  //    }
  //    return tier4_autoware_utils::calcDistance3d(current_pose, traj.points.at(end_idx).pose);
  //  }
  const double signed_length_on_traj = motion_utils::calcSignedArcLength(
    traj.points, current_pose.position, seg_idx, traj.points.at(end_idx).pose.position,
    std::min(end_idx, traj.points.size() - 2));

  if (std::isnan(signed_length_on_traj)) {
    return 0.0;
  }
  return signed_length_on_traj;
}

double getPitchByPose(const Quaternion & quaternion_msg)
{
  double roll, pitch, yaw;
  tf2::Quaternion quaternion;
  tf2::fromMsg(quaternion_msg, quaternion);
  tf2::Matrix3x3{quaternion}.getRPY(roll, pitch, yaw);

  return pitch;
}

double getPitchByTraj(
  const Trajectory & trajectory, const size_t nearest_idx, const double wheel_base)
{
  // cannot calculate pitch
  if (trajectory.points.size() <= 1) {
    return 0.0;
  }
  // find the trajectory point after wheelbase
  const auto pose_after_distance =
    findTrajectoryPoseAfterDistance(nearest_idx, wheel_base, trajectory);
  TrajectoryPoint point_after_distance = trajectory.points.back();
  point_after_distance.pose = pose_after_distance;

  return calcElevationAngle(trajectory.points.at(nearest_idx), point_after_distance);
}

double calcElevationAngle(const TrajectoryPoint & p_from, const TrajectoryPoint & p_to)
{
  const double dx = p_from.pose.position.x - p_to.pose.position.x;
  const double dy = p_from.pose.position.y - p_to.pose.position.y;
  const double dz = p_from.pose.position.z - p_to.pose.position.z;

  const double dxy = std::max(std::hypot(dx, dy), std::numeric_limits<double>::epsilon());
  const double pitch = std::atan2(dz, dxy);

  return pitch;
}

std::pair<double, double> calcDistAndVelAfterTimeDelay(
  const double delay_time, const double current_vel, const double current_acc)
{
  if (delay_time <= 0.0) {
    return std::make_pair(0.0, 0.0);
  }

  // check time to stop
  const double time_to_stop = -current_vel / current_acc;

  const double delay_time_calculation =
    time_to_stop > 0.0 && time_to_stop < delay_time ? time_to_stop : delay_time;
  // simple linear prediction
  const double vel_after_delay = current_vel + current_acc * delay_time_calculation;
  const double running_distance = delay_time_calculation * current_vel + 0.5 * current_acc *
                                                                           delay_time_calculation *
                                                                           delay_time_calculation;
  return std::make_pair(running_distance, vel_after_delay);
}

double lerp(const double v_from, const double v_to, const double ratio)
{
  return v_from + (v_to - v_from) * ratio;
}

Quaternion lerpOrientation(const Quaternion & o_from, const Quaternion & o_to, const double ratio)
{
  tf2::Quaternion q_from, q_to;
  tf2::fromMsg(o_from, q_from);
  tf2::fromMsg(o_to, q_to);

  const auto q_interpolated = q_from.slerp(q_to, ratio);
  return tf2::toMsg(q_interpolated);
}

double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double max_val,
  const double min_val)
{
  const double diff_raw = (input_val - prev_val) / dt;
  const double diff = std::min(std::max(diff_raw, min_val), max_val);
  const double filtered_val = prev_val + diff * dt;
  return filtered_val;
}

double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double lim_val)
{
  const double max_val = std::fabs(lim_val);
  const double min_val = -max_val;
  return applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val);
}

geometry_msgs::msg::Pose findTrajectoryPoseAfterDistance(
  const size_t src_idx, const double distance,
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory)
{
  double remain_dist = distance;
  geometry_msgs::msg::Pose p = trajectory.points.back().pose;
  for (size_t i = src_idx; i < trajectory.points.size() - 1; ++i) {
    const double dist = tier4_autoware_utils::calcDistance3d(
      trajectory.points.at(i).pose, trajectory.points.at(i + 1).pose);
    if (remain_dist < dist) {
      if (remain_dist <= 0.0) {
        return trajectory.points.at(i).pose;
      }
      double ratio = remain_dist / dist;
      p = trajectory.points.at(i).pose;
      p.position.x = trajectory.points.at(i).pose.position.x +
                     ratio * (trajectory.points.at(i + 1).pose.position.x -
                              trajectory.points.at(i).pose.position.x);
      p.position.y = trajectory.points.at(i).pose.position.y +
                     ratio * (trajectory.points.at(i + 1).pose.position.y -
                              trajectory.points.at(i).pose.position.y);
      p.position.z = trajectory.points.at(i).pose.position.z +
                     ratio * (trajectory.points.at(i + 1).pose.position.z -
                              trajectory.points.at(i).pose.position.z);
      break;
    }
    remain_dist -= dist;
  }
  return p;
}

TrajectoryPoint getExtendTrajectoryPoint(
  const double extend_distance, const TrajectoryPoint & goal_point)
{
  tf2::Transform map2goal;
  tf2::fromMsg(goal_point.pose, map2goal);
  tf2::Transform local_extend_point;
  local_extend_point.setOrigin(tf2::Vector3(extend_distance, 0.0, 0.0));
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  local_extend_point.setRotation(q);
  const auto map2extend_point = map2goal * local_extend_point;
  geometry_msgs::msg::Pose extend_pose;
  tf2::toMsg(map2extend_point, extend_pose);
  TrajectoryPoint extend_trajectory_point;
  extend_trajectory_point.pose = extend_pose;
  extend_trajectory_point.longitudinal_velocity_mps = goal_point.longitudinal_velocity_mps;
  extend_trajectory_point.lateral_velocity_mps = goal_point.lateral_velocity_mps;
  extend_trajectory_point.acceleration_mps2 = goal_point.acceleration_mps2;
  return extend_trajectory_point;
}

}  // namespace longitudinal_utils
}  // namespace autoware::motion::control::pid_longitudinal_controller
