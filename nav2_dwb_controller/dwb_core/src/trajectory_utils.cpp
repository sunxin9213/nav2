/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <dwb_core/trajectory_utils.hpp>

#include <cmath>

#include "rclcpp/duration.hpp"

#include "dwb_core/exceptions.hpp"

namespace dwb_core
{
// 返回轨迹中时间戳最接近 time_offset 的位姿（不插值，取离散最近点）
const geometry_msgs::msg::Pose2D & getClosestPose(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double time_offset)
{
  rclcpp::Duration goal_time = rclcpp::Duration::from_seconds(time_offset);  // 目标时刻
  const unsigned int num_poses = trajectory.poses.size();
  if (num_poses == 0) {
    throw nav2_core::PlannerException("Cannot call getClosestPose on empty trajectory.");
  }
  unsigned int closest_index = num_poses;  // 尚未找到有效候选的哨兵值
  double closest_diff = 0.0;
  // 线性扫描：找时间差最小的轨迹点。
  // 一旦越过目标时刻（时间戳单调递增），后续只会更差，可提前退出
  for (unsigned int i = 0; i < num_poses; ++i) {
    double diff = std::fabs((rclcpp::Duration(trajectory.time_offsets[i]) - goal_time).seconds());
    if (closest_index == num_poses || diff < closest_diff) {
      closest_index = i;
      closest_diff = diff;
    }
    if (goal_time < rclcpp::Duration(trajectory.time_offsets[i])) {
      break;  // 已越过目标时刻，提前结束
    }
  }
  return trajectory.poses[closest_index];
}

// 将目标时刻线性插值到轨迹上，得到精确的预测位姿
geometry_msgs::msg::Pose2D projectPose(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double time_offset)
{
  rclcpp::Duration goal_time = rclcpp::Duration::from_seconds(time_offset);
  const unsigned int num_poses = trajectory.poses.size();
  if (num_poses == 0) {
    throw nav2_core::PlannerException("Cannot call projectPose on empty trajectory.");
  }
  // 边界情况：早于轨迹起点/晚于轨迹终点，直接返回端点
  if (goal_time <= (trajectory.time_offsets[0])) {
    return trajectory.poses[0];
  } else if (goal_time >= rclcpp::Duration(trajectory.time_offsets[num_poses - 1])) {
    return trajectory.poses[num_poses - 1];
  }

  // 找到目标时刻所在的相邻两点区间 [t_i, t_{i+1})
  for (unsigned int i = 0; i < num_poses - 1; ++i) {
    if (goal_time >= rclcpp::Duration(trajectory.time_offsets[i]) &&
      goal_time < rclcpp::Duration(trajectory.time_offsets[i + 1]))
    {
      // 区间时长与目标在区间内的归一化比例 [0,1]
      double time_diff =
        (rclcpp::Duration(trajectory.time_offsets[i + 1]) -
        rclcpp::Duration(trajectory.time_offsets[i])).seconds();
      double ratio = (goal_time - rclcpp::Duration(trajectory.time_offsets[i])).seconds() /
        time_diff;
      double inv_ratio = 1.0 - ratio;
      // 对 x/y/theta 做线性插值
      const geometry_msgs::msg::Pose2D & pose_a = trajectory.poses[i];
      const geometry_msgs::msg::Pose2D & pose_b = trajectory.poses[i + 1];
      geometry_msgs::msg::Pose2D projected;
      projected.x = pose_a.x * inv_ratio + pose_b.x * ratio;
      projected.y = pose_a.y * inv_ratio + pose_b.y * ratio;
      projected.theta = pose_a.theta * inv_ratio + pose_b.theta * ratio;
      return projected;
    }
  }

  // Should not reach this point
  return trajectory.poses[num_poses - 1];  // 兜底（理论不可达）
}


}  // namespace dwb_core
