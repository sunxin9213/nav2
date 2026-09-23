/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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

#include "dwb_critics/rotate_to_goal.hpp"
#include <string>
#include <vector>
#include "nav_2d_utils/parameters.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/trajectory_utils.hpp"
#include "angles/angles.h"

PLUGINLIB_EXPORT_CLASS(dwb_critics::RotateToGoalCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

// 中文说明（文件级）：
// 旋转到目标（Rotate To Goal）critic：终点附近的朝向对齐评分。
// 三阶段逻辑：
// 1. 远离目标（不在窗口内）：不参与评分（返回 0）；
// 2. 进入位置容差窗口但仍在减速：惩罚平移速度（必须持续减速），
//    同时叠加朝向差评分；
// 3. 已基本停稳（rotating 状态）：任何平移速度直接判非法轨迹，
//    只比较各轨迹的最终朝向与目标朝向的角差，实现原地旋转对准。

// 距离平方（避免开方）
inline double hypot_sq(double dx, double dy)
{
  return dx * dx + dy * dy;
}

// 初始化：读取目标容差与减速相关参数
void RotateToGoalCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // 位置容差：机器人到目标距离小于该值即进入“窗口”
  xy_goal_tolerance_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + ".xy_goal_tolerance", 0.25);
  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
  // 平移停稳速度：速度低于该值视为已停止平移
  double stopped_xy_velocity = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + ".trans_stopped_velocity", 0.25);
  stopped_xy_velocity_sq_ = stopped_xy_velocity * stopped_xy_velocity;
  // 减速阶段的速度惩罚系数
  slowing_factor_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".slowing_factor", 5.0);
  // 朝向评价的前瞻时间（-1 表示用轨迹末点朝向）
  lookahead_time_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".lookahead_time", -1.0);
  reset();
}

// 重置状态机（窗口/旋转标志清零）
void RotateToGoalCritic::reset()
{
  in_window_ = false;
  rotating_ = false;
}

// 周期准备：更新状态机（窗口与停稳判定）并记录目标朝向
bool RotateToGoalCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D &)
{
  double dxy_sq = hypot_sq(pose.x - goal.x, pose.y - goal.y);
  // 状态机单向推进：一旦进入窗口/旋转状态就不再退出
  in_window_ = in_window_ || dxy_sq <= xy_goal_tolerance_sq_;
  current_xy_speed_sq_ = hypot_sq(vel.x, vel.y);
  rotating_ = rotating_ || (in_window_ && current_xy_speed_sq_ <= stopped_xy_velocity_sq_);
  goal_yaw_ = goal.theta;  // 目标朝向
  return true;
}

// 轨迹评分：按状态机三阶段分别处理
double RotateToGoalCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  // If we're not sufficiently close to the goal, we don't care what the twist is
  //（阶段 1：远离目标，本 critic 不参与）
  if (!in_window_) {
    return 0.0;
  } else if (!rotating_) {
    // 阶段 2：窗口内但未停稳：必须减速，否则判非法
    double speed_sq = hypot_sq(traj.velocity.x, traj.velocity.y);
    if (speed_sq >= current_xy_speed_sq_) {
      throw dwb_core::IllegalTrajectoryException(name_, "Not slowing down near goal.");
    }
    // 得分 = 速度平方 * 减速系数 + 朝向差
    return speed_sq * slowing_factor_ + scoreRotation(traj);
  }

  // If we're sufficiently close to the goal, any transforming velocity is invalid
  //（阶段 3：已停稳，任何平移速度都是非法轨迹，只允许原地旋转）
  if (fabs(traj.velocity.x) > 0 || fabs(traj.velocity.y) > 0) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Nonrotation command near goal.");
  }

  return scoreRotation(traj);
}

// 朝向评分：轨迹（末点或前瞻时刻）朝向与目标朝向的最短角差
double RotateToGoalCritic::scoreRotation(const dwb_msgs::msg::Trajectory2D & traj)
{
  if (traj.poses.empty()) {
    throw dwb_core::IllegalTrajectoryException(name_, "Empty trajectory.");
  }

  double end_yaw;
  if (lookahead_time_ >= 0.0) {
    // 正向前瞻：取轨迹上 lookahead_time 时刻的投影位姿朝向
    geometry_msgs::msg::Pose2D eval_pose = dwb_core::projectPose(traj, lookahead_time_);
    end_yaw = eval_pose.theta;
  } else {
    end_yaw = traj.poses.back().theta;  // 默认取轨迹末端朝向
  }
  return fabs(angles::shortest_angular_distance(end_yaw, goal_yaw_));
}

}  // namespace dwb_critics
