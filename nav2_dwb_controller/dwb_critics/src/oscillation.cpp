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

#include "dwb_critics/oscillation.hpp"
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include "nav_2d_utils/parameters.hpp"
#include "nav2_util/node_utils.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::OscillationCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

// 中文说明（文件级）：
// 振荡（Oscillation）critic：抑制“前后/左右/正反旋转”的往复振荡行为。
// 原理（源自 DWA 的经典振荡抑制）：
// 1. CommandTrend 跟踪每个速度分量的符号变化趋势：一旦某分量出现过
//    正负翻转（即已向该方向移动过），即禁止再向反方向移动的轨迹；
// 2. 机器人移动足够距离/角度或超时后，振荡标志自动复位，
//    允许再次反向（例如绕障时需要左右交替）。
// 评分恒为 0：违反振荡禁令的轨迹直接抛 IllegalTrajectoryException 剔除。

// 命令趋势跟踪器构造：复位状态
OscillationCritic::CommandTrend::CommandTrend()
{
  reset();
}

// 复位趋势状态：无符号记录、无单边禁令
void OscillationCritic::CommandTrend::reset()
{
  sign_ = Sign::ZERO;
  positive_only_ = false;
  negative_only_ = false;
}

// 用最新速度更新趋势：检测符号翻转并返回是否发生了翻转
bool OscillationCritic::CommandTrend::update(double velocity)
{
  bool flag_set = false;
  if (velocity < 0.0) {
    if (sign_ == Sign::POSITIVE) {
      // 之前为正、现在为负：已发生正→负翻转，此后禁止再正向
      negative_only_ = true;
      flag_set = true;
    }
    sign_ = Sign::NEGATIVE;
  } else if (velocity > 0.0) {
    if (sign_ == Sign::NEGATIVE) {
      // 之前为负、现在为正：此后禁止再负向
      positive_only_ = true;
      flag_set = true;
    }
    sign_ = Sign::POSITIVE;
  }
  return flag_set;
}

// 判断该速度是否违反振荡禁令（禁止回到已翻转离开的方向）
bool OscillationCritic::CommandTrend::isOscillating(double velocity)
{
  return (positive_only_ && velocity < 0.0) || (negative_only_ && velocity > 0.0);
}

// 是否发生过符号翻转（即是否处于受限状态）
bool OscillationCritic::CommandTrend::hasSignFlipped()
{
  return positive_only_ || negative_only_;
}

// 初始化：读取复位阈值参数
void OscillationCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  clock_ = node->get_clock();

  // 距离复位阈值：机器人移动超过该距离即解除振荡禁令
  oscillation_reset_dist_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".oscillation_reset_dist", 0.05);
  oscillation_reset_dist_sq_ = oscillation_reset_dist_ * oscillation_reset_dist_;
  // 角度复位阈值：旋转超过该角度即解除禁令
  oscillation_reset_angle_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".oscillation_reset_angle", 0.2);
  // 时间复位阈值：超过该时长即解除禁令（-1 禁用）
  oscillation_reset_time_ = rclcpp::Duration::from_seconds(
    nav_2d_utils::searchAndGetParam(
      node,
      dwb_plugin_name_ + "." + name_ + ".oscillation_reset_time", -1.0));

  // x_only_threshold：前进速度低于该值时才跟踪 y/theta 的振荡
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".x_only_threshold", rclcpp::ParameterValue(0.05));

  /**
   * Historical Parameter Loading
   * If x_only_threshold is set, use that.
   * If min_speed_xy is set in the namespace (as it is often used for trajectory generation), use that.
   * If min_trans_vel is set in the namespace, as it used to be used for trajectory generation, complain then use that.
   * Otherwise, set x_only_threshold_ to 0.05
   */
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".x_only_threshold", x_only_threshold_);
  // TODO(crdelsey): How to handle searchParam?
  // std::string resolved_name;
  // if (node->hasParam("x_only_threshold"))
  // {
  //   node->param("x_only_threshold", x_only_threshold_);
  // }
  // else if (node->searchParam("min_speed_xy", resolved_name))
  // {
  //   node->param(resolved_name, x_only_threshold_);
  // }
  // else if (node->searchParam("min_trans_vel", resolved_name))
  // {
  //   ROS_WARN_NAMED("OscillationCritic",
  //     "Parameter min_trans_vel is deprecated. "
  //     "Please use the name min_speed_xy or x_only_threshold instead.");
  //   node->param(resolved_name, x_only_threshold_);
  // }
  // else
  // {
  //   x_only_threshold_ = 0.05;
  // }

  reset();
}

// 周期准备：仅记录当前位姿（供 debrief 计算位移）
bool OscillationCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D &)
{
  pose_ = pose;
  return true;
}

// 事后复盘（每周期执行完指令后调用）：更新振荡趋势并判断是否复位
void OscillationCritic::debrief(const nav_2d_msgs::msg::Twist2D & cmd_vel)
{
  if (setOscillationFlags(cmd_vel)) {
    // 发生新的符号翻转：记录参考位姿与时间（复位判定的基准）
    prev_stationary_pose_ = pose_;
    prev_reset_time_ = clock_->now();
  }

  // if we've got restrictions... check if we can reset any oscillation flags
  //（任一分量处于受限状态时，检查是否满足复位条件以解除禁令）
  if (x_trend_.hasSignFlipped() || y_trend_.hasSignFlipped() || theta_trend_.hasSignFlipped()) {
    // Reset flags if enough time or distance has passed
    if (resetAvailable()) {
      reset();
    }
  }
}

// 判断是否满足任一复位条件（距离/角度/时间）
bool OscillationCritic::resetAvailable()
{
  // 条件 1：相对参考位姿的平移距离超过阈值
  if (oscillation_reset_dist_ >= 0.0) {
    double x_diff = pose_.x - prev_stationary_pose_.x;
    double y_diff = pose_.y - prev_stationary_pose_.y;
    double sq_dist = x_diff * x_diff + y_diff * y_diff;
    if (sq_dist > oscillation_reset_dist_sq_) {
      return true;
    }
  }
  // 条件 2：相对参考位姿的旋转角度超过阈值
  if (oscillation_reset_angle_ >= 0.0) {
    double th_diff = pose_.theta - prev_stationary_pose_.theta;
    if (fabs(th_diff) > oscillation_reset_angle_) {
      return true;
    }
  }
  // 条件 3：距上次翻转的时间超过阈值
  if (oscillation_reset_time_ >= rclcpp::Duration::from_seconds(0.0)) {
    auto t_diff = (clock_->now() - prev_reset_time_);
    if (t_diff > oscillation_reset_time_) {
      return true;
    }
  }
  return false;
}

// 复位全部三个分量的趋势跟踪器
void OscillationCritic::reset()
{
  x_trend_.reset();
  y_trend_.reset();
  theta_trend_.reset();
}

// 用本周期实际下发的速度更新各分量的振荡趋势
bool OscillationCritic::setOscillationFlags(const nav_2d_msgs::msg::Twist2D & cmd_vel)
{
  bool flag_set = false;
  // set oscillation flags for moving forward and backward
  //（x 方向：前进/后退的往复）
  flag_set |= x_trend_.update(cmd_vel.x);

  // we'll only set flags for strafing and rotating when we're not moving forward at all
  //（几乎无前进速度时才跟踪 y 横漂与 theta 正反旋转的振荡）
  if (x_only_threshold_ < 0.0 || fabs(cmd_vel.x) <= x_only_threshold_) {
    flag_set |= y_trend_.update(cmd_vel.y);
    flag_set |= theta_trend_.update(cmd_vel.theta);
  }
  return flag_set;
}

// 轨迹评分：恒为 0；违反振荡禁令的轨迹抛异常剔除
double OscillationCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  if (x_trend_.isOscillating(traj.velocity.x) ||
    y_trend_.isOscillating(traj.velocity.y) ||
    theta_trend_.isOscillating(traj.velocity.theta))
  {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory is oscillating.");
  }
  return 0.0;
}

}  // namespace dwb_critics
