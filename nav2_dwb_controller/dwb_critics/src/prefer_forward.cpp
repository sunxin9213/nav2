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

#include "dwb_critics/prefer_forward.hpp"
#include <math.h>
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::PreferForwardCritic, dwb_core::TrajectoryCritic)

using nav2_util::declare_parameter_if_not_declared;

namespace dwb_critics
{

// 中文说明（文件级）：
// 前进偏好（Prefer Forward）critic：惩罚倒车与横漂运动，
// 并惩罚大角速度（旋转越多前进越少）。
// 适用于无后向传感器、不宜倒车的机器人。

// 初始化：读取罚分与横漂/旋转阈值参数
void PreferForwardCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // penalty：倒车/横漂时的固定罚分
  declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".penalty", rclcpp::ParameterValue(1.0));
  // strafe_x / strafe_theta：判定“横漂”的线速度/角速度阈值
  declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".strafe_x", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".strafe_theta",
    rclcpp::ParameterValue(0.2));
  // theta_scale：角速度惩罚的缩放系数
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".theta_scale",
    rclcpp::ParameterValue(10.0));

  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".penalty", penalty_);
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".strafe_x", strafe_x_);
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".strafe_theta", strafe_theta_);
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".theta_scale", theta_scale_);
}

// 轨迹评分：倒车/横漂记罚分，否则按角速度惩罚
double PreferForwardCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  // backward motions bad on a robot without backward sensors
  //（倒车：直接返回罚分）
  if (traj.velocity.x < 0.0) {
    return penalty_;
  }
  // strafing motions also bad on such a robot
  //（低速前进 + 低速旋转 ≈ 原地横漂：同样返回罚分）
  if (traj.velocity.x < strafe_x_ && fabs(traj.velocity.theta) < strafe_theta_) {
    return penalty_;
  }

  // the more we rotate, the less we progress forward
  //（正常前进：按角速度绝对值 * 缩放系数惩罚大转弯）
  return fabs(traj.velocity.theta) * theta_scale_;
}

}  // namespace dwb_critics
