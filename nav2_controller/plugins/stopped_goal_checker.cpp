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

/*
 * =============================================================================
 * 文件：stopped_goal_checker.cpp（nav2_controller 模块）
 * =============================================================================
 * 功能概述：
 *   实现 StoppedGoalChecker——"停止型目标检查器"插件
 *   （nav2_core::GoalChecker 接口实现，继承自 SimpleGoalChecker）。
 *   在 SimpleGoalChecker 的位置+朝向判定基础上，额外要求机器人
 *   已经"停稳"：
 *     - 角速度 |ω| <= rot_stopped_velocity（默认 0.25 rad/s）；
 *     - 平移速度 hypot(vx,vy) <= trans_stopped_velocity（默认 0.25 m/s）。
 *   适用于要求任务结束时机器人完全静止的场景（如抓取、对接），
 *   避免机器人在运动惯性中被判定为"到达"。
 *
 * 在整个导航流程中的位置：
 *   bt_navigator → planner_server → controller_server【本插件在此被调用】
 *   → velocity_smoother。
 * =============================================================================
 */

#include <cmath>
#include <string>
#include <memory>
#include <limits>
#include <vector>
#include "nav2_controller/plugins/stopped_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

using std::hypot;
using std::fabs;

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

// 构造函数：先构造基类 SimpleGoalChecker（位置/朝向容差），
// 再初始化本插件特有的转动/平移"停止速度"阈值（默认均 0.25）。
StoppedGoalChecker::StoppedGoalChecker()
: SimpleGoalChecker(), rot_stopped_velocity_(0.25), trans_stopped_velocity_(0.25)
{
}

// 插件初始化：先调用基类 initialize（位置/朝向容差与动态参数回调），
// 再声明并读取本插件特有的 rot_stopped_velocity / trans_stopped_velocity
// 参数，最后注册本类的动态参数回调（会替换基类注册的回调）。
void StoppedGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  plugin_name_ = plugin_name;
  SimpleGoalChecker::initialize(parent, plugin_name, costmap_ros);

  auto node = parent.lock();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".rot_stopped_velocity", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".trans_stopped_velocity", rclcpp::ParameterValue(0.25));

  node->get_parameter(plugin_name + ".rot_stopped_velocity", rot_stopped_velocity_);
  node->get_parameter(plugin_name + ".trans_stopped_velocity", trans_stopped_velocity_);

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&StoppedGoalChecker::dynamicParametersCallback, this, _1));
}

// 核心判定："到达" = 位置与朝向均达标（基类判定）
//           且 机器人已停稳：
//           角速度 <= rot_stopped_velocity_ 且
//           平移速度模长 <= trans_stopped_velocity_。
// velocity 来自里程计反馈，反映机器人当前实际运动状态。
bool StoppedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  bool ret = SimpleGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
  if (!ret) {
    return ret;
  }

  return fabs(velocity.angular.z) <= rot_stopped_velocity_ &&
         hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_;
}

// 暴露容差：位置/朝向部分沿用基类结果，
// 速度部分覆写为两个"停止速度"阈值（linear.x/y 与 angular.z），
// 通知调用方本检查器对速度的判定条件。
bool StoppedGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  // populate the poses
  bool rtn = SimpleGoalChecker::getTolerances(pose_tolerance, vel_tolerance);

  // override the velocities
  vel_tolerance.linear.x = trans_stopped_velocity_;
  vel_tolerance.linear.y = trans_stopped_velocity_;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = rot_stopped_velocity_;

  return true && rtn;
}

// 动态参数回调：在线修改 rot/trans 停止速度阈值。
rcl_interfaces::msg::SetParametersResult
StoppedGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".rot_stopped_velocity") {
        rot_stopped_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".trans_stopped_velocity") {
        trans_stopped_velocity_ = parameter.as_double();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

// pluginlib 导出宏：注册为 nav2_core::GoalChecker 插件，
// 供 ControllerServer 的 goal_checker 插件加载器按名称动态加载。
PLUGINLIB_EXPORT_CLASS(nav2_controller::StoppedGoalChecker, nav2_core::GoalChecker)
