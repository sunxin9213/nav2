// Copyright (c) 2025 Prabhav Saxena
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

/*
 * =============================================================================
 * 文件：position_goal_checker.cpp（nav2_controller 模块）
 * =============================================================================
 * 功能概述：
 *   实现 PositionGoalChecker——"仅位置"目标检查器插件
 *   （nav2_core::GoalChecker 接口实现）。
 *   与 SimpleGoalChecker 的区别：只判定 XY 位置容差，完全忽略朝向，
 *   适用于圆形/全向机器人或"到达即可，不关心朝向"的任务。
 *   同样支持 stateful 模式：位置一旦达标即锁存，之后立即返回已到达。
 *
 * 在整个导航流程中的位置：
 *   bt_navigator → planner_server → controller_server【本插件在此被调用】
 *   → velocity_smoother。
 * =============================================================================
 */

#include <memory>
#include <string>
#include <limits>
#include "nav2_controller/plugins/position_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

// 构造函数：默认位置容差 0.25m（平方 0.0625），默认开启有状态检查。
PositionGoalChecker::PositionGoalChecker()
: xy_goal_tolerance_(0.25),
  xy_goal_tolerance_sq_(0.0625),
  stateful_(true),
  position_reached_(false)
{
}

// 插件初始化：声明并读取 xy_goal_tolerance 与 stateful 参数，
// 注册动态参数回调（可运行期修改）。
void PositionGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".stateful", rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name + ".stateful", stateful_);

  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PositionGoalChecker::dynamicParametersCallback, this, _1));
}

// 复位状态：清除"位置已达标"锁存，供新任务开始时调用。
void PositionGoalChecker::reset()
{
  position_reached_ = false;
}

// 核心判定：只比较 XY 位置（距离平方 <= 容差平方），
// 不检查朝向与速度（对应形参留空不使用）。
// stateful 模式下位置一旦达标即置 position_reached_ 锁存，
// 后续周期直接返回 true，避免容差圆边缘抖动导致反复判断。
bool PositionGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
  // If stateful and position was already reached, maintain state
  if (stateful_ && position_reached_) {
    return true;
  }

  // Check if position is within tolerance
  double dx = query_pose.position.x - goal_pose.position.x;
  double dy = query_pose.position.y - goal_pose.position.y;

  bool position_reached = (dx * dx + dy * dy <= xy_goal_tolerance_sq_);

  // If stateful, remember that we reached the position
  if (stateful_ && position_reached) {
    position_reached_ = true;
  }

  return position_reached;
}

// 暴露容差：位置容差仅填 x/y；朝向容差返回单位四元数（0 容差，
// 表示本检查器不做朝向判定）；速度容差全部置为无效值（不限制速度）。
bool PositionGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;

  // Return zero orientation tolerance as we don't check it
  pose_tolerance.orientation.x = 0.0;
  pose_tolerance.orientation.y = 0.0;
  pose_tolerance.orientation.z = 0.0;
  pose_tolerance.orientation.w = 1.0;

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}

// 编程接口：由代码（而非参数文件）直接设置位置容差，
// 供控制器插件等在特殊场景下动态调整；同步更新平方值。
void nav2_controller::PositionGoalChecker::setXYGoalTolerance(double tolerance)
{
  xy_goal_tolerance_ = tolerance;
  xy_goal_tolerance_sq_ = tolerance * tolerance;
}

// 动态参数回调：在线修改位置容差与 stateful 开关。
rcl_interfaces::msg::SetParametersResult
PositionGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".xy_goal_tolerance") {
        xy_goal_tolerance_ = parameter.as_double();
        xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

// pluginlib 导出宏：注册为 nav2_core::GoalChecker 插件，
// 供 ControllerServer 按名称动态加载。
PLUGINLIB_EXPORT_CLASS(nav2_controller::PositionGoalChecker, nav2_core::GoalChecker)
