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
 * 文件：simple_goal_checker.cpp（nav2_controller 模块）
 * =============================================================================
 * 功能概述：
 *   实现 SimpleGoalChecker——最常用的"简单目标检查器"插件
 *   （nav2_core::GoalChecker 接口实现，经 PLUGINLIB_EXPORT_CLASS 导出）。
 *   在控制器服务器（ControllerServer）的局部控制循环中，
 *   isGoalReached() 每个控制周期被调用一次，用于判断机器人是否
 *   已到达本次 FollowPath 任务的终点：
 *     - 位置容差：与目标的 XY 距离 <= xy_goal_tolerance（默认 0.25m）；
 *     - 朝向容差：与目标 yaw 的角度差 <= yaw_goal_tolerance（默认 0.25rad）；
 *     - stateful 模式：位置一旦达标即"锁存"，之后只需满足朝向条件
 *       （防止机器人在容差圆边缘抖动导致判断反复）；
 *     - symmetric_yaw_tolerance：对称机器人允许终点朝向 ±180° 等效。
 *   参数在 initialize() 中声明并注册动态参数回调，可运行期在线修改。
 *
 * 在整个导航流程中的位置：
 *   bt_navigator → planner_server → controller_server【本插件在此被调用】
 *   → velocity_smoother。
 * =============================================================================
 */

#include <memory>
#include <string>
#include <limits>
#include <vector>
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

// 构造函数：设置默认容差（位置 0.25m、朝向 0.25rad），默认开启有状态检查；
// xy_goal_tolerance_sq_ 为位置容差的平方，用于避免每次比较时开方运算。
SimpleGoalChecker::SimpleGoalChecker()
: xy_goal_tolerance_(0.25),
  yaw_goal_tolerance_(0.25),
  stateful_(true),
  check_xy_(true),
  symmetric_yaw_tolerance_(false),
  xy_goal_tolerance_sq_(0.0625)
{
}

// 插件初始化（由 ControllerServer 在 on_configure 中调用）：
// 1) 以 "插件名.参数名" 的形式在宿主节点上声明本插件的参数
//    （declare_parameter_if_not_declared 保证不与已有声明冲突）；
// 2) 读取 xy/yaw 容差、stateful、对称朝向容差等配置；
// 3) 预计算位置容差平方；
// 4) 注册动态参数回调，支持运行期在线修改上述参数。
void SimpleGoalChecker::initialize(
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
    plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".stateful", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".symmetric_yaw_tolerance", rclcpp::ParameterValue(false));

  node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".stateful", stateful_);
  node->get_parameter(plugin_name + ".symmetric_yaw_tolerance", symmetric_yaw_tolerance_);

  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SimpleGoalChecker::dynamicParametersCallback, this, _1));
}

// 复位状态：将 check_xy_ 恢复为 true，重新开始位置检查。
// 每次新的 FollowPath 任务开始时由 ControllerServer 调用，
// 保证上一个任务中"位置已达标"的锁存状态不会影响新任务。
void SimpleGoalChecker::reset()
{
  check_xy_ = true;
}

// 核心判定：机器人当前位姿（query_pose）是否到达目标（goal_pose）。
// 判定分两步：
// 1) 位置检查（check_xy_ 为 true 时）：XY 距离平方 > 容差平方则未到达；
//    若在容差内且 stateful_，置 check_xy_=false 锁存"位置已达标"状态；
// 2) 朝向检查：计算 query 与 goal 的 yaw 差（取最短角差）：
//    - 普通机器人：|dyaw| <= yaw_goal_tolerance_ 即达标；
//    - 对称机器人（symmetric_yaw_tolerance_）：额外允许目标朝向 +180°
//      的等效匹配（差速/圆形底盘倒着到位也算到达）。
// 速度参数（第三个形参）在本插件中不使用。
bool SimpleGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
  if (check_xy_) {
    double dx = query_pose.position.x - goal_pose.position.x,
      dy = query_pose.position.y - goal_pose.position.y;
    if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
      return false;
    }
    // We are within the window
    // If we are stateful, change the state.
    if (stateful_) {
      check_xy_ = false;
    }
  }

  double query_yaw = tf2::getYaw(query_pose.orientation);
  double goal_yaw = tf2::getYaw(goal_pose.orientation);
  if (symmetric_yaw_tolerance_) {
    // For symmetric robots: accept either goal orientation or goal + 180°
    double dyaw_forward = angles::shortest_angular_distance(query_yaw, goal_yaw);
    double dyaw_backward = angles::shortest_angular_distance(
      query_yaw, angles::normalize_angle(goal_yaw + M_PI));

    bool forward_match = fabs(dyaw_forward) <= yaw_goal_tolerance_;
    bool backward_match = fabs(dyaw_backward) <= yaw_goal_tolerance_;

    return forward_match || backward_match;
  } else {
    double dyaw = angles::shortest_angular_distance(query_yaw, goal_yaw);
    return fabs(dyaw) <= yaw_goal_tolerance_;
  }
}

// 向调用方（控制器服务器/控制器插件）暴露本检查器的容差：
// 位置容差填入 x/y（z 置为无效值），朝向容差转为绕 Z 轴的四元数；
// 速度容差本插件不使用，全部置为 invalid（lowest double），
// 以通知调用方"该检查器不限制速度"。
bool SimpleGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}

// 动态参数回调：运行期修改 xy/yaw 容差、stateful、对称朝向等参数时触发，
// 按参数类型（DOUBLE/BOOL）分发更新对应成员；注意位置容差更新时
// 同步重算其平方值。始终返回 successful=true（接受一切合法修改）。
rcl_interfaces::msg::SetParametersResult
SimpleGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".xy_goal_tolerance") {
        xy_goal_tolerance_ = parameter.as_double();
        xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
      } else if (name == plugin_name_ + ".yaw_goal_tolerance") {
        yaw_goal_tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".symmetric_yaw_tolerance") {
        symmetric_yaw_tolerance_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

// pluginlib 导出宏：将本类注册为 nav2_core::GoalChecker 基接口的实现，
// 使 ControllerServer 能通过 pluginlib::ClassLoader 按名称
// （nav2_controller::SimpleGoalChecker）动态加载本插件。
PLUGINLIB_EXPORT_CLASS(nav2_controller::SimpleGoalChecker, nav2_core::GoalChecker)
