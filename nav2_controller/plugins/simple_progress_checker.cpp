// Copyright (c) 2019 Intel Corporation
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
 * 文件：simple_progress_checker.cpp（nav2_controller 模块）
 * =============================================================================
 * 功能概述：
 *   实现 SimpleProgressChecker——最简单的"进度检查器"插件
 *   （nav2_core::ProgressChecker 接口实现）。
 *   在 ControllerServer 的控制循环中每个周期调用 check()，
 *   判断机器人是否仍在"有效推进"：
 *     - 机器人必须在 movement_time_allowance（默认 10s）内
 *       移动超过 required_movement_radius（默认 0.5m）；
 *     - 一旦移出基准半径，基准位姿/时间即被重置，重新计时；
 *     - 超时仍未移动足够的距离 → check() 返回 false，
 *       控制器据此判定"卡死/无进展"，结束 FollowPath 并报错。
 *   典型用途：检测机器人被障碍物卡住、电机堵转等异常。
 *
 * 在整个导航流程中的位置：
 *   bt_navigator → planner_server → controller_server【本插件在此被调用】
 *   → velocity_smoother。
 * =============================================================================
 */

#include "nav2_controller/plugins/simple_progress_checker.hpp"
#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include "nav2_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{
// 插件初始化（ControllerServer on_configure 时调用）：
// 声明并读取 required_movement_radius / movement_time_allowance 参数，
// 获取节点时钟（用于超时判断），注册动态参数回调。
void SimpleProgressChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".required_movement_radius", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".movement_time_allowance", rclcpp::ParameterValue(10.0));
  // Scale is set to 0 by default, so if it was not set otherwise, set to 0
  node->get_parameter_or(plugin_name + ".required_movement_radius", radius_, 0.5);
  double time_allowance_param = 0.0;
  node->get_parameter_or(plugin_name + ".movement_time_allowance", time_allowance_param, 10.0);
  time_allowance_ = rclcpp::Duration::from_seconds(time_allowance_param);

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SimpleProgressChecker::dynamicParametersCallback, this, _1));
}

// 核心判定：机器人是否仍在有效推进。
// 逻辑（利用短路求值）：
// 1) 基准位姿尚未设置，或机器人已移出基准半径 → 重置基准并返回 true；
// 2) 否则检查距上次基准的时间：超过 time_allowance_ 仍没有足够移动
//    → 返回 false（无进展，控制器将终止任务并报告异常）。
bool SimpleProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  // relies on short circuit evaluation to not call is_robot_moved_enough if
  // baseline_pose is not set.
  geometry_msgs::msg::Pose2D current_pose2d;
  current_pose2d = nav_2d_utils::poseToPose2D(current_pose.pose);

  if ((!baseline_pose_set_) || (isRobotMovedEnough(current_pose2d))) {
    resetBaselinePose(current_pose2d);
    return true;
  }
  return !((clock_->now() - baseline_time_) > time_allowance_);
}

// 复位状态：清除基准位姿标志，新任务开始时由控制器服务器调用，
// 下一次 check() 将重新建立基准。
void SimpleProgressChecker::reset()
{
  baseline_pose_set_ = false;
}

// 更新基准：记录当前位姿与时间戳，标记基准已设置。
// 机器人每移出一次半径就重置一次基准，相当于滚动窗口式监控。
void SimpleProgressChecker::resetBaselinePose(const geometry_msgs::msg::Pose2D & pose)
{
  baseline_pose_ = pose;
  baseline_time_ = clock_->now();
  baseline_pose_set_ = true;
}

// 判断机器人相对基准位姿是否移动了足够距离（> 半径阈值）。
bool SimpleProgressChecker::isRobotMovedEnough(const geometry_msgs::msg::Pose2D & pose)
{
  return pose_distance(pose, baseline_pose_) > radius_;
}

double SimpleProgressChecker::pose_distance(
  const geometry_msgs::msg::Pose2D & pose1,
  const geometry_msgs::msg::Pose2D & pose2)
{
  double dx = pose1.x - pose2.x;
  double dy = pose1.y - pose2.y;

  return std::hypot(dx, dy);
}

rcl_interfaces::msg::SetParametersResult
SimpleProgressChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".required_movement_radius") {
        radius_ = parameter.as_double();
      } else if (name == plugin_name_ + ".movement_time_allowance") {
        time_allowance_ = rclcpp::Duration::from_seconds(parameter.as_double());
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::SimpleProgressChecker, nav2_core::ProgressChecker)
