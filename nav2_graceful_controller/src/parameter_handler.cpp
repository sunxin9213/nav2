// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "nav2_graceful_controller/parameter_handler.hpp"

namespace nav2_graceful_controller
{

using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

// 中文说明（文件级）：
// 参数处理器：声明/读取 Graceful Controller 的全部参数，
// 并注册 ROS2 动态参数回调（运行时修改参数即时生效，互斥锁保护）。
// 参数分为四组：路径变换、控制律增益（k_phi/k_delta/beta/lambda）、
// 速度限制、行为开关（起始/终点旋转、倒车）。

// 构造：声明并读取全部参数，注册动态参数回调
ParameterHandler::ParameterHandler(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string & plugin_name,
  rclcpp::Logger & logger, const double costmap_size_x)
{
  plugin_name_ = plugin_name;
  logger_ = logger;

  // ---- 参数声明（带默认值）----
  // TF 变换容差
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  // 运动目标点（前探点）距离
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".motion_target_dist", rclcpp::ParameterValue(0.6));
  // 在路径上搜索机器人最近点的最大距离（默认为代价地图半宽）
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist",
    rclcpp::ParameterValue(costmap_size_x / 2.0));
  // 控制律增益：k_phi（φ 反馈）、k_delta（δ 比例）、
  // beta/lambda（速度-曲率衰减系数与指数）
  declare_parameter_if_not_declared(node, plugin_name_ + ".k_phi", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".k_delta", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".beta", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lambda", rclcpp::ParameterValue(2.0));
  // 速度限制：最小/最大线速度、最大角速度
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_min", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_max", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_max", rclcpp::ParameterValue(1.0));
  // 接近目标的减速半径
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".slowdown_radius", rclcpp::ParameterValue(1.5));
  // 起始/终点是否原地旋转，及起始旋转的角度阈值与旋转缩放系数
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".initial_rotation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".initial_rotation_min_angle", rclcpp::ParameterValue(0.75));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".final_rotation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_scaling_factor", rclcpp::ParameterValue(0.5));
  // 是否允许倒车跟踪
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".allow_backward", rclcpp::ParameterValue(false));

  // ---- 参数读取 ----
  node->get_parameter(plugin_name_ + ".transform_tolerance", params_.transform_tolerance);
  node->get_parameter(plugin_name_ + ".motion_target_dist", params_.motion_target_dist);
  node->get_parameter(
    plugin_name_ + ".max_robot_pose_search_dist", params_.max_robot_pose_search_dist);
  // 搜索距离为负：视为搜索整条路径
  if (params_.max_robot_pose_search_dist < 0.0) {
    RCLCPP_WARN(
      logger_, "Max robot search distance is negative, setting to max to search"
      " every point on path for the closest value.");
    params_.max_robot_pose_search_dist = std::numeric_limits<double>::max();
  }

  node->get_parameter(plugin_name_ + ".k_phi", params_.k_phi);
  node->get_parameter(plugin_name_ + ".k_delta", params_.k_delta);
  node->get_parameter(plugin_name_ + ".beta", params_.beta);
  node->get_parameter(plugin_name_ + ".lambda", params_.lambda);
  node->get_parameter(plugin_name_ + ".v_linear_min", params_.v_linear_min);
  // 记录初始最大速度（速度限制恢复时使用）
  node->get_parameter(plugin_name_ + ".v_linear_max", params_.v_linear_max);
  params_.v_linear_max_initial = params_.v_linear_max;
  node->get_parameter(plugin_name_ + ".v_angular_max", params_.v_angular_max);
  params_.v_angular_max_initial = params_.v_angular_max;
  node->get_parameter(plugin_name_ + ".slowdown_radius", params_.slowdown_radius);
  node->get_parameter(plugin_name_ + ".initial_rotation", params_.initial_rotation);
  node->get_parameter(
    plugin_name_ + ".initial_rotation_min_angle", params_.initial_rotation_min_angle);
  node->get_parameter(plugin_name_ + ".final_rotation", params_.final_rotation);
  node->get_parameter(plugin_name_ + ".rotation_scaling_factor", params_.rotation_scaling_factor);
  node->get_parameter(plugin_name_ + ".allow_backward", params_.allow_backward);

  // 互斥约束：起始旋转与倒车不能同时启用（语义冲突）
  if (params_.initial_rotation && params_.allow_backward) {
    RCLCPP_WARN(
      logger_, "Initial rotation and allow backward parameters are both true, "
      "setting allow backward to false.");
    params_.allow_backward = false;
  }

  // 注册 ROS2 动态参数变化回调（运行时调参即时生效）
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ParameterHandler::dynamicParametersCallback, this, std::placeholders::_1));
}

// 动态参数回调：按名称逐个更新参数值
rcl_interfaces::msg::SetParametersResult
ParameterHandler::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);  // 与计算线程互斥

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // 浮点参数：路径/控制律/速度/减速/旋转相关
    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".transform_tolerance") {
        params_.transform_tolerance = parameter.as_double();
      } else if (name == plugin_name_ + ".motion_target_dist") {
        params_.motion_target_dist = parameter.as_double();
      } else if (name == plugin_name_ + ".k_phi") {
        params_.k_phi = parameter.as_double();
      } else if (name == plugin_name_ + ".k_delta") {
        params_.k_delta = parameter.as_double();
      } else if (name == plugin_name_ + ".beta") {
        params_.beta = parameter.as_double();
      } else if (name == plugin_name_ + ".lambda") {
        params_.lambda = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_min") {
        params_.v_linear_min = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_max") {
        params_.v_linear_max = parameter.as_double();
        params_.v_linear_max_initial = params_.v_linear_max;  // 同步更新初始值
      } else if (name == plugin_name_ + ".v_angular_max") {
        params_.v_angular_max = parameter.as_double();
        params_.v_angular_max_initial = params_.v_angular_max;
      } else if (name == plugin_name_ + ".slowdown_radius") {
        params_.slowdown_radius = parameter.as_double();
      } else if (name == plugin_name_ + ".initial_rotation_min_angle") {
        params_.initial_rotation_min_angle = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_scaling_factor") {
        params_.rotation_scaling_factor = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      // 布尔参数：行为开关（保持互斥约束）
      if (name == plugin_name_ + ".initial_rotation") {
        if (parameter.as_bool() && params_.allow_backward) {
          // 与倒车冲突：拒绝本次修改
          RCLCPP_WARN(
            logger_, "Initial rotation and allow backward parameters are both true, "
            "rejecting parameter change.");
          continue;
        }
        params_.initial_rotation = parameter.as_bool();
      } else if (name == plugin_name_ + ".final_rotation") {
        params_.final_rotation = parameter.as_bool();
      } else if (name == plugin_name_ + ".allow_backward") {
        if (params_.initial_rotation && parameter.as_bool()) {
          // 与起始旋转冲突：拒绝本次修改
          RCLCPP_WARN(
            logger_, "Initial rotation and allow backward parameters are both true, "
            "rejecting parameter change.");
          continue;
        }
        params_.allow_backward = parameter.as_bool();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_graceful_controller
