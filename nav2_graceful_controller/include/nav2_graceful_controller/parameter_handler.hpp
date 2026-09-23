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

#ifndef NAV2_GRACEFUL_CONTROLLER__PARAMETER_HANDLER_HPP_
#define NAV2_GRACEFUL_CONTROLLER__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_graceful_controller
{

// 中文说明：
// Parameters 结构体：Graceful Controller 的全部参数集合。
// 分为四组：路径变换（transform_tolerance 等）、控制律增益
//（k_phi/k_delta/beta/lambda）、速度限制（v_linear_min/max 等，
// *_initial 保存限速前的初值供恢复用）、行为开关与阈值
//（起始/终点旋转、减速半径、倒车）。
struct Parameters
{
  double transform_tolerance;         // TF 变换容差
  double motion_target_dist;          // 运动目标点（前探）距离
  double max_robot_pose_search_dist;  // 路径最近点搜索最大距离
  double k_phi;      // 慢子系统增益（φ-r 变化率比）
  double k_delta;    // 快子系统增益（朝向误差反馈）
  double beta;       // 速度-曲率衰减系数
  double lambda;     // 速度-曲率衰减指数
  double v_linear_min;   // 最小线速度
  double v_linear_max;   // 当前最大线速度
  double v_linear_max_initial;  // 初始最大线速度（限速恢复基准）
  double v_angular_max;   // 当前最大角速度
  double v_angular_max_initial;  // 初始最大角速度
  double slowdown_radius;  // 接近目标减速半径
  bool initial_rotation;   // 是否启用起始原地旋转
  double initial_rotation_min_angle;  // 触发起始旋转的最小角度
  bool final_rotation;     // 是否启用终点朝向旋转
  double rotation_scaling_factor;  // 旋转速度缩放系数
  bool allow_backward;     // 是否允许倒车
};

/**
 * @class nav2_graceful_controller::ParameterHandler
 * @brief Handles parameters and dynamic parameters for GracefulMotionController
 *（参数处理器：声明/读取参数并管理动态参数回调）
 *
 * 中文说明：
 * 构造时声明并读取全部参数，注册 ROS2 动态参数回调；
 * 计算线程通过 getParams()/getMutex() 在锁保护下访问最新参数。
 * 实现见 src/parameter_handler.cpp。
 */
class ParameterHandler
{
public:
  /**
   * @brief Constructor for nav2_graceful_controller::ParameterHandler
   */
  ParameterHandler(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::string & plugin_name,
    rclcpp::Logger & logger, const double costmap_size_x);

  /**
   * @brief Destructor for nav2_graceful_controller::ParameterHandler
   */
  ~ParameterHandler() = default;

  std::mutex & getMutex() {return mutex_;}

  Parameters * getParams() {return &params_;}

protected:
  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  Parameters params_;
  std::string plugin_name_;
  rclcpp::Logger logger_ {rclcpp::get_logger("GracefulMotionController")};
};

}  // namespace nav2_graceful_controller

#endif  // NAV2_GRACEFUL_CONTROLLER__PARAMETER_HANDLER_HPP_
