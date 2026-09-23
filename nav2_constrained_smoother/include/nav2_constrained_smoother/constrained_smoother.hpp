// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

// ============================================================================
// 文件说明（中文）：
// ConstrainedSmoother 插件类的头文件声明。
// 模块定位：nav2_constrained_smoother 包的对外插件接口，实现
// nav2_core::Smoother 接口，供 nav2_smoother 服务器（或
// SmacPlanner 的混合 A* 规划器）通过 pluginlib 加载。
// 算法为改进的 elastic band 平滑（Ceres 非线性最小二乘），
// 支持曲率约束（最小转弯半径）、代价地图避障、尖点（前进/倒车切换）
// 处理、路径降采样与三次贝塞尔上采样、起点/终点朝向保持。
// 本类是薄适配层：参数读取与消息格式转换；
// 核心算法见 smoother.hpp / smoother_cost_function.hpp / utils.hpp。
// ============================================================================

#ifndef NAV2_CONSTRAINED_SMOOTHER__CONSTRAINED_SMOOTHER_HPP_
#define NAV2_CONSTRAINED_SMOOTHER__CONSTRAINED_SMOOTHER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "nav2_core/smoother.hpp"
#include "nav2_constrained_smoother/smoother.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace nav2_constrained_smoother
{

/**
 * @class nav2_constrained_smoother::ConstrainedSmoother
 * @brief Regulated pure pursuit controller plugin
 */
class ConstrainedSmoother : public nav2_core::Smoother
{
public:
  /**
   * @brief Constructor for nav2_constrained_smoother::ConstrainedSmoother
   */
  ConstrainedSmoother() = default;

  /**
   * @brief Destrructor for nav2_constrained_smoother::ConstrainedSmoother
   */
  ~ConstrainedSmoother() override = default;

  /**
   * @brief Configure smoother parameters and member variables
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Method to smooth given path
   *
   * @param path In-out path to be optimized
   * @param max_time Maximum duration smoothing should take
   * @return Smoothed path
   */
  bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) override;

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_;   // TF 变换缓冲（预留）
  std::string plugin_name_;               // 本插件的配置名
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;  // 全局代价地图订阅器
  rclcpp::Logger logger_ {rclcpp::get_logger("ConstrainedSmoother")};  // 日志器

  // 核心优化器（Ceres 求解封装）及其参数
  std::unique_ptr<nav2_constrained_smoother::Smoother> smoother_;
  SmootherParams smoother_params_;    // 平滑算法参数（权重/曲率/采样倍率等）
  OptimizerParams optimizer_params_;  // Ceres 求解器参数
};

}  // namespace nav2_constrained_smoother

#endif  // NAV2_CONSTRAINED_SMOOTHER__CONSTRAINED_SMOOTHER_HPP_
