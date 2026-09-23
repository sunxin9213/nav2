// Copyright (c) 2022, Samsung Research America
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
// limitations under the License. Reserved.

// ============================================================================
// 文件说明（中文）：
// SavitzkyGolaySmoother 插件类的头文件声明。
// 模块定位：nav2_smoother 的平滑算法插件，基于 Savitzky-Golay 滤波器
// （7 点卷积平滑）对路径 x/y 坐标做低通平滑，可选多轮精炼。
// 实现（.cpp）见 src/savitzky_golay_smoother.cpp。
// 注意：该插件不使用 TF / 代价地图（不做碰撞检查），接口参数仅占位。
// ============================================================================

#ifndef NAV2_SMOOTHER__SAVITZKY_GOLAY_SMOOTHER_HPP_
#define NAV2_SMOOTHER__SAVITZKY_GOLAY_SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include "nav2_core/smoother.hpp"
#include "nav2_smoother/smoother_utils.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "angles/angles.h"
#include "tf2/utils.h"

namespace nav2_smoother
{

/**
 * @class nav2_smoother::SavitzkyGolaySmoother
 * @brief A path smoother implementation using Savitzky Golay filters
 */
class SavitzkyGolaySmoother : public nav2_core::Smoother
{
public:
  /**
   * @brief A constructor for nav2_smoother::SavitzkyGolaySmoother
   */
  SavitzkyGolaySmoother() = default;

  /**
   * @brief A destructor for nav2_smoother::SavitzkyGolaySmoother
   */
  ~SavitzkyGolaySmoother() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) override;

  /**
   * @brief Method to cleanup resources.
   */
  void cleanup() override {}

  /**
   * @brief Method to activate smoother and any threads involved in execution.
   */
  void activate() override {}

  /**
   * @brief Method to deactivate smoother and any threads involved in execution.
   */
  void deactivate() override {}

  /**
   * @brief Method to smooth given path
   *
   * @param path In-out path to be smoothed
   * @param max_time Maximum duration smoothing should take
   * @return If smoothing was completed (true) or interrupted by time limit (false)
   */
  bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) override;

protected:
  /**
   * @brief Smoother method - does the smoothing on a segment
   * @param path Reference to path
   * @param reversing_segment Return if this is a reversing segment
   * @param costmap Pointer to minimal costmap
   * @param max_time Maximum time to compute, stop early if over limit
   * @return If smoothing was successful
   */
  bool smoothImpl(
    nav_msgs::msg::Path & path,
    bool & reversing_segment);

  bool do_refinement_;   // 是否在平滑后进行多轮精炼
  int refinement_num_;   // 精炼迭代次数
  rclcpp::Logger logger_{rclcpp::get_logger("SGSmoother")};  // 日志器
};

}  // namespace nav2_smoother

#endif  // NAV2_SMOOTHER__SAVITZKY_GOLAY_SMOOTHER_HPP_
