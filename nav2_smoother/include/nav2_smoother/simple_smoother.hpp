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
// SimpleSmoother 插件类的头文件声明。
// 模块定位：nav2_smoother 的默认平滑算法插件。采用迭代梯度下降式平滑：
// 每个内部点在"数据保真项 w_data*(x_i-y_i)"与"平滑项
// w_smooth*(y_{i+1}+y_{i-1}-2*y_i)（离散二阶差分）"的共同作用下更新，
// 首尾端点固定不动，收敛阈值 tolerance、迭代上限 max_its、
// 时间预算与代价地图碰撞检查共同保障结果可行。
// 实现（.cpp）见 src/simple_smoother.cpp。
// ============================================================================

#ifndef NAV2_SMOOTHER__SIMPLE_SMOOTHER_HPP_
#define NAV2_SMOOTHER__SIMPLE_SMOOTHER_HPP_

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
 * @class nav2_smoother::SimpleSmoother
 * @brief A path smoother implementation
 */
class SimpleSmoother : public nav2_core::Smoother
{
public:
  /**
   * @brief A constructor for nav2_smoother::SimpleSmoother
   */
  SimpleSmoother() = default;

  /**
   * @brief A destructor for nav2_smoother::SimpleSmoother
   */
  ~SimpleSmoother() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) override;

  /**
   * @brief Method to cleanup resources.
   */
  void cleanup() override {costmap_sub_.reset();}

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
    bool & reversing_segment,
    const nav2_costmap_2d::Costmap2D * costmap,
    const double & max_time);

  /**
   * @brief Get the field value for a given dimension
   * @param msg Current pose to sample
   * @param dim Dimension ID of interest
   * @return dim value
   */
  inline double getFieldByDim(
    const geometry_msgs::msg::PoseStamped & msg,
    const unsigned int & dim);

  /**
   * @brief Set the field value for a given dimension
   * @param msg Current pose to sample
   * @param dim Dimension ID of interest
   * @param value to set the dimention to for the pose
   */
  inline void setFieldByDim(
    geometry_msgs::msg::PoseStamped & msg, const unsigned int dim,
    const double & value);

  double tolerance_, data_w_, smooth_w_;  // 收敛容差 / 数据保真权重 / 平滑权重
  int max_its_, refinement_ctr_;          // 最大迭代次数 / 已精炼次数（上限 4）
  bool do_refinement_;                    // 收敛后是否递归精炼
  // 全局代价地图订阅器（平滑过程中的碰撞检查）
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  rclcpp::Logger logger_{rclcpp::get_logger("SimpleSmoother")};  // 日志器
};

}  // namespace nav2_smoother

#endif  // NAV2_SMOOTHER__SIMPLE_SMOOTHER_HPP_
