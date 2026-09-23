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

#ifndef DWB_CORE__PUBLISHER_HPP_
#define DWB_CORE__PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_msgs/msg/local_plan_evaluation.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "builtin_interfaces/msg/duration.hpp"

using rclcpp_lifecycle::LifecyclePublisher;

namespace dwb_core
{

/**
 * @class DWBPublisher
 * @brief Consolidation of all the publishing logic for the DWB Local Planner.
 *
 * Right now, it can publish
 *   1) The Global Plan (as passed in using setPath)
 *   2) The Local Plan (after it is calculated)
 *   3) The Transformed Global Plan (since it may be different than the global)
 *   4) The Full LocalPlanEvaluation
 *   5) Markers representing the different trajectories evaluated
 *   6) The CostGrid (in the form of a complex PointCloud2)
 * （DWB 发布器：集中管理调试信息的发布，包括全局/局部路径、
 *  完整评分评价消息、候选轨迹可视化 Marker 以及评价器逐格评分
 *  点云。所有发布器均为 LifecyclePublisher，随节点生命周期启停。）
 */
class DWBPublisher
{
public:
  explicit DWBPublisher(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name);

  // 四个生命周期回调：创建 -> 激活 -> 去激活 -> 清理发布器
  nav2_util::CallbackReturn on_configure();
  nav2_util::CallbackReturn on_activate();
  nav2_util::CallbackReturn on_deactivate();
  nav2_util::CallbackReturn on_cleanup();

  /**
   * @brief Does the publisher require that the LocalPlanEvaluation be saved
   * @return True if the Evaluation is needed to publish either directly or as trajectories
   */
  // 是否需要记录完整评价结果（供发布评价消息或轨迹可视化用）
  bool shouldRecordEvaluation() {return publish_evaluation_ || publish_trajectories_;}

  /**
   * @brief If the pointer is not null, publish the evaluation and trajectories as needed
   */
  void publishEvaluation(std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results);
  void publishLocalPlan(   // 发布选中的局部轨迹（Trajectory2D 版本）
    const std_msgs::msg::Header & header,
    const dwb_msgs::msg::Trajectory2D & traj);
  void publishCostGrid(    // 发布评价器逐格评分点云（RViz 调试用）
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    const std::vector<TrajectoryCritic::Ptr> critics);
  void publishGlobalPlan(const nav_2d_msgs::msg::Path2D plan);        // 发布原始全局路径
  void publishTransformedPlan(const nav_2d_msgs::msg::Path2D plan);    // 发布变换后的路径
  void publishLocalPlan(const nav_2d_msgs::msg::Path2D plan);          // 发布局部路径

protected:
  void publishTrajectories(const dwb_msgs::msg::LocalPlanEvaluation & results);

  // Helper function for publishing other plans
  // 通用路径发布辅助：检查订阅数与开关后发布
  void publishGenericPlan(
    const nav_2d_msgs::msg::Path2D plan,
    rclcpp::Publisher<nav_msgs::msg::Path> & pub, bool flag);

  // Flags for turning on/off publishing specific components
  bool publish_evaluation_;      // 发布评分评价消息
  bool publish_global_plan_;    // 发布全局路径
  bool publish_transformed_;    // 发布变换后路径
  bool publish_local_plan_;     // 发布局部路径
  bool publish_trajectories_;   // 发布候选轨迹 Marker
  bool publish_cost_grid_pc_;   // 发布逐格评分点云
  bool publish_input_params_;

  // Marker Lifetime
  builtin_interfaces::msg::Duration marker_lifetime_;  // 轨迹 Marker 的存活时长

  // Publisher Objects
  std::shared_ptr<LifecyclePublisher<dwb_msgs::msg::LocalPlanEvaluation>> eval_pub_;  // 评价消息
  std::shared_ptr<LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;   // 全局路径
  std::shared_ptr<LifecyclePublisher<nav_msgs::msg::Path>> transformed_pub_;  // 变换路径
  std::shared_ptr<LifecyclePublisher<nav_msgs::msg::Path>> local_pub_;    // 局部路径
  std::shared_ptr<LifecyclePublisher<visualization_msgs::msg::MarkerArray>> marker_pub_;  // 轨迹 Marker
  std::shared_ptr<LifecyclePublisher<sensor_msgs::msg::PointCloud2>> cost_grid_pc_pub_;  // 评分点云

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;  // 生命周期节点弱引用
  rclcpp::Clock::SharedPtr clock_;                 // 时钟
  std::string plugin_name_;                       // 插件名（参数命名空间）
};

}  // namespace dwb_core

#endif  // DWB_CORE__PUBLISHER_HPP_
