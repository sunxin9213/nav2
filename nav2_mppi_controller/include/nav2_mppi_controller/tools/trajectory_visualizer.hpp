// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_

#include <memory>
#include <string>
#include <xtensor/xtensor.hpp>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"

namespace mppi
{

/**
 * @class mppi::TrajectoryVisualizer
 * @brief Visualizes trajectories for debugging
 *（轨迹可视化器：将候选轨迹/最优轨迹/变换后路径发布为 RViz 消息）
 *
 * 中文说明：
 * 发布两类调试话题：
 * 1. MarkerArray —— 以散点形式显示全部候选轨迹与最优轨迹
 *    （通过 visualize_footprint 参数还可叠加 Footprint 足迹多边形）；
 * 2. Path —— 机器人坐标系下变换后的全局路径。
 * 遵循 ROS2 生命周期（on_configure/on_activate/on_deactivate/on_cleanup），
 * 发布器为 LifecyclePublisher，仅在激活状态下发布。
 */
class TrajectoryVisualizer
{
public:
  /**
    * @brief Constructor for mppi::TrajectoryVisualizer
    */
  TrajectoryVisualizer() = default;

  /**
    * @brief Configure trajectory visualizer
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param frame_id Frame to publish trajectories in
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    const std::string & frame_id, ParametersHandler * parameters_handler);

  /**
    * @brief Cleanup object on shutdown
    */
  void on_cleanup();

  /**
    * @brief Activate object
    */
  void on_activate();

  /**
    * @brief Deactivate object
    */
  void on_deactivate();

  /**
    * @brief Add an optimal trajectory to visualize
    * @param trajectory Optimal trajectory
    */
  void add(const xt::xtensor<float, 2> & trajectory, const std::string & marker_namespace);

  /**
    * @brief Add candidate trajectories to visualize
    * @param trajectories Candidate trajectories
    */
  void add(const models::Trajectories & trajectories, const std::string & marker_namespace);

  /**
    * @brief Visualize the plan
    * @param plan Plan to visualize
    */
  void visualize(const nav_msgs::msg::Path & plan);

  /**
    * @brief Reset object
    */
  void reset();

protected:
  std::string frame_id_;  // 发布轨迹所用的坐标系（通常为局部代价地图坐标系）
  // 轨迹散点标记发布器（MarkerArray，含全部候选轨迹与最优轨迹）
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
  trajectories_publisher_;
  // 变换后全局路径发布器（机器人坐标系下的路径，用于调试）
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> transformed_path_pub_;

  // 累积的轨迹点标记（每次 add() 追加，visualize() 时统一发布）
  std::unique_ptr<visualization_msgs::msg::MarkerArray> points_;
  int marker_id_ = 0;  // 标记 id 计数器（RViz 用 id 区分不同 marker）

  ParametersHandler * parameters_handler_;  // 参数处理器（读取可视化步长等）

  size_t trajectory_step_{0};  // 轨迹发布间隔：每隔多少条候选轨迹发布一条（降采样）
  size_t time_step_{0};       // 时间步发布间隔：每条轨迹每隔多少步取一个点（降采样）

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};  // 日志器
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_
