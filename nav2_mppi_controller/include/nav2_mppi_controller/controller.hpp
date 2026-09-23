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

#ifndef NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
#define NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_

#include <string>
#include <memory>

#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_mppi_controller
{

using namespace mppi;  // NOLINT

/**
 * @class mppi::MPPIController
 * @brief Main plugin controller for MPPI Controller
 * （MPPI 控制器的 pluginlib 插件入口。实现 nav2_core::Controller 接口，
 *  被 nav2_controller 作为 "FollowPath" 插件加载。内部组合三大部件：
 *  - Optimizer：MPPI 优化核心（采样、推演、打分、加权平均）；
 *  - PathHandler：全局路径的裁剪与坐标变换；
 *  - ParametersHandler：参数读取与动态调参管理。
 *  computeVelocityCommands() 每个控制周期被调用一次，输出速度指令。）
 */
class MPPIController : public nav2_core::Controller
{
public:
  /**
    * @brief Constructor for mppi::MPPIController
    */
  MPPIController() = default;

  /**
    * @brief Configure controller on bringup
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param tf TF buffer to use
    * @param costmap_ros Costmap2DROS object of environment
    */
  // 生命周期配置：初始化各子部件并注册参数
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
    * @brief Cleanup resources
    */
  void cleanup() override;  // 生命周期清理：注销回调、释放资源

  /**
    * @brief Activate controller
    */
  void activate() override;  // 激活：创建发布器、启动优化器异步线程

  /**
    * @brief Deactivate controller
    */
  void deactivate() override;  // 去激活：停线程、停发布器

  /**
    * @brief Reset the controller state between tasks
    */
  void reset();  // 任务间复位：清空控制序列等内部状态

  /**
    * @brief Main method to compute velocities using the optimizer
    * @param robot_pose Robot pose
    * @param robot_speed Robot speed
    * @param goal_checker Pointer to the goal checker for awareness if completed task
    */
  // 主入口：路径变换 -> 优化器求最优控制 -> 可视化 -> 返回 TwistStamped
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker) override;

  /**
    * @brief Set new reference path to track
    * @param path Path to track
    */
  void setPlan(const nav_msgs::msg::Path & path) override;  // 接收全局规划器的新路径

  /**
    * @brief Set new speed limit from callback
    * @param speed_limit Speed limit to use
    * @param percentage Bool if the speed limit is absolute or relative
    */
  // 限速回调：由速度限制话题触发（绝对值或百分比两种模式）
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
    * @brief Visualize trajectories
    * @param transformed_plan Transformed input plan
    */
  void visualize(nav_msgs::msg::Path transformed_plan);  // RViz 可视化：参考路径 + 最优轨迹

  std::string name_;                        // 插件名（如 FollowPath）
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;  // 生命周期节点弱引用
  rclcpp::Clock::SharedPtr clock_;           // 时钟
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  // 代价地图
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                   // TF 缓冲

  std::unique_ptr<ParametersHandler> parameters_handler_;  // 参数管理器
  Optimizer optimizer_;                    // MPPI 优化核心
  PathHandler path_handler_;               // 路径裁剪与变换
  TrajectoryVisualizer trajectory_visualizer_;  // 轨迹可视化

  bool visualize_;                          // 是否开启 RViz 可视化

  double reset_period_;                     // 状态自动复位周期（秒），
                                            // 长时间未被调用则清空优化器状态
  // Last time computeVelocityCommands was called
  rclcpp::Time last_time_called_;           // 上次 computeVelocityCommands 调用时刻
};

}  // namespace nav2_mppi_controller

#endif  // NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
