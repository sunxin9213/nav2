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

#ifndef NAV2_GRACEFUL_CONTROLLER__GRACEFUL_CONTROLLER_HPP_
#define NAV2_GRACEFUL_CONTROLLER__GRACEFUL_CONTROLLER_HPP_

#include <string>
#include <limits>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_graceful_controller/path_handler.hpp"
#include "nav2_graceful_controller/parameter_handler.hpp"
#include "nav2_graceful_controller/smooth_control_law.hpp"
#include "nav2_graceful_controller/utils.hpp"

namespace nav2_graceful_controller
{

/**
 * @class nav2_graceful_controller::GracefulController
 * @brief Graceful controller plugin
 *（优雅控制器插件）
 *
 * 中文说明：
 * Graceful Controller 是 Nav2 的几何跟踪控制器，基于 Smooth Control Law
 *（平滑控制律，Yoshihiro Kusano 等 2021 IEEE-AC 混合 A* 后处理控制器的
 * 改进版）。核心思想：在“自极坐标（ego-polar）”坐标系中用非线性反馈
 * 控制律（SmoothControlLaw）直接计算曲率与速度指令，收敛到“运动目标点”
 *（motion target，路径上距机器人固定距离的前探点）。
 * 特点：
 * 1. 输出天然平滑（速度与曲率连续变化），无需后处理平滑；
 * 2. 通过前瞻点距离自适应实现减速；
 * 3. 接近目标时平滑收敛到目标位姿（含朝向）；
 * 4. 仿真前向轨迹（simulateTrajectory）检查碰撞，
 *    碰撞时尝试减速（slowdown）或反向（backward）。
 * 在 Nav2 中实现 nav2_core::Controller 接口，由 controller_server 加载。
 */
class GracefulController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for nav2_graceful_controller::GracefulController
   */
  GracefulController() = default;

  /**
   * @brief Destructor for nav2_graceful_controller::GracefulController
   */
  ~GracefulController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine.
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine.
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine.
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity.
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan.
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed
   * @param percentage setting speed limit in percentage if true
   * or in absolute values in false case
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Get motion target point.
   * @param motion_target_dist Optimal motion target distance
   * @param path Current global path
   * @return Motion target point
   */
  geometry_msgs::msg::PoseStamped getMotionTarget(
    const double & motion_target_dist,
    const nav_msgs::msg::Path & path);

  /**
   * @brief Simulate trajectory calculating in every step the new velocity command based on
   * a new curvature value and checking for collisions.
   *
   * @param robot_pose Robot pose
   * @param motion_target Motion target point
   * @param costmap_transform Transform between global and local costmap
   * @param trajectory Simulated trajectory
   * @param backward Flag to indicate if the robot is moving backward
   * @return true if the trajectory is collision free, false otherwise
   */
  bool simulateTrajectory(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & motion_target,
    const geometry_msgs::msg::TransformStamped & costmap_transform,
    nav_msgs::msg::Path & trajectory,
    const bool & backward);

  /**
   * @brief Rotate the robot to face the motion target with maximum angular velocity.
   *
   * @param angle_to_target Angle to the motion target
   * @return geometry_msgs::msg::Twist Velocity command
   */
  geometry_msgs::msg::Twist rotateToTarget(
    const double & angle_to_target);

  /**
   * @brief Checks if the robot is in collision
   * @param x The x coordinate of the robot in global frame
   * @param y The y coordinate of the robot in global frame
   * @param theta The orientation of the robot in global frame
   * @return Whether in collision
   */
  bool inCollision(const double & x, const double & y, const double & theta);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;   // TF2 缓冲区（坐标变换）
  std::string plugin_name_;                     // 插件名称
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  // 局部代价地图
  // 足迹碰撞检查器（前向仿真轨迹的碰撞检查）
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;
  rclcpp::Logger logger_{rclcpp::get_logger("GracefulController")};  // 日志器

  Parameters * params_;          // 控制器参数（由 param_handler_ 管理）
  double goal_dist_tolerance_;  // 目标位置容差
  bool goal_reached_;           // 是否已到达目标（进入朝向对齐阶段）

  // 调试发布器：变换后的局部路径
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> transformed_plan_pub_;
  // 调试发布器：仿真的局部轨迹
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
  // 调试发布器：运动目标点（前探点）
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
  motion_target_pub_;
  // 调试发布器：减速标记
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>>
  slowdown_pub_;
  // 组件：路径处理（截断/变换全局路径）
  std::unique_ptr<nav2_graceful_controller::PathHandler> path_handler_;
  // 组件：参数处理（含动态参数回调）
  std::unique_ptr<nav2_graceful_controller::ParameterHandler> param_handler_;
  // 组件：平滑控制律（曲率/速度计算核心）
  std::unique_ptr<nav2_graceful_controller::SmoothControlLaw> control_law_;
};

}  // namespace nav2_graceful_controller

#endif  // NAV2_GRACEFUL_CONTROLLER__GRACEFUL_CONTROLLER_HPP_
