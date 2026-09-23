// Copyright (c) 2021 Samsung Research America
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

#ifndef NAV2_ROTATION_SHIM_CONTROLLER__NAV2_ROTATION_SHIM_CONTROLLER_HPP_
#define NAV2_ROTATION_SHIM_CONTROLLER__NAV2_ROTATION_SHIM_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_controller/plugins/position_goal_checker.hpp"
#include "angles/angles.h"

namespace nav2_rotation_shim_controller
{

/**
 * @class nav2_rotation_shim_controller::RotationShimController
 * @brief Rotate to rough path heading controller shim plugin
 *（旋转垫片控制器：在主控制器外层包装“先原地旋转对准路径朝向”的行为）
 *
 * 中文说明：
 * “垫片（shim）”控制器：不实现路径跟踪本身，而是在将工作转交给
 * 内部加载的主控制器（primary_controller_，如 Pure Pursuit/DWB/MPPI）
 * 之前，先处理大角度偏差场景——当机器人朝向与路径方向夹角超过
 * angular_dist_threshold 时，先原地旋转到大致对准路径（computeRotateToHeadingCommand，
 * 带角速度加减速与碰撞检查 isCollisionFree），再交给主控制器跟踪。
 * 适用于差速底盘在路径起点朝向偏差大、或换路径时需要先转向的情况。
 * 在 Nav2 中由 controller_server 通过 pluginlib 加载，自身再通过
 * lp_loader_ 动态加载真正的主控制器插件。
 */
class RotationShimController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for nav2_rotation_shim_controller::RotationShimController
   */
  RotationShimController();

  /**
   * @brief Destrructor for nav2_rotation_shim_controller::RotationShimController
   */
  ~RotationShimController() override = default;

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
   * @brief Compute the best command given the current pose and velocity
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Finds the point on the path that is roughly the sampling
   * point distance away from the robot for use.
   * May throw exception if a point at least that far away cannot be found
   * @return pt location of the output point
   */
  geometry_msgs::msg::PoseStamped getSampledPathPt();

  /**
   * @brief Find the goal point in path
   * May throw exception if the path is empty
   * @return pt location of the output point
   */
  geometry_msgs::msg::PoseStamped getSampledPathGoal();

  /**
   * @brief Uses TF to find the location of the sampled path point in base frame
   * @param pt location of the sampled path point
   * @return location of the pose in base frame
   */
  geometry_msgs::msg::Pose transformPoseToBaseFrame(const geometry_msgs::msg::PoseStamped & pt);

  /**
   * @brief Rotates the robot to the rough heading
   * @param angular_distance Angular distance to the goal remaining
   * @param pose Starting pose of robot
   * @param velocity Starting velocity of robot
   * @return Twist command for rotation to rough heading
   */
  geometry_msgs::msg::TwistStamped computeRotateToHeadingCommand(
    const double & angular_distance,
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);

  /**
   * @brief Checks if rotation is safe
   * @param cmd_vel Velocity to check over
   * @param angular_distance_to_heading Angular distance to heading requested
   * @param pose Starting pose of robot
   */
  void isCollisionFree(
    const geometry_msgs::msg::TwistStamped & cmd_vel,
    const double & angular_distance_to_heading,
    const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;   // 父生命周期节点（controller_server）
  std::shared_ptr<tf2_ros::Buffer> tf_;            // TF2 缓冲区（采样点转机器人坐标系）
  std::string plugin_name_;                        // 本插件名称
  rclcpp::Logger logger_ {rclcpp::get_logger("RotationShimController")};  // 日志器
  rclcpp::Clock::SharedPtr clock_;                 // 节点时钟
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  // 局部代价地图
  // 足迹碰撞检查器（旋转安全性检查用）
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;

  // pluginlib 加载器：动态加载主控制器（如 RPP/DWB/MPPI）
  pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
  nav2_core::Controller::Ptr primary_controller_;  // 被包装的主控制器实例
  bool path_updated_;                              // 新路径是否已下发标志
  nav_msgs::msg::Path current_path_;                // 当前全局路径
  // forward_sampling_distance：在路径上取采样点的距离（估计路径方向用）
  // angular_dist_threshold：触发旋转的角度阈值
  // angular_disengage_threshold：退出旋转状态的角度阈值（通常更小）
  double forward_sampling_distance_, angular_dist_threshold_, angular_disengage_threshold_;
  double rotate_to_heading_angular_vel_, max_angular_accel_;  // 旋转角速度与最大角加速度
  double control_duration_, simulate_ahead_time_;  // 控制周期 / 前向碰撞仿真时长
  // rotate_to_goal_heading：接近目标时是否旋转到目标朝向；in_rotation：当前是否处于旋转状态
  bool rotate_to_goal_heading_, in_rotation_;
  bool closed_loop_;  // 旋转是否采用闭环（按剩余角度实时计算）控制
  double last_angular_vel_ = std::numeric_limits<double>::max();  // 上一周期角速度（开环模式用）

  // Dynamic parameters handler（动态参数回调）
  std::mutex mutex_;  // 保护动态参数的互斥锁
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  // 位置目标检查器（判断是否到达目标位置、仅旋转到朝向阶段使用）
  std::unique_ptr<nav2_controller::PositionGoalChecker> position_goal_checker_;
};

}  // namespace nav2_rotation_shim_controller

#endif  // NAV2_ROTATION_SHIM_CONTROLLER__NAV2_ROTATION_SHIM_CONTROLLER_HPP_
