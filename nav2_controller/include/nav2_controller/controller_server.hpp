// Copyright (c) 2019 Intel Corporation
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

/*
 * =============================================================================
 * 文件：controller_server.hpp（nav2_controller 模块）
 * =============================================================================
 * 功能概述：
 *   声明 ControllerServer 类——Nav2 的局部控制服务器（生命周期节点）。
 *   核心设计要点：
 *   - 继承 nav2_util::LifecycleNode，由 lifecycle_manager 统一驱动
 *     configure/activate/deactivate/cleanup 状态机；
 *   - pluginlib 三类插件：
 *       * nav2_core::Controller       —— 局部控制算法（DWB、RPP、MPPI 等），
 *                                        以 controller_id → 实例 的哈希表管理；
 *       * nav2_core::GoalChecker      —— 判定是否到达目标（可配多个）；
 *       * nav2_core::ProgressChecker  —— 判定机器人是否仍在前进（防卡死）；
 *   - 动作服务：SimpleActionServer 封装的 FollowPath 动作，
 *     执行回调 computeControl() 构成局部控制闭环；
 *   - 速度输出：发布 geometry_msgs::msg::Twist 到 cmd_vel 话题
 *     （通常由下游 velocity_smoother 订阅平滑后再发给底盘）；
 *   - 其它输入：odom（OdomSubscriber）、speed_limit（限速消息订阅）。
 *
 * 在整个导航流程中的位置：
 *   bt_navigator → planner_server → controller_server【本类】→ velocity_smoother。
 *   对应实现见 controller_server.cpp。
 * =============================================================================
 */

#ifndef NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_
#define NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_controller
{

class ProgressChecker;
/**
 * @class nav2_controller::ControllerServer
 * @brief This class hosts variety of plugins of different algorithms to
 * complete control tasks from the exposed FollowPath action server.
 */
// 中文说明：ControllerServer 宿主多种控制算法插件，通过 FollowPath
// 动作服务器接收全局路径，并按 controller_frequency 周期调用当前
// 控制器插件计算速度指令，配合 goal/progress checker 判定任务完成或卡死。
class ControllerServer : public nav2_util::LifecycleNode
{
public:
  using ControllerMap = std::unordered_map<std::string, nav2_core::Controller::Ptr>;
  using GoalCheckerMap = std::unordered_map<std::string, nav2_core::GoalChecker::Ptr>;

  /**
   * @brief Constructor for nav2_controller::ControllerServer
   * @param options Additional options to control creation of the node.
   */
  explicit ControllerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for nav2_controller::ControllerServer
   */
  ~ControllerServer();

protected:
  /**
   * @brief Configures controller parameters and member variables
   *
   * Configures controller plugin and costmap; Initialize odom subscriber,
   * velocity publisher and follow path action server.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize controller
   * plugin
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates member variables
   *
   * Activates controller, costmap, velocity publisher and follow path action
   * server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates member variables
   *
   * Deactivates follow path action server, controller, costmap and velocity
   * publisher. Before calling deactivate state, velocity is being set to zero.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Calls clean up states and resets member variables.
   *
   * Controller and costmap clean up state is called, and resets rest of the
   * variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // ---- 动作类型别名 ----
  // Action：FollowPath 动作定义（目标=全局路径+controller_id，结果=执行状态）；
  // ActionServer：nav2_util::SimpleActionServer 高层封装，
  //   内部基于 rclcpp_action::create_server 实现，处理 handle_goal/handle_cancel/
  //   handle_accepted，并在专属线程中执行绑定到本类的 computeControl 回调。
  using Action = nav2_msgs::action::FollowPath;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  // Our action server implements the FollowPath action
  std::unique_ptr<ActionServer> action_server_;

  /**
   * @brief FollowPath action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * Provides global path to controller received from action client. Twist
   * velocities for the robot are calculated and published using controller at
   * the specified rate till the goal is reached.
   * @throw nav2_core::PlannerException
   */
  void computeControl();

  /**
   * @brief Find the valid controller ID name for the given request
   *
   * @param c_name The requested controller name
   * @param name Reference to the name to use for control if any valid available
   * @return bool Whether it found a valid controller to use
   */
  bool findControllerId(const std::string & c_name, std::string & name);

  /**
   * @brief Find the valid goal checker ID name for the specified parameter
   *
   * @param c_name The goal checker name
   * @param name Reference to the name to use for goal checking if any valid available
   * @return bool Whether it found a valid goal checker to use
   */
  bool findGoalCheckerId(const std::string & c_name, std::string & name);

  /**
   * @brief Assigns path to controller
   * @param path Path received from action server
   */
  void setPlannerPath(const nav_msgs::msg::Path & path);
  /**
   * @brief Calculates velocity and publishes to "cmd_vel" topic
   */
  void computeAndPublishVelocity();
  /**
   * @brief Calls setPlannerPath method with an updated path received from
   * action server
   */
  void updateGlobalPath();
  /**
   * @brief Calls velocity publisher to publish the velocity on "cmd_vel" topic
   * @param velocity Twist velocity to be published
   */
  void publishVelocity(const geometry_msgs::msg::TwistStamped & velocity);
  /**
   * @brief Calls velocity publisher to publish zero velocity
   */
  void publishZeroVelocity();
  /**
   * @brief Checks if goal is reached
   * @return true or false
   */
  bool isGoalReached();
  /**
   * @brief Obtain current pose of the robot
   * @param pose To store current pose of the robot
   * @return true if able to obtain current pose of the robot, else false
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief get the thresholded velocity
   * @param velocity The current velocity from odometry
   * @param threshold The minimum velocity to return non-zero
   * @return double velocity value
   */
  double getThresholdedVelocity(double velocity, double threshold)
  {
    return (std::abs(velocity) > threshold) ? velocity : 0.0;
  }

  /**
   * @brief get the thresholded Twist
   * @param Twist The current Twist from odometry
   * @return Twist Twist after thresholds applied
   */
  nav_2d_msgs::msg::Twist2D getThresholdedTwist(const nav_2d_msgs::msg::Twist2D & twist)
  {
    nav_2d_msgs::msg::Twist2D twist_thresh;
    twist_thresh.x = getThresholdedVelocity(twist.x, min_x_velocity_threshold_);
    twist_thresh.y = getThresholdedVelocity(twist.y, min_y_velocity_threshold_);
    twist_thresh.theta = getThresholdedVelocity(twist.theta, min_theta_velocity_threshold_);
    return twist_thresh;
  }

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // ---- 动态参数 ----
  // dyn_params_handler_：add_on_set_parameters_callback 注册的回调句柄，
  //                       on_deactivate 时 reset 注销；
  // dynamic_params_lock_：保护控制循环与参数回调之间的共享状态。
  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  // ---- 局部 Costmap ----
  // 控制器需要局部代价地图（local_costmap）来避障；
  // costmap_thread_：Costmap 的专属执行线程，避免与主执行器互相阻塞。
  // The controller needs a costmap node
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

  // ---- 发布器与订阅器 ----
  // odom_sub_：里程计订阅（nav_2d_utils::OdomSubscriber，含超时检测），
  //            为控制器提供当前速度反馈；
  // vel_publisher_：速度发布器（生命周期），发布 Twist 到 cmd_vel；
  // speed_limit_sub_：speed_limit 话题订阅，接收外部限速百分比/绝对值。
  // Publishers and subscribers
  std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;

  // ---- 进度检查器（Progress Checker）插件 ----
  // progress_checker_loader_：pluginlib 类加载器（基接口 nav2_core::ProgressChecker）；
  // progress_checker_：当前实例（单个，判断机器人是否持续向目标推进）；
  // default_*/（无前缀）：默认与实际使用的插件 id / 类型。
  // Progress Checker Plugin
  pluginlib::ClassLoader<nav2_core::ProgressChecker> progress_checker_loader_;
  nav2_core::ProgressChecker::Ptr progress_checker_;
  std::string default_progress_checker_id_;
  std::string default_progress_checker_type_;
  std::string progress_checker_id_;
  std::string progress_checker_type_;

  // ---- 目标检查器（Goal Checker）插件 ----
  // goal_checker_loader_：pluginlib 类加载器（基接口 nav2_core::GoalChecker）；
  // goal_checkers_：goal_checker_id → 实例 的哈希表（可加载多个，按目标选用），
  //                 判定机器人是否到达终点（位置/朝向/速度容差等）。
  // Goal Checker Plugin
  pluginlib::ClassLoader<nav2_core::GoalChecker> goal_checker_loader_;
  GoalCheckerMap goal_checkers_;
  std::vector<std::string> default_goal_checker_ids_;
  std::vector<std::string> default_goal_checker_types_;
  std::vector<std::string> goal_checker_ids_;
  std::vector<std::string> goal_checker_types_;
  std::string goal_checker_ids_concat_, current_goal_checker_;

  // ---- 控制器（Controller）插件 ----
  // lp_loader_：pluginlib 类加载器（基接口 nav2_core::Controller）；
  // controllers_：controller_id → 实例 的哈希表（FollowPath、DWB、RPP、MPPI 等）；
  // current_controller_：本次 FollowPath 目标选用的控制器 id。
  // Controller Plugins
  pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
  ControllerMap controllers_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> controller_ids_;
  std::vector<std::string> controller_types_;
  std::string controller_ids_concat_, current_controller_;

  // ---- 控制循环参数 ----
  // controller_frequency_：控制周期频率（Hz），computeControl 的循环节拍；
  // min_*_velocity_threshold_：odom 速度死区阈值，低于阈值的分量视为 0，
  //                            避免编码器噪声被当作真实运动反馈给控制器。
  double controller_frequency_;
  double min_x_velocity_threshold_;
  double min_y_velocity_threshold_;
  double min_theta_velocity_threshold_;

  // failure_tolerance_：控制器抛出异常后的容忍时长（秒），
  //                    期间沿用上一次有效速度指令，超时才判定控制失败；
  // publish_zero_velocity_：动作结束后是否向 cmd_vel 发布零速度停车。
  double failure_tolerance_;
  bool publish_zero_velocity_;

  // 本次任务的终点位姿（由 goal checker 使用）
  // Whether we've published the single controller warning yet
  geometry_msgs::msg::PoseStamped end_pose_;

  // 最近一次生成有效速度指令的时间（配合 failure_tolerance_ 使用）
  // Last time the controller generated a valid command
  rclcpp::Time last_valid_cmd_time_;

  // 当前正在跟踪的全局路径
  // Current path container
  nav_msgs::msg::Path current_path_;

private:
  /**
    * @brief Callback for speed limiting messages
    * @param msg Shared pointer to nav2_msgs::msg::SpeedLimit
    */
  void speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_
