// Copyright (c) 2019 Samsung Research America
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
 * 文件：planner_server.hpp（nav2_planner 模块）
 * =============================================================================
 * 功能概述：
 *   声明 PlannerServer 类——Nav2 的全局规划服务器（生命周期节点）。
 *   核心设计要点：
 *   - 继承 nav2_util::LifecycleNode：参与 lifecycle_manager 统一管理的
 *     状态机（unconfigured → inactive → active → finalized ...），
 *     资源的创建/销毁分别在 on_configure/on_cleanup 等回调中完成；
 *   - pluginlib 插件化：通过 ClassLoader<nav2_core::GlobalPlanner> 在
 *     运行期按参数 planner_plugins 动态加载任意全局规划算法实现；
 *   - 动作服务：借助 nav2_util::SimpleActionServer<T> 封装
 *     rclcpp_action 动作服务器，对外提供 compute_path_to_pose 与
 *     compute_path_through_poses 两个动作，供行为树导航器调用；
 *   - 服务：is_path_valid 查询路径在当前地图上是否仍然可行；
 *   - 动态参数：add_on_set_parameters_callback 支持运行期修改部分参数。
 *
 * 在整个导航流程中的位置：
 *   bt_navigator → planner_server（本类）→ controller_server → velocity_smoother。
 *   对应实现见 planner_server.cpp。
 * =============================================================================
 */

#ifndef NAV2_PLANNER__PLANNER_SERVER_HPP_
#define NAV2_PLANNER__PLANNER_SERVER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"

namespace nav2_planner
{
/**
 * @class nav2_planner::PlannerServer
 * @brief An action server implements the behavior tree's ComputePathToPose
 * interface and hosts various plugins of different algorithms to compute plans.
 */
// 中文说明：PlannerServer 是实现行为树 ComputePathToPose 接口的动作服务器，
// 宿主并管理多个全局规划算法插件（以 planner_id → 插件实例 的哈希表组织），
// 按 Action 目标中指定的 planner_id 选用对应算法计算全局路径。
class PlannerServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_planner::PlannerServer
   * @param options Additional options to control creation of the node.
   */
  explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for nav2_planner::PlannerServer
   */
  ~PlannerServer();

  using PlannerMap = std::unordered_map<std::string, nav2_core::GlobalPlanner::Ptr>;

  /**
   * @brief Method to get plan from the desired plugin
   * @param start starting pose
   * @param goal goal request
   * @return Path
   */
  nav_msgs::msg::Path getPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id);

protected:
  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using ActionToPose = nav2_msgs::action::ComputePathToPose;
  using ActionThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  using ActionServerToPose = nav2_util::SimpleActionServer<ActionToPose>;
  using ActionServerThroughPoses = nav2_util::SimpleActionServer<ActionThroughPoses>;

  /**
   * @brief Check if an action server is valid / active
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isServerInactive(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Check if an action server has a cancellation request pending
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isCancelRequested(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Wait for costmap to be valid with updated sensor data or repopulate after a
   * clearing recovery. Blocks until true without timeout.
   */
  void waitForCostmap();

  /**
   * @brief Check if an action server has a preemption request and replaces the goal
   * with the new preemption goal.
   * @param action_server Action server to get updated goal if required
   * @param goal Goal to overwrite
   */
  template<typename T>
  void getPreemptedGoalIfRequested(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> goal);

  /**
   * @brief Get the starting pose from costmap or message, if valid
   * @param action_server Action server to terminate if required
   * @param goal Goal to find start from
   * @param start The starting pose to use
   * @return bool If successful in finding a valid starting pose
   */
  template<typename T>
  bool getStartPose(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> goal,
    geometry_msgs::msg::PoseStamped & start);

  /**
   * @brief Transform start and goal poses into the costmap
   * global frame for path planning plugins to utilize
   * @param action_server Action server to terminate if required
   * @param start The starting pose to transform
   * @param goal Goal pose to transform
   * @return bool If successful in transforming poses
   */
  template<typename T>
  bool transformPosesToGlobalFrame(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    geometry_msgs::msg::PoseStamped & curr_start,
    geometry_msgs::msg::PoseStamped & curr_goal);

  /**
   * @brief Validate that the path contains a meaningful path
   * @param action_server Action server to terminate if required
   * @param goal Goal Current goal
   * @param path Current path
   * @param planner_id The planner ID used to generate the path
   * @return bool If path is valid
   */
  template<typename T>
  bool validatePath(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    const geometry_msgs::msg::PoseStamped & curr_goal,
    const nav_msgs::msg::Path & path,
    const std::string & planner_id);

  // ---- 动作服务器（Action Server）成员 ----
  // 对 rclcpp_action 服务器的高层封装（nav2_util::SimpleActionServer），
  // 内部处理 handle_goal/handle_cancel/handle_accepted，并在独立线程执行
  // 传入的执行回调（此处分别绑定 computePlan / computePlanThroughPoses）。
  // Our action server implements the ComputePathToPose action
  std::unique_ptr<ActionServerToPose> action_server_pose_;
  std::unique_ptr<ActionServerThroughPoses> action_server_poses_;

  /**
   * @brief The action server callback which calls planner to get the path
   * ComputePathToPose
   */
  void computePlan();

  /**
   * @brief The action server callback which calls planner to get the path
   * ComputePathThroughPoses
   */
  void computePlanThroughPoses();

  /**
   * @brief The service callback to determine if the path is still valid
   * @param request to the service
   * @param response from the service
   */
  void isPathValid(
    const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
    std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response);

  /**
   * @brief Publish a path for visualization purposes
   * @param path Reference to Global Path
   */
  void publishPlan(const nav_msgs::msg::Path & path);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // ---- 动态参数成员 ----
  // dyn_params_handler_：on_activate 中通过 add_on_set_parameters_callback
  // 注册的回调句柄；on_deactivate 时 reset() 以注销回调。
  // dynamic_params_lock_：保护动态参数与规划线程之间的共享数据。
  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  // ---- 规划器插件成员 ----
  // planners_：planner_id → 插件实例 的哈希表（on_configure 时填充）；
  // gp_loader_：pluginlib 类加载器（基接口 nav2_core::GlobalPlanner），
  //             通过 createUniqueInstance(<package>/<class>) 从插件库
  //             动态构造规划器实例；
  // default_ids_/default_types_：默认插件名与类型（GridBased → Navfn），
  //             用于在参数未设置时生成默认参数；
  // planner_ids_/planner_types_：实际使用的插件名/类型列表；
  // max_planner_duration_：期望规划周期的倒数（用于超时告警）；
  // planner_ids_concat_：所有插件名的拼接字符串（仅用于日志提示）。
  // Planner
  PlannerMap planners_;
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> gp_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> planner_ids_;
  std::vector<std::string> planner_types_;
  double max_planner_duration_;
  std::string planner_ids_concat_;

  // ---- TF 缓冲区 ----
  // 由全局 Costmap 提供的 tf2 缓冲（含 TransformListener），
  // 供坐标变换（transformPoseToGlobalFrame 等）查询使用。
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // ---- 全局 Costmap 相关 ----
  // costmap_ros_：全局代价地图（生命周期子节点，命名 global_costmap）；
  // costmap_thread_：专属执行线程，用于跑 Costmap 的回调（如地图订阅、
  //                 定时更新），避免与主节点的执行器相互阻塞；
  // costmap_：Costmap 的底层网格数据指针（引用计数归 costmap_ros_ 所有）；
  // collision_checker_：脚印碰撞检查器，is_path_valid 服务中按非圆形
  //                    机器人轮廓计算路径点碰撞代价时使用。
  // Global Costmap
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;

  // ---- 路径发布器 ----
  // 生命周期发布器：仅 active 状态才真正发布，发布到 "plan" 话题供可视化。
  // Publishers for the path
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

  // Service to deterime if the path is valid
  rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr is_path_valid_service_;
};

}  // namespace nav2_planner

#endif  // NAV2_PLANNER__PLANNER_SERVER_HPP_
