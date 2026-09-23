// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2022 Samsung Research America
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
 * 文件：lifecycle_manager.hpp（nav2_lifecycle_manager 模块）
 * =============================================================================
 * 功能概述：
 *   声明 LifecycleManager 类——Nav2 的生命周期管理器（普通 rclcpp::Node）。
 *   核心职责：
 *   - 状态机编排：按 node_names 参数顺序（configure/activate 正序，
 *     deactivate/cleanup/shutdown 逆序）驱动所有受管生命周期节点；
 *   - Bond 心跳守护：为每个 active 节点建立 bond::Bond 连接，
 *     周期检查心跳，失联时硬重置系统并可选等待节点复活重启；
 *   - 对外服务：manage_nodes（STARTUP/RESET/SHUTDOWN/PAUSE/RESUME 命令）、
 *     is_active（系统是否整体激活）；
 *   - 诊断上报：经 diagnostic_updater 发布 Nav2 健康状态。
 *   关键实现细节：服务回调跑在独立的 MutuallyExclusive 回调组与专属线程
 *   （service_thread_）上，避免耗时生命周期转换阻塞主执行器。
 *   对应实现见 lifecycle_manager.cpp。
 * =============================================================================
 */

#ifndef NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "nav2_util/lifecycle_service_client.hpp"
#include "nav2_util/node_thread.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "bondcpp/bond.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"


namespace nav2_lifecycle_manager
{
using namespace std::chrono_literals;  // NOLINT

using nav2_msgs::srv::ManageLifecycleNodes;
/**
 * @class nav2_lifecycle_manager::LifecycleManager
 * @brief Implements service interface to transition the lifecycle nodes of
 * Nav2 stack. It receives transition request and then uses lifecycle
 * interface to change lifecycle node's state.
 */
class LifecycleManager : public rclcpp::Node
{
public:
  /**
   * @brief A constructor for nav2_lifecycle_manager::LifecycleManager
   * @param options Additional options to control creation of the node.
   */
  // 创建管理节点并声明受管节点、自动启动和 bond 监控参数。
  explicit LifecycleManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for nav2_lifecycle_manager::LifecycleManager
   */
  // 停止服务线程和 bond 监控，释放生命周期客户端。
  ~LifecycleManager();

protected:
  // ---- 回调组与服务线程 ----
  // callback_group_：MutuallyExclusive 回调组，服务/定时器均挂在其上，
  //                 与主执行器分离，保证状态转换期间系统仍可响应；
  // service_thread_：跑上述回调组的专属执行线程（NodeThread）。
  // Callback group used by services and timers
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::unique_ptr<nav2_util::NodeThread> service_thread_;

  // ---- 本节点提供的服务 ----
  // manager_srv_：manage_nodes 服务（生命周期命令分派）；
  // is_active_srv_：is_active 服务（查询系统整体是否激活）。
  // The services provided by this node
  rclcpp::Service<ManageLifecycleNodes>::SharedPtr manager_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr is_active_srv_;
  /**
   * @brief Lifecycle node manager callback function
   * @param request_header Header of the service request
   * @param request Service request
   * @param reponse Service response
   */
  void managerCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManageLifecycleNodes::Request> request,
    std::shared_ptr<ManageLifecycleNodes::Response> response);
  /**
   * @brief Trigger callback function checks if the managed nodes are in active
   * state.
   * @param request_header Header of the request
   * @param request Service request
   * @param reponse Service response
   */
  void isActiveCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Support functions for the service calls
  /**
   * @brief Start up managed nodes.
   * @return true or false
   */
  bool startup();
  /**
   * @brief Deactivate, clean up and shut down all the managed nodes.
   * @return true or false
   */
  bool shutdown();
  /**
   * @brief Reset all the managed nodes.
   * @return true or false
   */
  bool reset(bool hard_reset = false);
  /**
   * @brief Pause all the managed nodes.
   * @return true or false
   */
  bool pause();
  /**
   * @brief Resume all the managed nodes.
   * @return true or false
   */
  bool resume();

  /**
   * @brief Perform preshutdown activities before our Context is shutdown.
   * Note that this is related to our Context's shutdown sequence, not the
   * lifecycle node state machine or shutdown().
   */
  void onRclPreshutdown();

  // Support function for creating service clients
  /**
   * @brief Support function for creating service clients
   */
  void createLifecycleServiceClients();

  // Support functions for shutdown
  /**
   * @brief Support function for shutdown
   */
  void shutdownAllNodes();
  /**
   * @brief Destroy all the lifecycle service clients.
   */
  void destroyLifecycleServiceClients();

  // Support function for creating bond timer
  /**
   * @brief Support function for creating bond timer
   */
  void createBondTimer();

  // Support function for creating bond connection
  /**
   * @brief Support function for creating bond connections
   */
  bool createBondConnection(const std::string & node_name);

  // Support function for killing bond connections
  /**
   * @brief Support function for killing bond connections
   */
  void destroyBondTimer();

  // Support function for checking on bond connections
  /**
   * @ brief Support function for checking on bond connections
   * will take down system if there's something non-responsive
   */
  void checkBondConnections();

  // Support function for checking if bond connections come back after respawn
  /**
   * @ brief Support function for checking on bond connections
   * will bring back the system if something goes from non-responsive to responsive
   */
  void checkBondRespawnConnection();

  /**
   * @brief For a node, transition to the new target state
   */
  bool changeStateForNode(
    const std::string & node_name,
    std::uint8_t transition);

  /**
   * @brief For each node in the map, transition to the new target state
   */
  bool changeStateForAllNodes(std::uint8_t transition, bool hard_change = false);

  // Convenience function to highlight the output on the console
  /**
   * @brief Helper function to highlight the output on the console
   */
  void message(const std::string & msg);

  // Diagnostics functions
  /**
   * @brief function to check if the Nav2 system is active
   */
  void CreateActiveDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * Register our preshutdown callback for this Node's rcl Context.
   * The callback fires before this Node's Context is shutdown.
   * Note this is not directly related to the lifecycle state machine or the
   * shutdown() instance function.
   */
  void registerRclPreshutdownCallback();

  // ---- 定时器与 bond 参数 ----
  // init_timer_：延迟初始化定时器（触发一次后自取消）；
  // bond_timer_：200ms 周期的 bond 心跳检查定时器；
  // bond_respawn_timer_：bond 失败后 1s 周期的复活重连定时器；
  // bond_timeout_：心跳超时阈值（默认 4s，<=0 表示禁用 bond 监控）。
  // Timer thread to look at bond connections
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr bond_timer_;
  rclcpp::TimerBase::SharedPtr bond_respawn_timer_;
  std::chrono::milliseconds bond_timeout_;

  // ---- bond 与节点管理表 ----
  // bond_map_：节点名 → bond::Bond 连接（激活节点的心跳通道）；
  // node_map_：节点名 → LifecycleServiceClient（发送生命周期转换的客户端）；
  // transition_label_map_：转换 ID → 日志标签（如 "Activating "）；
  // transition_state_map_：转换 ID → 预期的最终 primary state，
  //                         用于转换后校验节点是否真正到达目标状态。
  // A map of all nodes to check bond connection
  std::map<std::string, std::shared_ptr<bond::Bond>> bond_map_;

  // A map of all nodes to be controlled
  std::map<std::string, std::shared_ptr<nav2_util::LifecycleServiceClient>> node_map_;

  std::map<std::uint8_t, std::string> transition_label_map_;

  // A map of the expected transitions to primary states
  std::unordered_map<std::uint8_t, std::uint8_t> transition_state_map_;

  // ---- 参数与状态 ----
  // node_names_：受管节点名列表（按期望的启动顺序排列）；
  // autostart_：构造完成后是否自动执行 startup()；
  // attempt_respawn_reconnection_：bond 失败后是否等待崩溃节点复活重连；
  // system_active_：受管系统整体是否处于 active 状态（is_active 依据）；
  // diagnostics_updater_：诊断更新器，发布 Nav2 健康状态；
  // bond_respawn_*：复活重连的起始时间与最大等待时长。
  // The names of the nodes to be managed, in the order of desired bring-up
  std::vector<std::string> node_names_;

  // Whether to automatically start up the system
  bool autostart_;
  bool attempt_respawn_reconnection_;

  bool system_active_{false};
  diagnostic_updater::Updater diagnostics_updater_;

  rclcpp::Time bond_respawn_start_time_{0};
  rclcpp::Duration bond_respawn_max_duration_{10s};
};

}  // namespace nav2_lifecycle_manager

#endif  // NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
