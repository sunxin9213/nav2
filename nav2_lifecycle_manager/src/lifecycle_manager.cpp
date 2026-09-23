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
 * 文件：lifecycle_manager.cpp（nav2_lifecycle_manager 模块）
 * =============================================================================
 * 功能概述：
 *   实现 LifecycleManager——Nav2 的生命周期管理器。
 *   Nav2 的各个核心服务器（map_server、amcl、planner_server、controller_server、
 *   bt_navigator 等）都是生命周期节点（LifecycleNode），其状态机为：
 *     unconfigured --configure--> inactive --activate--> active
 *     active --deactivate--> inactive --cleanup--> unconfigured --shutdown--> finalized
 *   本节点（普通 rclcpp::Node，非生命周期节点）负责按正确的顺序驱动上述转换：
 *     - startup：按 node_names 参数正序 configure + activate 全部受管节点；
 *     - shutdown/reset：逆序 deactivate + cleanup（+ shutdown），保证依赖反向释放；
 *   并通过 Bond 心跳机制监控各受管节点存活状态，一旦失联立即硬重置系统。
 *
 * 对外接口：
 *   - <manager>/manage_nodes 服务（nav2_msgs/srv/ManageLifecycleNodes）：
 *       STARTUP / RESET / SHUTDOWN / PAUSE / RESUME 五种命令；
 *   - <manager>/is_active 服务（std_srvs/srv/Trigger）：查询系统是否整体激活；
 *   - diagnostics 话题：发布 Nav2 健康状态。
 *
 * 在整个导航流程中的位置：
 *   本管理器位于所有导航节点"之下"，是 bringup 的基础——它先于
 *   bt_navigator → planner_server → controller_server → velocity_smoother
 *   任务链启动，负责把整条链上的节点带入 active 状态并持续守护。
 * =============================================================================
 */

#include "nav2_lifecycle_manager/lifecycle_manager.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using lifecycle_msgs::msg::Transition;
using lifecycle_msgs::msg::State;
using nav2_util::LifecycleServiceClient;

namespace nav2_lifecycle_manager
{

// 创建普通 ROS 2 管理节点，声明 autostart、受管节点列表和 bond 重连参数，并创建管理服务。
LifecycleManager::LifecycleManager(const rclcpp::NodeOptions & options)
: Node("lifecycle_manager", options), diagnostics_updater_(this)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // The list of names is parameterized, allowing this module to be used with a different set
  // of nodes
  declare_parameter("node_names", rclcpp::PARAMETER_STRING_ARRAY);
  // autostart=false 时等待外部 /manage_nodes 请求；true 时初始化完成后自动启动全部节点。
  declare_parameter("autostart", rclcpp::ParameterValue(false));
  // bond_timeout 和重连时长单位为秒，用于判断生命周期节点是否仍存活以及是否尝试恢复。
  declare_parameter("bond_timeout", 4.0);
  declare_parameter("bond_respawn_max_duration", 10.0);
  declare_parameter("attempt_respawn_reconnection", true);

  registerRclPreshutdownCallback();

  node_names_ = get_parameter("node_names").as_string_array();
  get_parameter("autostart", autostart_);
  double bond_timeout_s;
  get_parameter("bond_timeout", bond_timeout_s);
  bond_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(bond_timeout_s));

  double respawn_timeout_s;
  get_parameter("bond_respawn_max_duration", respawn_timeout_s);
  bond_respawn_max_duration_ = rclcpp::Duration::from_seconds(respawn_timeout_s);

  get_parameter("attempt_respawn_reconnection", attempt_respawn_reconnection_);

  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  manager_srv_ = create_service<ManageLifecycleNodes>(
    get_name() + std::string("/manage_nodes"),
    std::bind(&LifecycleManager::managerCallback, this, _1, _2, _3),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group_);

  is_active_srv_ = create_service<std_srvs::srv::Trigger>(
    get_name() + std::string("/is_active"),
    std::bind(&LifecycleManager::isActiveCallback, this, _1, _2, _3),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group_);

  transition_state_map_[Transition::TRANSITION_CONFIGURE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_CLEANUP] = State::PRIMARY_STATE_UNCONFIGURED;
  transition_state_map_[Transition::TRANSITION_ACTIVATE] = State::PRIMARY_STATE_ACTIVE;
  transition_state_map_[Transition::TRANSITION_DEACTIVATE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
    State::PRIMARY_STATE_FINALIZED;

  transition_label_map_[Transition::TRANSITION_CONFIGURE] = std::string("Configuring ");
  transition_label_map_[Transition::TRANSITION_CLEANUP] = std::string("Cleaning up ");
  transition_label_map_[Transition::TRANSITION_ACTIVATE] = std::string("Activating ");
  transition_label_map_[Transition::TRANSITION_DEACTIVATE] = std::string("Deactivating ");
  transition_label_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
    std::string("Shutting down ");

  // 延迟初始化定时器（0s 后触发一次即自取消）：
  // 在 executor 尚未 spin 之前直接创建服务客户端/线程并不安全，
  // 故用定时器把 createLifecycleServiceClients / autostart 的 startup()
  // 以及服务回调组的专属执行线程的建立推迟到事件循环启动之后。
  init_timer_ = this->create_wall_timer(
    0s,
    [this]() -> void {
      init_timer_->cancel();
      createLifecycleServiceClients();
      if (autostart_) {
        init_timer_ = this->create_wall_timer(
          0s,
          [this]() -> void {
            init_timer_->cancel();
            startup();
          },
          callback_group_);
      }
      auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor->add_callback_group(callback_group_, get_node_base_interface());
      service_thread_ = std::make_unique<nav2_util::NodeThread>(executor);
    });
  diagnostics_updater_.setHardwareID("Nav2");
  diagnostics_updater_.add("Nav2 Health", this, &LifecycleManager::CreateActiveDiagnostic);
}

// 析构函数：停掉专用的服务回调线程，防止析构期间回调仍访问已销毁成员。
LifecycleManager::~LifecycleManager()
{
  RCLCPP_INFO(get_logger(), "Destroying %s", get_name());
  service_thread_.reset();
}

// 将 STARTUP/RESET/SHUTDOWN/PAUSE/RESUME 服务命令分派到对应生命周期操作。
void
LifecycleManager::managerCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ManageLifecycleNodes::Request> request,
  std::shared_ptr<ManageLifecycleNodes::Response> response)
{
  switch (request->command) {
    case ManageLifecycleNodes::Request::STARTUP:
      response->success = startup();
      break;
    case ManageLifecycleNodes::Request::RESET:
      response->success = reset();
      break;
    case ManageLifecycleNodes::Request::SHUTDOWN:
      response->success = shutdown();
      break;
    case ManageLifecycleNodes::Request::PAUSE:
      response->success = pause();
      break;
    case ManageLifecycleNodes::Request::RESUME:
      response->success = resume();
      break;
  }
}

// 返回受管节点系统是否整体处于 active 状态，供监控或上层节点快速查询。
void
LifecycleManager::isActiveCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = system_active_;
}

// diagnostics（诊断）回调：把 system_active_ 映射为诊断消息。
// system_active_ 为 true 时报告 OK（Nav2 运行中），否则报告 ERROR，
// 便于 robot_upstart / 监控系统通过 /diagnostics 话题掌握导航健康状态。
void
LifecycleManager::CreateActiveDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (system_active_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Nav2 is active");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Nav2 is inactive");
  }
}

// 创建每个受管节点的生命周期服务客户端，后续通过这些客户端发送状态转换。
void
LifecycleManager::createLifecycleServiceClients()
{
  message("Creating and initializing lifecycle service clients");
  for (auto & node_name : node_names_) {
    node_map_[node_name] =
      std::make_shared<LifecycleServiceClient>(node_name, shared_from_this());
  }
}

// 销毁所有节点的生命周期服务客户端，释放与各受管节点通信所需的资源。
// 在 shutdown 流程末尾调用，之后若要重新启动系统需先重建客户端。
void
LifecycleManager::destroyLifecycleServiceClients()
{
  message("Destroying lifecycle service clients");
  for (auto & kv : node_map_) {
    kv.second.reset();
  }
}

// 为 active 节点创建 bond 心跳连接；4 秒默认超时用于发现节点失联。
bool
LifecycleManager::createBondConnection(const std::string & node_name)
{
  const double timeout_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(bond_timeout_).count();
  const double timeout_s = timeout_ns / 1e9;

  if (bond_map_.find(node_name) == bond_map_.end() && bond_timeout_.count() > 0.0) {
    bond_map_[node_name] =
      std::make_shared<bond::Bond>("bond", node_name, shared_from_this());
    bond_map_[node_name]->setHeartbeatTimeout(timeout_s);
    bond_map_[node_name]->setHeartbeatPeriod(0.10);
    bond_map_[node_name]->start();
    if (
      !bond_map_[node_name]->waitUntilFormed(
        rclcpp::Duration(rclcpp::Duration::from_nanoseconds(timeout_ns / 2))))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Server %s was unable to be reached after %0.2fs by bond. "
        "This server may be misconfigured.",
        node_name.c_str(), timeout_s);
      return false;
    }
    RCLCPP_INFO(get_logger(), "Server %s connected with bond.", node_name.c_str());
  }

  return true;
}

// 向单个节点发送 lifecycle transition，并校验其最终 primary state；激活时建立 bond。
bool
LifecycleManager::changeStateForNode(const std::string & node_name, std::uint8_t transition)
{
  message(transition_label_map_[transition] + node_name);

  if (!node_map_[node_name]->change_state(transition) ||
    !(node_map_[node_name]->get_state() == transition_state_map_[transition]))
  {
    RCLCPP_ERROR(get_logger(), "Failed to change state for node: %s", node_name.c_str());
    return false;
  }

  if (transition == Transition::TRANSITION_ACTIVATE) {
    return createBondConnection(node_name);
  } else if (transition == Transition::TRANSITION_DEACTIVATE) {
    bond_map_.erase(node_name);
  }

  return true;
}

// 批量改变节点状态；configure/activate 按正序，deactivate/cleanup/shutdown 按逆序执行。
bool
LifecycleManager::changeStateForAllNodes(std::uint8_t transition, bool hard_change)
{
  // Hard change will continue even if a node fails
  if (transition == Transition::TRANSITION_CONFIGURE ||
    transition == Transition::TRANSITION_ACTIVATE)
  {
    for (auto & node_name : node_names_) {
      try {
        if (!changeStateForNode(node_name, transition) && !hard_change) {
          return false;
        }
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to change state for node: %s. Exception: %s.", node_name.c_str(), e.what());
        return false;
      }
    }
  } else {
    std::vector<std::string>::reverse_iterator rit;
    for (rit = node_names_.rbegin(); rit != node_names_.rend(); ++rit) {
      try {
        if (!changeStateForNode(*rit, transition) && !hard_change) {
          return false;
        }
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to change state for node: %s. Exception: %s.", (*rit).c_str(), e.what());
        return false;
      }
    }
  }
  return true;
}

// 依次停用、清理和关闭全部受管节点，保证依赖关系按反向顺序释放。
void
LifecycleManager::shutdownAllNodes()
{
  message("Deactivate, cleanup, and shutdown nodes");
  changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE);
  changeStateForAllNodes(Transition::TRANSITION_CLEANUP);
  changeStateForAllNodes(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
}

// 执行 configure + activate，成功后标记系统 active 并启动 bond 检查定时器。
bool
LifecycleManager::startup()
{
  message("Starting managed nodes bringup...");
  if (!changeStateForAllNodes(Transition::TRANSITION_CONFIGURE) ||
    !changeStateForAllNodes(Transition::TRANSITION_ACTIVATE))
  {
    RCLCPP_ERROR(get_logger(), "Failed to bring up all requested nodes. Aborting bringup.");
    return false;
  }
  message("Managed nodes are active");
  system_active_ = true;
  createBondTimer();
  return true;
}

// 停止 bond 监控并逆序关闭所有受管节点，最后清理客户端和系统状态。
bool
LifecycleManager::shutdown()
{
  system_active_ = false;
  destroyBondTimer();

  message("Shutting down managed nodes...");
  shutdownAllNodes();
  destroyLifecycleServiceClients();
  message("Managed nodes have been shut down");
  return true;
}

// 重置系统：先停用（deactivate）再清理（cleanup）所有受管节点，
// 使它们回到 unconfigured 状态（可再次 configure/activate）。
// hard_reset=true 时即使个别节点转换失败也继续处理其余节点，
// 用于异常恢复场景（如 bond 失联后的强制重置）。
bool
LifecycleManager::reset(bool hard_reset)
{
  system_active_ = false;
  destroyBondTimer();

  message("Resetting managed nodes...");
  // Should transition in reverse order
  if (!changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE, hard_reset) ||
    !changeStateForAllNodes(Transition::TRANSITION_CLEANUP, hard_reset))
  {
    if (!hard_reset) {
      RCLCPP_ERROR(get_logger(), "Failed to reset nodes: aborting reset");
      return false;
    }
  }

  message("Managed nodes have been reset");
  return true;
}

// 暂停系统：停用（deactivate）所有受管节点并停止 bond 监控。
// 与 shutdown 的区别：节点仍保留在 inactive 状态（已配置），
// 之后可调用 resume() 快速重新激活，无需重新 configure。
bool
LifecycleManager::pause()
{
  system_active_ = false;
  destroyBondTimer();

  message("Pausing managed nodes...");
  if (!changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE)) {
    RCLCPP_ERROR(get_logger(), "Failed to pause nodes: aborting pause");
    return false;
  }

  message("Managed nodes have been paused");
  return true;
}

// 恢复系统：与 pause() 相对，激活（activate）所有受管节点、
// 重启 bond 心跳监控并标记系统为 active。
bool
LifecycleManager::resume()
{
  message("Resuming managed nodes...");
  if (!changeStateForAllNodes(Transition::TRANSITION_ACTIVATE)) {
    RCLCPP_ERROR(get_logger(), "Failed to resume nodes: aborting resume");
    return false;
  }

  message("Managed nodes are active");
  system_active_ = true;
  createBondTimer();
  return true;
}

// 创建 bond 心跳检查定时器（200ms 周期，跑在独立回调组上）。
// bond_timeout_ <= 0 表示用户禁用了 bond 监控，直接返回不创建。
void
LifecycleManager::createBondTimer()
{
  if (bond_timeout_.count() <= 0) {
    return;
  }

  message("Creating bond timer...");
  bond_timer_ = this->create_wall_timer(
    200ms,
    std::bind(&LifecycleManager::checkBondConnections, this),
    callback_group_);
}

// 销毁 bond 心跳检查定时器：先 cancel 停止触发，再 reset 释放定时器。
void
LifecycleManager::destroyBondTimer()
{
  if (bond_timer_) {
    message("Terminating bond timer...");
    bond_timer_->cancel();
    bond_timer_.reset();
  }
}

// rcl 预关机回调：在 ROS 2 context 正式关闭之前执行。
// 目的是在 rclcpp 全局 shutdown 时序下抢先清理 bond 与服务线程，
// 避免 Costmap 等子系统的 preshutdown 回调乱序导致崩溃
// （rclcpp 的 preshutdown 回调无严格顺序保证，见 rclcpp#2096）。
void
LifecycleManager::onRclPreshutdown()
{
  RCLCPP_INFO(
    get_logger(), "Running Nav2 LifecycleManager rcl preshutdown (%s)",
    this->get_name());

  destroyBondTimer();

  /*
   * Dropping the bond map is what we really need here, but we drop the others
   * to prevent the bond map being used. Likewise, squash the service thread.
   */
  service_thread_.reset();
  node_names_.clear();
  node_map_.clear();
  bond_map_.clear();
}

// 向 ROS 2 context 注册预关机回调（绑定到 onRclPreshutdown）。
// 在构造函数中调用，确保进程退出时清理动作先于节点/客户端销毁。
void
LifecycleManager::registerRclPreshutdownCallback()
{
  rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();

  context->add_pre_shutdown_callback(
    std::bind(&LifecycleManager::onRclPreshutdown, this)
  );
}

// bond 心跳监控回调（200ms 周期）：
// 1) 系统未激活、context 已关闭或无 bond 连接时直接返回；
// 2) 遍历所有受管节点的 bond，一旦发现某个心跳中断（isBroken）：
//    - 判定为"关键失败"，立即硬重置（hard reset）整个系统，
//      把仍存活的节点全部转入 inactive，避免部分节点带病运行；
//    - 手动清空 bond 表（崩溃的节点无法通过正常转换清理）；
//    - 若启用 attempt_respawn_reconnection，则启动 1s 周期的
//      respawn 定时器，等待崩溃的服务器被重新拉起。
void
LifecycleManager::checkBondConnections()
{
  if (!system_active_ || !rclcpp::ok() || bond_map_.empty()) {
    return;
  }

  for (auto & node_name : node_names_) {
    if (!rclcpp::ok()) {
      return;
    }

    if (bond_map_[node_name]->isBroken()) {
      message(
        std::string(
          "Have not received a heartbeat from " + node_name + "."));

      // if one is down, bring them all down
      RCLCPP_ERROR(
        get_logger(),
        "CRITICAL FAILURE: SERVER %s IS DOWN after not receiving a heartbeat for %i ms."
        " Shutting down related nodes.",
        node_name.c_str(), static_cast<int>(bond_timeout_.count()));
      reset(true);  // hard reset to transition all still active down
      // if a server crashed, it won't get cleared due to failed transition, clear manually
      bond_map_.clear();

      // Initialize the bond respawn timer to check if server comes back online
      // after a failure, within a maximum timeout period.
      if (attempt_respawn_reconnection_) {
        bond_respawn_timer_ = this->create_wall_timer(
          1s,
          std::bind(&LifecycleManager::checkBondRespawnConnection, this),
          callback_group_);
      }
      return;
    }
  }
}

// respawn（复活重连）检查回调（1s 周期，bond 失败后启动）：
// 用于崩溃的受管节点（如被 Supervisor/systemd 重启的服务器进程）恢复场景：
// 1) 首次触发时记录起始时间，限制最长重连等待 bond_respawn_max_duration_；
// 2) 若期间系统被外部重新激活，则取消重连定时器退出；
// 3) 逐个调用 get_state() 探测各节点是否存活（只有服务在才会正常返回）；
// 4) 全部节点恢复 → 重新执行 startup() 完成系统启动；
//    超时仍未恢复 → 放弃重连，记录失败日志并停止定时器。
void
LifecycleManager::checkBondRespawnConnection()
{
  // First attempt in respawn, start maximum duration to respawn
  if (bond_respawn_start_time_.nanoseconds() == 0) {
    bond_respawn_start_time_ = now();
  }

  // Note: system_active_ is inverted since this should be in a failure
  // condition. If another outside user actives the system again, this should not process.
  if (system_active_ || !rclcpp::ok() || node_names_.empty()) {
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
    return;
  }

  // Check number of live connections after a bond failure
  int live_servers = 0;
  const int max_live_servers = node_names_.size();
  for (auto & node_name : node_names_) {
    if (!rclcpp::ok()) {
      return;
    }

    try {
      node_map_[node_name]->get_state();  // Only won't throw if the server exists
      live_servers++;
    } catch (...) {
      break;
    }
  }

  // If all are alive, kill timer and retransition system to active
  // Else, check if maximum timeout has occurred
  if (live_servers == max_live_servers) {
    message("Successfully re-established connections from server respawns, starting back up.");
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
    startup();
  } else if (now() - bond_respawn_start_time_ >= bond_respawn_max_duration_) {
    message("Failed to re-establish connection from a server crash after maximum timeout.");
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
  }
}

#define ANSI_COLOR_RESET    "\x1b[0m"
#define ANSI_COLOR_BLUE     "\x1b[34m"

// 以蓝色加粗的终端样式输出管理器日志，
// 使生命周期管理器的重要消息在启动日志中易于辨识。
void
LifecycleManager::message(const std::string & msg)
{
  RCLCPP_INFO(get_logger(), ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());
}

}  // namespace nav2_lifecycle_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_lifecycle_manager::LifecycleManager)
