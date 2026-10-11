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

// ============================================================================
// 【文件概况】轻量 Action 服务器模板（nav2 全部服务器的基础设施）
// ----------------------------------------------------------------------------
// 概述: 对 rclcpp_action::Server 的封装，约定"单活跃目标 + 单抢占槽"模型:
//       同一时刻只执行一个 goal；新 goal 到来时放入 pending 槽，
//       当前执行结束后自动切换（或由用户通过 accept_pending_goal 切换）。
// 主要内容:
//   1. 构造: 绑定 execute_callback（业务逻辑）与 completion_callback
//            （异常终止通知）；可选 spin_thread 用独立线程驱动回调
//   2. 生命周期: activate()/deactivate()（后者会等待执行线程退出，
//               超过 server_timeout 则强制 terminate_all）
//   3. goal 管理: get_current_goal/accept_pending_goal/terminate_current/
//                succeeded_current/is_cancel_requested/publish_feedback 等
//   4. work(): 后台循环调用 execute_callback，统一处理异常与抢占切换
// 线程模型: handle_* 回调运行在（外部或内部）executor 线程，
//           execute_callback 运行在 std::async 派生的工作线程，
//           两线程间以 recursive_mutex 保护句柄状态。
// 使用场景: planner_server、controller_server、smoother_server、
//           behavior_server、waypoint_follower 等 nav2 全部 action 服务器。
// ============================================================================

#ifndef NAV2_UTIL__SIMPLE_ACTION_SERVER_HPP_
#define NAV2_UTIL__SIMPLE_ACTION_SERVER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <future>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_util/node_thread.hpp"

namespace nav2_util
{

/**
 * @class nav2_util::SimpleActionServer
 * @brief An action server wrapper to make applications simpler using Actions
 */
template<typename ActionT>
class SimpleActionServer
{
public:
  // Callback function to complete main work. This should itself deal with its
  // own exceptions, but if for some reason one is thrown, it will be caught
  // in SimpleActionServer and terminate the action itself.
  typedef std::function<void ()> ExecuteCallback;

  // Callback function to notify the user that an exception was thrown that
  // the simple action server caught (or another failure) and the action was
  // terminated. To avoid using, catch exceptions in your application such that
  // the SimpleActionServer will never need to terminate based on failed action
  // ExecuteCallback.
  typedef std::function<void ()> CompletionCallback;

  /**
   * @brief An constructor for SimpleActionServer
   * @param node Ptr to node to make actions
   * @param action_name Name of the action to call
   * @param execute_callback Execution  callback function of Action
   * @param server_timeout Timeout to to react to stop or preemption requests
   * @param spin_thread Whether to spin with a dedicated thread internally
   * @param options Options to pass to the underlying rcl_action_server_t
   */
  template<typename NodeT>
  // 使用完整 ROS 2 节点接口创建 Action Server；默认不单独开线程，由外部 executor 驱动回调。
  explicit SimpleActionServer(
    NodeT node,
    const std::string & action_name,
    ExecuteCallback execute_callback,
    CompletionCallback completion_callback = nullptr,
    std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500),
    bool spin_thread = false,
    const rcl_action_server_options_t & options = rcl_action_server_get_default_options())
  : SimpleActionServer(
      node->get_node_base_interface(),
      node->get_node_clock_interface(),
      node->get_node_logging_interface(),
      node->get_node_waitables_interface(),
      action_name, execute_callback, completion_callback, server_timeout, spin_thread, options)
  {}

  /**
   * @brief An constructor for SimpleActionServer
   * @param <node interfaces> Abstract node interfaces to make actions
   * @param action_name Name of the action to call
   * @param execute_callback Execution  callback function of Action
   * @param server_timeout Timeout to to react to stop or preemption requests
   * @param spin_thread Whether to spin with a dedicated thread internally
   * @param options Options to pass to the underlying rcl_action_server_t
   */
  explicit SimpleActionServer(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
    const std::string & action_name,
    ExecuteCallback execute_callback,
    CompletionCallback completion_callback = nullptr,
    std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500),
    bool spin_thread = false,
    const rcl_action_server_options_t & options = rcl_action_server_get_default_options())
  : node_base_interface_(node_base_interface),
    node_clock_interface_(node_clock_interface),
    node_logging_interface_(node_logging_interface),
    node_waitables_interface_(node_waitables_interface),
    action_name_(action_name),
    execute_callback_(execute_callback),
    completion_callback_(completion_callback),
    server_timeout_(server_timeout),
    spin_thread_(spin_thread)
  {
    // server_timeout 默认 500 ms，用于等待取消/抢占处理；spin_thread=true 时创建独立单线程 executor。
    using namespace std::placeholders;  // NOLINT
    if (spin_thread_) {
      callback_group_ = node_base_interface->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    }
    action_server_ = rclcpp_action::create_server<ActionT>(
      node_base_interface_,
      node_clock_interface_,
      node_logging_interface_,
      node_waitables_interface_,
      action_name_,
      std::bind(&SimpleActionServer::handle_goal, this, _1, _2),
      std::bind(&SimpleActionServer::handle_cancel, this, _1),
      std::bind(&SimpleActionServer::handle_accepted, this, _1),
      options,
      callback_group_);
    if (spin_thread_) {
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_callback_group(callback_group_, node_base_interface_);
      executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);
    }
  }

  /**
   * @brief handle the goal requested: accept or reject. This implementation always accepts.
   * @param uuid Goal ID
   * @param Goal A shared pointer to the specific goal
   * @return GoalResponse response of the goal processed
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const typename ActionT::Goal>/*goal*/)
  {
    // 非 active 状态拒绝 goal，保证生命周期节点不会在未激活时执行动作。
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!server_active_) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    debug_msg("Received request for goal acceptance");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief Accepts cancellation requests of action server.
   * @param uuid Goal ID
   * @param Goal A server goal handle to cancel
   * @return CancelResponse response of the goal cancelled
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
  {
    // 只有仍 active 的 goal 才接受取消请求，避免对已结束句柄重复操作。
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!handle->is_active()) {
      warn_msg(
        "Received request for goal cancellation,"
        "but the handle is inactive, so reject the request");
      return rclcpp_action::CancelResponse::REJECT;
    }

    debug_msg("Received request for goal cancellation");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief Handles accepted goals and adds to preempted queue to switch to
   * @param Goal A server goal handle to cancel
   */
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
  {
    // 新 goal 到来时放入 pending；当前 goal 结束后再切换，避免并发执行两个动作。
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    debug_msg("Receiving a new goal");

    if (is_active(current_handle_) || is_running()) {
      debug_msg("An older goal is active, moving the new goal to a pending slot.");

      if (is_active(pending_handle_)) {
        debug_msg(
          "The pending slot is occupied."
          " The previous pending goal will be terminated and replaced.");
        terminate(pending_handle_);
      }
      pending_handle_ = handle;
      preempt_requested_ = true;
    } else {
      if (is_active(pending_handle_)) {
        // Shouldn't reach a state with a pending goal but no current one.
        error_msg("Forgot to handle a preemption. Terminating the pending goal.");
        terminate(pending_handle_);
        preempt_requested_ = false;
      }

      current_handle_ = handle;

      // Return quickly to avoid blocking the executor, so spin up a new thread
      debug_msg("Executing goal asynchronously.");
      execution_future_ = std::async(std::launch::async, [this]() {work();});
    }
  }

  /**
   * @brief Computed background work and processes stop requests
   */
  void work()
  {
    // 后台循环反复调用业务 execute_callback，直到取消、抢占、异常或 ROS 2 关闭。
    while (rclcpp::ok() && !stop_execution_ && is_active(current_handle_)) {
      debug_msg("Executing the goal...");
      try {
        execute_callback_();
      } catch (std::exception & ex) {
        RCLCPP_ERROR(
          node_logging_interface_->get_logger(),
          "Action server failed while executing action callback: \"%s\"", ex.what());
        terminate_all();
        completion_callback_();
        return;
      }

      debug_msg("Blocking processing of new goal handles.");
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);

      if (stop_execution_) {
        warn_msg("Stopping the thread per request.");
        terminate_all();
        completion_callback_();
        break;
      }

      if (is_active(current_handle_)) {
        warn_msg("Current goal was not completed successfully.");
        terminate(current_handle_);
        completion_callback_();
      }

      if (is_active(pending_handle_)) {
        debug_msg("Executing a pending handle on the existing thread.");
        accept_pending_goal();
      } else {
        debug_msg("Done processing available goals.");
        break;
      }
    }
    debug_msg("Worker thread done.");
  }

  /**
   * @brief Active action server
   */
  // 激活服务器: 开始接受新 goal 并清除停止标志（对应生命周期 on_activate）
  void activate()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    server_active_ = true;
    stop_execution_ = false;
  }

  /**
   * @brief Deactive action server
   */
  void deactivate()
  {
    debug_msg("Deactivating...");

    {
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);
      server_active_ = false;
      stop_execution_ = true;
    }

    if (!execution_future_.valid()) {
      return;
    }

    if (is_running()) {
      warn_msg(
        "Requested to deactivate server but goal is still executing."
        " Should check if action server is running before deactivating.");
    }

    using namespace std::chrono;  //NOLINT
    auto start_time = steady_clock::now();
    while (execution_future_.wait_for(milliseconds(100)) != std::future_status::ready) {
      info_msg("Waiting for async process to finish.");
      if (steady_clock::now() - start_time >= server_timeout_) {
        terminate_all();
        if (completion_callback_) {completion_callback_();}
        error_msg("Action callback is still running and missed deadline to stop");
      }
    }

    debug_msg("Deactivation completed.");
  }

  /**
   * @brief Whether the action server is munching on a goal
   * @return bool If its running or not
   */
  // 后台工作线程是否仍在执行 goal（future 尚未就绪即视为仍在运行）
  bool is_running()
  {
    return execution_future_.valid() &&
           (execution_future_.wait_for(std::chrono::milliseconds(0)) ==
           std::future_status::timeout);
  }

  /**
   * @brief Whether the action server is active or not
   * @return bool If its active or not
   */
  // 服务器是否处于激活状态（由 activate/deactivate 切换）
  bool is_server_active()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    return server_active_;
  }

  /**
   * @brief Whether the action server has been asked to be preempted with a new goal
   * @return bool If there's a preemption request or not
   */
  // 是否有新 goal 正在等待切换（pending 槽被占用）
  bool is_preempt_requested() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    return preempt_requested_;
  }

  /**
   * @brief Accept pending goals
   * @return Goal Ptr to the  goal that's going to be accepted
   */
  // 接受 pending 槽中的新 goal: 终止当前 goal、切换句柄并返回新 goal
  const std::shared_ptr<const typename ActionT::Goal> accept_pending_goal()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!pending_handle_ || !pending_handle_->is_active()) {
      error_msg("Attempting to get pending goal when not available");
      return std::shared_ptr<const typename ActionT::Goal>();
    }

    if (is_active(current_handle_) && current_handle_ != pending_handle_) {
      debug_msg("Cancelling the previous goal");
      current_handle_->abort(empty_result());
    }

    current_handle_ = pending_handle_;
    pending_handle_.reset();
    preempt_requested_ = false;

    debug_msg("Preempted goal");

    return current_handle_->get_goal();
  }

  /**
   * @brief Terminate pending goals
   */
  // 终止 pending 槽中的 goal（不切换执行）
  void terminate_pending_goal()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!pending_handle_ || !pending_handle_->is_active()) {
      error_msg("Attempting to terminate pending goal when not available");
      return;
    }

    terminate(pending_handle_);
    preempt_requested_ = false;

    debug_msg("Pending goal terminated");
  }

  /**
   * @brief Get the current goal object
   * @return Goal Ptr to the  goal that's being processed currently
   */
  // 获取当前正在执行的 goal（无活跃 goal 时返回空指针）
  const std::shared_ptr<const typename ActionT::Goal> get_current_goal() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!is_active(current_handle_)) {
      error_msg("A goal is not available or has reached a final state");
      return std::shared_ptr<const typename ActionT::Goal>();
    }

    return current_handle_->get_goal();
  }

  // 获取当前 goal 的 UUID（无活跃 goal 时返回空 UUID）
  const rclcpp_action::GoalUUID get_current_goal_id() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!is_active(current_handle_)) {
      error_msg("A goal is not available or has reached a final state");
      return rclcpp_action::GoalUUID();
    }

    return current_handle_->get_goal_id();
  }

  /**
   * @brief Get the pending goal object
   * @return Goal Ptr to the goal that's pending
   */
  // 获取 pending 槽中的 goal（无则返回空指针）
  const std::shared_ptr<const typename ActionT::Goal> get_pending_goal() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!pending_handle_ || !pending_handle_->is_active()) {
      error_msg("Attempting to get pending goal when not available");
      return std::shared_ptr<const typename ActionT::Goal>();
    }

    return pending_handle_->get_goal();
  }

  /**
   * @brief Whether or not a cancel command has come in
   * @return bool Whether a cancel command has been requested or not
   */
  // 客户端是否请求了取消: 优先检查 pending 句柄，其次检查当前句柄
  bool is_cancel_requested() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    // A cancel request is assumed if either handle is canceled by the client.

    if (current_handle_ == nullptr) {
      error_msg("Checking for cancel but current goal is not available");
      return false;
    }

    if (pending_handle_ != nullptr) {
      return pending_handle_->is_canceling();
    }

    return current_handle_->is_canceling();
  }

  /**
   * @brief Terminate all pending and active actions
   * @param result A result object to send to the terminated actions
   */
  // 终止当前与 pending 的所有 goal（默认以空 Result 结束）
  void terminate_all(
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    terminate(current_handle_, result);
    terminate(pending_handle_, result);
    preempt_requested_ = false;
  }

  /**
   * @brief Terminate the active action
   * @param result A result object to send to the terminated action
   */
  // 仅终止当前 goal（默认以空 Result 结束）
  void terminate_current(
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    terminate(current_handle_, result);
  }

  /**
   * @brief Return success of the active action
   * @param result A result object to send to the terminated actions
   */
  // 将当前 goal 标记为成功并释放句柄（业务逻辑完成时的标准出口）
  void succeeded_current(
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (is_active(current_handle_)) {
      debug_msg("Setting succeed on current goal.");
      current_handle_->succeed(result);
      current_handle_.reset();
    }
  }

  /**
   * @brief Publish feedback to the action server clients
   * @param feedback A feedback object to send to the clients
   */
  // 向客户端发布 feedback（当前句柄不活跃时拒绝发布并报错）
  void publish_feedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)
  {
    if (!is_active(current_handle_)) {
      error_msg("Trying to publish feedback when the current goal handle is not active");
      return;
    }

    current_handle_->publish_feedback(feedback);
  }

protected:
  // The SimpleActionServer isn't itself a node, so it needs interfaces to one
  // 本类不是节点，只借用宿主节点的四类底层接口（创建 action server/日志/时钟用）
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface_;
  std::string action_name_;                  // action 名称（用于日志前缀）

  ExecuteCallback execute_callback_;         // 业务执行回调（由使用方注入）
  CompletionCallback completion_callback_;   // 动作被异常终止时的通知回调（可空）
  std::future<void> execution_future_;       // 工作线程（std::async）的 future
  bool stop_execution_{false};               // 停止标志: work() 循环的退出条件之一

  mutable std::recursive_mutex update_mutex_;  // 保护下列句柄/状态的多线程访问
  bool server_active_{false};                  // 是否接受新 goal
  bool preempt_requested_{false};              // pending 槽是否被占用（有抢占请求）
  std::chrono::milliseconds server_timeout_;   // deactivate 时等待执行线程的最长时间

  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> current_handle_;   // 当前执行中的句柄
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> pending_handle_;   // 等待切换的新句柄

  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;  // 底层 action server
  bool spin_thread_;                                                  // 是否用内部线程驱动回调
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};          // 内部线程模式下的独立回调组
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;     // 内部线程模式的执行器
  std::unique_ptr<nav2_util::NodeThread> executor_thread_;            // 执行器的宿主线程（RAII）

  /**
   * @brief Generate an empty result object for an action type
   */
  // 生成当前动作类型的空 Result 对象（终止 goal 时的默认结果）
  constexpr auto empty_result() const
  {
    return std::make_shared<typename ActionT::Result>();
  }

  /**
   * @brief Whether a given goal handle is currently active
   * @param handle Goal handle to check
   * @return Whether this goal handle is active
   */
  // 句柄非空且仍处于活跃（未终结）状态
  constexpr bool is_active(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle) const
  {
    return handle != nullptr && handle->is_active();
  }

  /**
   * @brief Terminate a particular action with a result
   * @param handle goal handle to terminate
   * @param the Results object to terminate the action with
   */
  // 终止指定句柄: 若客户端请求了取消则按 canceled 结束，否则按 abort 结束
  void terminate(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & handle,
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (is_active(handle)) {
      if (handle->is_canceling()) {
        info_msg("Client requested to cancel the goal. Cancelling.");
        handle->canceled(result);
      } else {
        warn_msg("Aborting handle.");
        handle->abort(result);
      }
      handle.reset();
    }
  }

  /**
   * @brief Info logging
   */
  void info_msg(const std::string & msg) const
  {
    RCLCPP_INFO(
      node_logging_interface_->get_logger(),
      "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }

  /**
   * @brief Debug logging
   */
  void debug_msg(const std::string & msg) const
  {
    RCLCPP_DEBUG(
      node_logging_interface_->get_logger(),
      "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }

  /**
   * @brief Error logging
   */
  void error_msg(const std::string & msg) const
  {
    RCLCPP_ERROR(
      node_logging_interface_->get_logger(),
      "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }

  /**
   * @brief Warn logging
   */
  void warn_msg(const std::string & msg) const
  {
    RCLCPP_WARN(
      node_logging_interface_->get_logger(),
      "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }
};

}  // namespace nav2_util

#endif   // NAV2_UTIL__SIMPLE_ACTION_SERVER_HPP_
