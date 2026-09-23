// Copyright (c) 2018 Intel Corporation
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

// =============================================================================
// 【文件说明】bt_action_node.hpp
// 【所属模块】nav2_behavior_tree —— Nav2 的行为树（Behavior Tree）核心模块
//
// 本文件定义了 BtActionNode<ActionT> 模板类，它是 Nav2 中所有"动作型"
// (Action) 行为树节点的公共抽象基类。
//
// 【在 Nav2 导航系统中的位置和作用】
// Nav2 的导航任务（如 ComputePathToPose、FollowPath、Spin、Wait、BackUp 等）
// 全部以 ROS2 Action 协议（goal 请求 -> feedback 反馈 -> result 结果 的
// 异步三段式通信）对外提供服务。行为树中的动作节点需要扮演"客户端"角色：
//   1. 创建并持有 rclcpp_action::Client<ActionT>（动作客户端）；
//   2. 管理 goal（目标）的异步发送、应答超时、目标更新重发、取消（halt）
//      的完整生命周期；
//   3. 等待结果期间向行为树引擎返回 RUNNING，不阻塞整棵树的执行；
//   4. 通过虚函数 on_tick / on_wait_for_result / on_success / on_aborted /
//      on_cancelled 把"业务差异"下放给子类实现（模板方法设计模式）。
//
// 【BehaviorTree.CPP 核心概念】
// - tick()：行为树引擎每个周期调用一次节点的主函数，必须返回
//   SUCCESS / FAILURE / RUNNING 三种状态之一；RUNNING 表示动作尚未完成，
//   下一周期会继续 tick 同一节点；IDLE 表示节点尚未开始或已被 halt 复位。
// - blackboard（黑板）：行为树的全局键值存储，各节点通过端口(port)读写；
//   本类约定从黑板读取 "node"（共享的 ROS2 节点指针）、"bt_loop_duration"
//   （单次 tick 的时间预算）、"server_timeout"（服务器应答超时）、
//   "wait_for_service_timeout"（等待服务出现的超时）等键。
// - providedPorts()：声明节点在 XML 中可用的输入/输出端口；本基类统一提供
//   server_name / server_timeout 两个输入端口（InputPort），子类通过
//   providedBasicPorts() 在此基础上追加自己的端口。
// - halt()：当父节点（如 RecoveryNode、PipelineSequence）决定放弃本节点时
//   被调用，用于清理（此处是取消正在执行的 ROS2 action goal）并把状态
//   复位为 IDLE。
//
// 【XML 使用示例】（以子类 FollowPath 为例，一行）
//   <FollowPath path="{path}" controller_id="FollowPath"/>
// 其中 {path} 表示从黑板键 "path" 读取输入；controller_id 映射到 goal 字段。
//
// 【线程模型】
// 每个节点创建自己的回调组(callback group) + 单线程执行器
// (SingleThreadedExecutor)，在 tick() 中通过 spin_some()/spin_until_future_
// complete() 主动泵取 action 客户端的回调，避免与行为树主执行器产生竞争。
// =============================================================================

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_util/node_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace nav2_behavior_tree
{

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief Abstract class representing an action based BT node
 *        （表示"基于 ROS2 Action 的行为树节点"的抽象基类）
 * @tparam ActionT Type of action
 *        （模板参数：该节点要调用的 ROS2 Action 类型，如 nav2_msgs::action::FollowPath）
 */
template<class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::BtActionNode constructor
   *        （BtActionNode 构造函数：初始化动作客户端及各类超时参数）
   * @param xml_tag_name Name for the XML tag for this node
   *        （该节点在行为树 XML 文件中使用的标签名，如 "FollowPath"）
   * @param action_name Action name this node creates a client for
   *        （要连接的 ROS2 Action 服务名，如 "follow_path"）
   * @param conf BT node configuration
   *        （BT 节点配置，内含 blackboard 黑板指针和端口映射）
   */
  BtActionNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name), should_send_goal_(true)
  {
    // 从黑板获取共享的 ROS2 节点指针（由 BtActionServer 在初始化时放入黑板），
    // 后续所有 ROS2 通信（创建客户端、打日志、取参数）都基于这个节点。
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    // 为本节点创建一个互斥型回调组（同一时刻组内只允许一个回调执行），
    // 第二个参数 false 表示不自动加入节点的默认执行器——
    // 这样主执行器 spin 时不会并发触发本节点的回调，避免数据竞争。
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    // 把回调组挂到本节点私有的单线程执行器上，
    // 之后在 tick() 里手动调用 spin_some()/spin_until_future_complete() 泵取回调。
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // Get the required items from the blackboard
    // 从黑板读取行为树引擎预设的超时配置：
    // bt_loop_duration = 单次 tick 的最大时长预算（毫秒）
    auto bt_loop_duration =
      config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
    // server_timeout = 等待 action 服务器应答的超时（毫秒）；
    // 若 XML 中给该节点的 server_timeout 端口传了值，则优先使用端口值。
    server_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
    // wait_for_service_timeout = 构造时等待 action 服务器上线的时间上限。
    wait_for_service_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("wait_for_service_timeout");

    // timeout should be less than bt_loop_duration to be able to finish the current tick
    // 单次同步等待的最大时长取 tick 预算的一半，
    // 保证一次 tick 不会把整个 bt_loop_duration 耗光，树还有余量调度其他节点。
    max_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(bt_loop_duration * 0.5);

    // Initialize the input and output messages
    // 初始化 goal（发送给服务器的目标）与 result（服务器返回的结果）为默认值，
    // 子类构造函数随后会按需填充 goal_ 的具体字段。
    goal_ = typename ActionT::Goal();
    result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

    // 支持通过端口 "server_name" 在 XML 中动态改写要连接的 action 服务名
    // （例如同一棵树里对接不同命名空间下的服务器）。
    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      action_name_ = remapped_action_name;
    }
    // 创建动作客户端并阻塞等待服务器上线（超时则抛异常，节点构造失败）。
    createActionClient(action_name_);

    RCLCPP_DEBUG(node_->get_logger(), "\"%s\" BtActionNode initialized", xml_tag_name.c_str());
  }

  // 禁用默认构造：必须显式提供 XML 标签名、服务名和配置。
  BtActionNode() = delete;

  // 虚析构函数：便于通过基类指针安全释放子类对象。
  virtual ~BtActionNode()
  {
  }

  /**
   * @brief Create instance of an action client
   *        （创建 ROS2 Action 客户端并等待服务器可用）
   * @param action_name Action name to create client for
   *        （Action 服务名）
   */
  void createActionClient(const std::string & action_name)
  {
    // Now that we have the ROS node to use, create the action client for this BT action
    // 基于共享节点和本节点的回调组创建 rclcpp_action 客户端：
    // 客户端会自动订阅 /goal、/cancel、/feedback、/result 等话题。
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name, callback_group_);

    RCLCPP_DEBUG(node_->get_logger(), "Waiting for \"%s\" action server", action_name.c_str());
    // 阻塞等待 action 服务器出现在网络上（发送 ros2 服务发现探测）。
    // 若超时仍未发现，抛出 runtime_error 使该 BT 节点构造失败。
    if (!action_client_->wait_for_action_server(wait_for_service_timeout_)) {
      RCLCPP_ERROR(
        node_->get_logger(), "\"%s\" action server not available after waiting for %.2fs",
        action_name.c_str(),
        wait_for_service_timeout_.count() / 1000.0);
      throw std::runtime_error(
              std::string("Action server ") + action_name +
              std::string(" not available"));
    }
  }

  /**
   * @brief Any subclass of BtActionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   *        （子类若要声明自己的端口，必须重写 providedPorts() 并在其中
   *         调用 providedBasicPorts() 以保留基类的基础端口）
   * @param addition Additional ports to add to BT port list
   *        （子类要追加的端口列表）
   * @return BT::PortsList Containing basic ports along with node-specific ports
   *        （基础端口 + 子类端口的合并列表）
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    // 基础端口（所有 action 节点通用）：
    // - server_name：允许 XML 中覆盖 action 服务名（无默认值）
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "Action server name"),
      // - server_timeout：服务器应答超时，单位毫秒（无默认值，通常由黑板提供）
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    // 把子类传入的附加端口合并进基础端口表。
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief Creates list of BT ports
   *        （生成该节点的端口列表；未被重写时只有基础端口）
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  // Derived classes can override any of the following methods to hook into the
  // processing for the action: on_tick, on_wait_for_result, and on_success

  /**
   * @brief Function to perform some user-defined operation on tick
   *        （每个 tick 开始时的钩子：子类在此读取黑板更新 goal、做动态检查）
   * Could do dynamic checks, such as getting updates to values on the blackboard
   */
  virtual void on_tick()
  {
  }

  /**
   * @brief Function to perform some user-defined operation after a timeout
   * waiting for a result that hasn't been received yet. Also provides access to
   * the latest feedback message from the action server. Feedback will be nullptr
   * in subsequent calls to this function if no new feedback is received while waiting for a result.
   *        （等待结果期间的周期性钩子：可在此处理服务器反馈 feedback，
   *         例如 FollowPath 反馈当前速度、剩余距离等）
   * @param feedback shared_ptr to latest feedback message, nullptr if no new feedback was received
   *        （最新的反馈消息；本次等待周期内没有新反馈时为 nullptr）
   */
  virtual void on_wait_for_result(std::shared_ptr<const typename ActionT::Feedback>/*feedback*/)
  {
  }

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the action. Could put a value on the blackboard.
   *        （动作成功完成时的钩子：可把结果写回黑板，如把路径写入 "path" 键）
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   *        （默认返回 SUCCESS，子类可改写为其他状态）
   */
  virtual BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief Function to perform some user-defined operation when the action is aborted.
   *        （动作被服务器中止(ABORTED)时的钩子：例如路径规划失败时记录日志）
   * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
   *        （默认返回 FAILURE）
   */
  virtual BT::NodeStatus on_aborted()
  {
    return BT::NodeStatus::FAILURE;
  }

  /**
   * @brief Function to perform some user-defined operation when the action is cancelled.
   *        （动作被取消(CANCELED)时的钩子：例如 halt 或目标更新导致的取消）
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   *        （默认返回 SUCCESS，即"主动取消"不算失败）
   */
  virtual BT::NodeStatus on_cancelled()
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief The main override required by a BT action
   *        （BT 动作节点的主函数：行为树引擎每个周期调用一次，
   *         实现 goal 发送 -> 等待应答 -> 等待结果 -> 分发回调 的完整状态机）
   * @return BT::NodeStatus Status of tick execution
   *        （返回 SUCCESS / FAILURE / RUNNING）
   */
  BT::NodeStatus tick() override
  {
    // first step to be done only at the beginning of the Action
    // ---- 阶段 1：仅在动作开始时（上一次被复位为 IDLE 后的首次 tick）执行 ----
    if (status() == BT::NodeStatus::IDLE) {
      // 状态迁移：IDLE -> RUNNING，之后本节点会连续被 tick 直到完成或被 halt。
      setStatus(BT::NodeStatus::RUNNING);

      // reset the flag to send the goal or not, allowing the user the option to set it in on_tick
      // 重置"是否发送 goal"标志，允许子类在 on_tick 中把它置 false 来跳过本次发送。
      should_send_goal_ = true;

      // 调用子类的 on_tick 钩子：子类通常在此用 getInput() 读取端口/黑板，
      // 填充 goal_ 字段或做前置检查。
      on_tick();

      // 若子类决定不发送 goal（例如输入非法），本节点直接返回 FAILURE。
      if (!should_send_goal_) {
        return BT::NodeStatus::FAILURE;
      }
      // 首次发送 goal（见 send_new_goal）。
      send_new_goal();
    }

    try {
      // ---- 阶段 2：等待服务器对 goal 请求的"受理应答" ----
      // if new goal was sent and action server has not yet responded
      // check the future goal handle
      // future_goal_handle_ 非空说明服务器尚未应答新发送的 goal。
      if (future_goal_handle_) {
        auto elapsed =
          (node_->now() - time_goal_sent_).template to_chrono<std::chrono::milliseconds>();
        // 在不超过 max_timeout_ 的时间内等待应答 future 完成。
        if (!is_future_goal_handle_complete(elapsed)) {
          // return RUNNING if there is still some time before timeout happens
          // 尚未超时：本 tick 先返回 RUNNING，下个周期继续等待。
          if (elapsed < server_timeout_) {
            return BT::NodeStatus::RUNNING;
          }
          // 超时仍未应答：丢弃 future，本节点判定失败。
          RCLCPP_WARN(
            node_->get_logger(),
            "Timed out while waiting for action server to acknowledge goal request for %s",
            action_name_.c_str());
          future_goal_handle_.reset();
          return BT::NodeStatus::FAILURE;
        }
      }

      // ---- 阶段 3："RUNNING" 主循环：等待最终结果 ----
      // The following code corresponds to the "RUNNING" loop
      // rclcpp::ok()：检查 ROS2 全局是否仍在正常运行（未被 Ctrl+C 关闭）。
      if (rclcpp::ok() && !goal_result_available_) {
        // 调用子类钩子处理最新 feedback（可能为 nullptr），随后清空反馈
        // 防止下个周期误用过期的反馈数据。
        on_wait_for_result(feedback_);

        // reset feedback to avoid stale information
        feedback_.reset();

        // 若子类在 on_tick/on_wait_for_result 中修改了 goal_ 且置了
        // goal_updated_ 标志（如目标点变了），在服务器仍在执行旧目标时
        // 重新发送新目标，实现"运行中更新目标"。
        auto goal_status = goal_handle_->get_status();
        if (goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
          goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
        {
          goal_updated_ = false;
          send_new_goal();
          // 新 goal 同样要等待服务器受理应答，逻辑与阶段 2 相同。
          auto elapsed =
            (node_->now() - time_goal_sent_).template to_chrono<std::chrono::milliseconds>();
          if (!is_future_goal_handle_complete(elapsed)) {
            if (elapsed < server_timeout_) {
              return BT::NodeStatus::RUNNING;
            }
            RCLCPP_WARN(
              node_->get_logger(),
              "Timed out while waiting for action server to acknowledge goal request for %s",
              action_name_.c_str());
            future_goal_handle_.reset();
            return BT::NodeStatus::FAILURE;
          }
        }

        // 手动泵取本节点回调组的挂起回调（结果回调、反馈回调等）。
        // spin_some() 非阻塞：执行完已就绪的回调后立刻返回。
        callback_group_executor_.spin_some();

        // check if, after invoking spin_some(), we finally received the result
        // 若结果回调还没触发（结果未到），本周期返回 RUNNING 让出 CPU。
        if (!goal_result_available_) {
          // Yield this Action, returning RUNNING
          return BT::NodeStatus::RUNNING;
        }
      }
    } catch (const std::runtime_error & e) {
      // ---- 异常分流：区分"动作级失败"与"内部错误" ----
      if (e.what() == std::string("send_goal failed") ||
        e.what() == std::string("Goal was rejected by the action server"))
      {
        // Action related failure that should not fail the tree, but the node
        // 发送失败或目标被服务器拒绝：只让本节点失败，不影响整棵树。
        return BT::NodeStatus::FAILURE;
      } else {
        // Internal exception to propagate to the tree
        // 其他异常属于程序内部错误，向上抛给行为树引擎统一处理。
        throw e;
      }
    }

    // ---- 阶段 4：分发结果钩子 ----
    // 根据 action 结果码调用对应的子类钩子，由钩子决定最终返回状态。
    BT::NodeStatus status;
    switch (result_.code) {
      // 动作成功：默认返回 SUCCESS。
      case rclcpp_action::ResultCode::SUCCEEDED:
        status = on_success();
        break;

      // 动作被服务器中止（执行中出错）：默认返回 FAILURE。
      case rclcpp_action::ResultCode::ABORTED:
        status = on_aborted();
        break;

      // 动作被取消（halt 或重新发目标导致）：默认返回 SUCCESS。
      case rclcpp_action::ResultCode::CANCELED:
        status = on_cancelled();
        break;

      default:
        // 未知结果码视为逻辑错误，直接抛异常。
        throw std::logic_error("BtActionNode::Tick: invalid status value");
    }

    // 释放当前 goal 句柄，节点回到"无进行中目标"状态，
    // 下次进入 tick 时 status 仍是 RUNNING，直到父节点 halt 或树重置。
    goal_handle_.reset();
    return status;
  }

  /**
   * @brief The other (optional) override required by a BT action. In this case, we
   * make sure to cancel the ROS2 action if it is still running.
   *        （BT 节点中断钩子：父节点放弃本节点时调用，
   *         负责取消仍在执行的 ROS2 action 并复位节点状态）
   */
  void halt() override
  {
    // 若当前有正在执行且需要取消的 goal：先取结果再取消，确保状态一致。
    if (should_cancel_goal()) {
      // 异步请求获取最终结果（取消后的结果），用于状态同步。
      auto future_result = action_client_->async_get_result(goal_handle_);
      // 异步发送取消请求给 action 服务器。
      auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
      // 在超时上限内等待取消请求完成；失败只记日志，不抛异常
      // （halt 是清理路径，不应中断整棵树）。
      if (callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Failed to cancel action server for %s", action_name_.c_str());
      }

      // 等待取消后的最终结果，避免悬挂的回调在节点销毁后被触发。
      if (callback_group_executor_.spin_until_future_complete(future_result, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Failed to get result for %s in node halt!", action_name_.c_str());
      }

      // 通知子类"动作被取消"，让其做业务层清理（如复位内部计数）。
      on_cancelled();
    }

    // 把节点状态复位为 IDLE，下次 tick 将走"阶段 1"重新开始整个流程。
    setStatus(BT::NodeStatus::IDLE);
  }

protected:
  /**
   * @brief Function to check if current goal should be cancelled
   *        （判断当前是否存在需要取消的 goal）
   * @return bool True if current goal should be cancelled, false otherwise
   *        （true = 需要取消；false = 无需取消）
   */
  bool should_cancel_goal()
  {
    // Shut the node down if it is currently running
    // 节点不在 RUNNING 状态说明没有进行中的动作，无需取消。
    if (status() != BT::NodeStatus::RUNNING) {
      return false;
    }

    // No need to cancel the goal if goal handle is invalid
    // goal 句柄为空说明 goal 尚未发送或已结束，无需取消。
    if (!goal_handle_) {
      return false;
    }

    // 先泵取回调，确保 goal 状态是最新的（服务器可能刚推送状态变更）。
    callback_group_executor_.spin_some();
    auto status = goal_handle_->get_status();

    // Check if the goal is still executing
    // 只有目标仍处于"已受理/执行中"状态时才需要向服务器发送取消请求。
    return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
           status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }

  /**
   * @brief Function to send new goal to action server
   *        （向 action 服务器发送新的 goal，并注册结果/反馈回调）
   */
  void send_new_goal()
  {
    // 复位"结果已到"标志：等待新 goal 的结果。
    goal_result_available_ = false;
    // 构造发送选项：注册结果回调和反馈回调（这是 rclcpp_action 的回调风格）。
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    // ---- 结果回调：服务器推送最终结果时触发（在回调组执行器中执行）----
    send_goal_options.result_callback =
      [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result) {
        // 若此时还没收到 goal 受理应答，说明这是上一个 goal 的迟到结果，
        // 直接忽略，避免新旧目标的结果错乱。
        if (future_goal_handle_) {
          RCLCPP_DEBUG(
            node_->get_logger(),
            "Goal result for %s available, but it hasn't received the goal response yet. "
            "It's probably a goal result for the last goal request", action_name_.c_str());
          return;
        }

        // TODO(#1652): a work around until rcl_action interface is updated
        // if goal ids are not matched, the older goal call this callback so ignore the result
        // if matched, it must be processed (including aborted)
        // 校验 goal_id：只有与当前 goal 句柄匹配的结果才被采纳
        // （防止旧 goal 的取消结果覆盖新 goal 的状态）。
        if (this->goal_handle_->get_goal_id() == result.goal_id) {
          goal_result_available_ = true;
          result_ = result;
        }
      };
    // ---- 反馈回调：服务器周期性推送进度反馈时触发 ----
    send_goal_options.feedback_callback =
      [this](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
        const std::shared_ptr<const typename ActionT::Feedback> feedback) {
        // 暂存最新反馈，供下一次 on_wait_for_result() 钩子使用。
        feedback_ = feedback;
      };

    // 异步发送 goal：立即返回 future（受理应答），不阻塞 tick；
    // future_goal_handle_ 存共享 future，供 is_future_goal_handle_complete() 等待。
    future_goal_handle_ = std::make_shared<
      std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>(
      action_client_->async_send_goal(goal_, send_goal_options));
    // 记录发送时刻，用于计算等待应答的耗时是否超时。
    time_goal_sent_ = node_->now();
  }

  /**
   * @brief Function to check if the action server acknowledged a new goal
   *        （等待并检查服务器是否已受理新 goal）
   * @param elapsed Duration since the last goal was sent and future goal handle has not completed.
   * After waiting for the future to complete, this value is incremented with the timeout value.
   *        （入参/出参：自发送 goal 以来已等待的时长；本函数等待后会把
   *         实际等待时间累加进去，供调用方判断是否总超时）
   * @return boolean True if future_goal_handle_ returns SUCCESS, False otherwise
   *        （true = 服务器已受理并返回 goal 句柄；false = 尚未应答或失败）
   */
  bool is_future_goal_handle_complete(std::chrono::milliseconds & elapsed)
  {
    // 计算距离 server_timeout_ 还剩多少等待额度。
    auto remaining = server_timeout_ - elapsed;

    // server has already timed out, no need to sleep
    // 额度已耗尽：放弃等待并清空 future，返回失败。
    if (remaining <= std::chrono::milliseconds(0)) {
      future_goal_handle_.reset();
      return false;
    }

    // 单次等待时长取 min(剩余额度, max_timeout_)，
    // max_timeout_ = bt_loop_duration 的一半，防止单次 tick 被阻塞过久。
    auto timeout = remaining > max_timeout_ ? max_timeout_ : remaining;
    auto result =
      callback_group_executor_.spin_until_future_complete(*future_goal_handle_, timeout);
    // 把本次实际等待的时长累加进 elapsed（引用传回给调用方）。
    elapsed += timeout;

    // 执行器被中断（如 rclcpp 关闭中）：视为发送失败。
    if (result == rclcpp::FutureReturnCode::INTERRUPTED) {
      future_goal_handle_.reset();
      throw std::runtime_error("send_goal failed");
    }

    // 服务器已应答：取出 goal 句柄（后续可用它查询状态、取消、取结果）。
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      goal_handle_ = future_goal_handle_->get();
      future_goal_handle_.reset();
      // 句柄为空表示服务器拒绝了该 goal（如同名目标已在执行）。
      if (!goal_handle_) {
        throw std::runtime_error("Goal was rejected by the action server");
      }
      return true;
    }

    // 超时但整体额度未耗尽：返回 false，调用方下个周期继续等待。
    return false;
  }

  /**
   * @brief Function to increment recovery count on blackboard if this node wraps a recovery
   *        （若本节点是"恢复行为"节点，则递增黑板上的恢复计数器
   *         number_recoveries，供上层监控导航尝试了多少次恢复）
   */
  void increment_recovery_count()
  {
    // 读取当前计数（黑板中不存在时保持初值 0）。
    int recovery_count = 0;
    config().blackboard->template get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    // 写回黑板，供 RecoveryNode / Groot / 监控端读取。
    config().blackboard->template set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  // ---- 成员变量说明 ----
  // 要连接的 action 服务名（可被 XML 端口 server_name 覆盖）。
  std::string action_name_;
  // ROS2 Action 客户端：负责 goal/取消/结果/反馈的全部底层通信。
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

  // 即将发送给服务器的 goal 对象（子类填充字段）。
  typename ActionT::Goal goal_;
  // 标志：子类在运行中修改了 goal_，需要重新发送（运行中更新目标）。
  bool goal_updated_{false};
  // 标志：结果回调已触发、result_ 已就绪。
  bool goal_result_available_{false};
  // 当前 goal 的句柄：由服务器受理应答返回，可查询状态、取消、取结果。
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
  // 最近一次收到的最终结果（含 ResultCode 与返回值）。
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

  // 最近一次收到的过程反馈（每轮 on_wait_for_result 后被清空）。
  std::shared_ptr<const typename ActionT::Feedback> feedback_;

  // 共享的 ROS2 节点指针（从黑板 "node" 键获取）。
  rclcpp::Node::SharedPtr node_;
  // 本节点专属的互斥回调组：隔离与其他节点的回调并发。
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  // 泵取 callback_group_ 回调的单线程执行器（在 tick/halt 中手动驱动）。
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // The timeout value while waiting for response from a server when a
  // 等待服务器应答（受理 goal、取消等）的超时时间（毫秒）。
  std::chrono::milliseconds server_timeout_;

  // 单次同步等待的上限（bt_loop_duration 的一半），保证 tick 不阻塞过久。
  std::chrono::milliseconds max_timeout_;

  // 构造阶段等待 action 服务器上线的超时时间（毫秒）。
  std::chrono::milliseconds wait_for_service_timeout_;

  // "服务器受理应答"对应的共享 future（send_new_goal 创建、应答后清空）。
  std::shared_ptr<std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>
  future_goal_handle_;
  // 最近一次发送 goal 的时刻，用于计算应答等待耗时。
  rclcpp::Time time_goal_sent_;

  // Can be set in on_tick or on_wait_for_result to indicate if a goal should be sent.
  // 子类可在 on_tick / on_wait_for_result 中置 false，表示本周期不发送 goal
  // （tick 会直接返回 FAILURE）。
  bool should_send_goal_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
