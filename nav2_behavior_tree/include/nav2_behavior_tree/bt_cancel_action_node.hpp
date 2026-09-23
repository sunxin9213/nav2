// Copyright (c) 2022 Neobotix GmbH
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
// 【文件说明】bt_cancel_action_node.hpp
// 【所属模块】nav2_behavior_tree —— Nav2 的行为树核心模块
//
// 本文件定义了 BtCancelActionNode<ActionT> 模板类——所有"取消型"动作节点的
// 公共抽象基类。它与 BtActionNode 相反：不发送新 goal，而是向某个 action
// 服务器发送取消请求，把该服务器上"指定时刻之前受理的所有 goal"全部取消。
//
// 【在 Nav2 导航系统中的位置和作用】
// Nav2 的导航树中有一组 *Cancel 节点（如 SpinCancel、BackUpCancel、
// WaitCancel、ControllerCancel、AssistedTeleopCancel、DriveOnHeadingCancel），
// 它们都继承本类。它们通常被放在 Recovery 分支或 fallback 分支中：当导航
// 失败需要切换行为时，先取消正在执行的动作（例如停止跟随路径），再执行
// 恢复行为。
//
// 【BehaviorTree.CPP 核心概念】
// - 本类是"动作节点"（继承 BT::ActionNodeBase），但实现为"同步节点"：
//   tick() 会在一个周期内完成取消请求并等待应答（受 server_timeout 限制），
//   返回 SUCCESS（取消成功）或 FAILURE（取消超时/失败）。
// - halt() 为空实现：因为取消操作本身很快且幂等，无需中断清理。
// - providedPorts() 提供 server_name / server_timeout 两个输入端口，
//   子类可通过 providedBasicPorts() 追加端口。
//
// 【XML 使用示例】（以子类 SpinCancel 为例，一行）
//   <SpinCancel server_name="spin" server_timeout="20"/>
// =============================================================================

#ifndef NAV2_BEHAVIOR_TREE__BT_CANCEL_ACTION_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__BT_CANCEL_ACTION_NODE_HPP_

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
 * @brief Abstract class representing an action for cancelling BT node
 *        （表示"取消 ROS2 Action"的行为树节点的抽象基类）
 * @tparam ActionT Type of action
 *        （模板参数：要取消的 ROS2 Action 类型，如 nav2_msgs::action::Spin）
 */
template<class ActionT>
class BtCancelActionNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::BtCancelActionNode constructor
   *        （构造函数：初始化动作客户端与超时参数）
   * @param xml_tag_name Name for the XML tag for this node
   *        （节点在行为树 XML 中的标签名，如 "SpinCancel"）
   * @param action_name Action name this node creates a client for
   *        （要连接并取消的 Action 服务名，如 "spin"）
   * @param conf BT node configuration
   *        （BT 节点配置，含黑板指针和端口映射）
   */
  BtCancelActionNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name)
  {
    // 从黑板取出共享的 ROS2 节点（由 BtActionServer 初始化时放入）。
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    // 创建本节点专属的互斥回调组（不自动加入默认执行器，第二参数 false），
    // 避免主执行器与本节点的回调产生并发竞争。
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    // 把回调组挂到本节点私有的单线程执行器上，tick 中手动泵取回调。
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // Get the required items from the blackboard
    // 从黑板读取服务器应答超时；若 XML 端口 server_timeout 有值则优先。
    server_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
    // 构造阶段等待 action 服务器上线的超时时间。
    wait_for_service_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("wait_for_service_timeout");

    // 允许在 XML 中通过 server_name 端口动态改写要取消的服务名。
    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      action_name_ = remapped_action_name;
    }
    // 创建动作客户端并阻塞等待服务器上线（超时抛异常，节点构造失败）。
    createActionClient(action_name_);

    RCLCPP_DEBUG(
      node_->get_logger(), "\"%s\" BtCancelActionNode initialized",
      xml_tag_name.c_str());
  }

  // 禁用默认构造：必须显式提供标签名、服务名和配置。
  BtCancelActionNode() = delete;

  // 虚析构函数：支持通过基类指针安全释放子类。
  virtual ~BtCancelActionNode()
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
    // 基于共享节点 + 本节点回调组创建 action 客户端
    // （自动订阅 goal/cancel/result 相关话题）。
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name, callback_group_);

    RCLCPP_DEBUG(node_->get_logger(), "Waiting for \"%s\" action server", action_name.c_str());
    // 阻塞等待 action 服务器出现；超时则抛异常终止节点构造。
    if (!action_client_->wait_for_action_server(wait_for_service_timeout_)) {
      RCLCPP_ERROR(
        node_->get_logger(), "\"%s\" action server not available after waiting for %.2fs",
        action_name.c_str(), wait_for_service_timeout_.count() / 1000.0);
      throw std::runtime_error(
              std::string("Action server ") + action_name +
              std::string(" not available"));
    }
  }

  /**
   * @brief Any subclass of BtCancelActionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   *        （子类若声明自己的端口，须重写 providedPorts() 并调用本函数）
   * @param addition Additional ports to add to BT port list
   *        （子类追加的端口列表）
   * @return BT::PortsList Containing basic ports along with node-specific ports
   *        （基础端口 + 子类端口的合并结果）
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    // 基础端口（所有取消节点通用）：
    // - server_name：可在 XML 中覆盖 action 服务名
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "Action server name"),
      // - server_timeout：等待服务器应答的超时（毫秒）
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    // 合并子类附加端口。
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  // halt() 空实现：取消请求本身短促且无副作用，无需在节点被中断时做额外清理。
  void halt() override
  {
  }

  /**
   * @brief Creates list of BT ports
   *        （生成端口列表；未被重写时仅含基础端口）
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  /**
   * @brief The main override required by a BT action
   *        （节点主函数：发送"批量取消"请求并等待应答，单周期同步完成）
   * @return BT::NodeStatus Status of tick execution
   *        （SUCCESS = 取消成功；FAILURE = 取消超时或失败）
   */
  BT::NodeStatus tick() override
  {
    // 标记本节点进入执行状态（虽然本节点单周期完成，仍按 BT 约定设置 RUNNING）。
    setStatus(BT::NodeStatus::RUNNING);

    // Cancel all the goals specified before 10ms from current time
    // to avoid async communication error
    // 计算取消截止时刻 = 当前时间往前回退 10ms：
    // 留出这 10ms 的余量是为了规避异步通信中"刚好在边界上受理的 goal"
    // 造成的服务器端状态竞争（time-of-check 到 time-of-use 的间隙）。
    rclcpp::Time goal_expiry_time = node_->now() - std::chrono::milliseconds(10);

    // 异步发送批量取消请求：取消该服务器上所有受理时间早于截止时刻的 goal。
    // 立即返回 future，不阻塞。
    auto future_cancel = action_client_->async_cancel_goals_before(goal_expiry_time);

    // 在 server_timeout_ 上限内等待取消应答完成。
    if (callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      // 超时或中断：取消失败，节点返回 FAILURE。
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to cancel the action server for %s", action_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    // 取消成功：返回 SUCCESS，上游（通常是 fallback/恢复分支）继续执行。
    return BT::NodeStatus::SUCCESS;
  }

protected:
  // ---- 成员变量说明 ----
  // 要连接并取消的 action 服务名（可被 XML 端口 server_name 覆盖）。
  std::string action_name_;
  // ROS2 Action 客户端：负责发送取消请求及底层通信。
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

  // 共享的 ROS2 节点指针（黑板 "node" 键）。
  rclcpp::Node::SharedPtr node_;
  // 本节点专属的互斥回调组：隔离回调并发。
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  // 泵取回调组回调的单线程执行器（tick 中手动驱动）。
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // The timeout value while waiting for response from a server when a
  // 等待服务器应答（取消请求等）的超时时间（毫秒）。
  std::chrono::milliseconds server_timeout_;
  // 构造阶段等待服务器上线的超时时间（毫秒）。
  std::chrono::milliseconds wait_for_service_timeout_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_CANCEL_ACTION_NODE_HPP_
