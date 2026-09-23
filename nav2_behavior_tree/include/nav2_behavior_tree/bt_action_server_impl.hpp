// Copyright (c) 2020 Sarthak Mittal
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
// 【文件说明】bt_action_server_impl.hpp
// 【所属模块】nav2_behavior_tree —— Nav2 的行为树核心模块
//
// 本文件是 BtActionServer<ActionT> 模板类的实现文件（模板类实现必须
// 对使用方可见，因此放在 .hpp 中，由 bt_action_server.hpp 在末尾 include）。
// 类的职责与设计说明见 bt_action_server.hpp 文件头。
//
// 【实现要点】
// - 生命周期：on_configure（建客户端节点/黑板/引擎/服务器）→ on_activate
//   （加载默认树并启动服务器）→ on_deactivate（停服务器）→ on_cleanup
//   （全部复位）。与 Nav2 生命周期管理器配合。
// - executeCallback() 是核心：action 服务器收到 goal 后在专用线程调用它，
//   它驱动 BehaviorTreeEngine::run() 循环 tick 行为树，把树结果转换成
//   action 的 succeeded/terminate 结果。
// - 黑板初始化：往黑板写入 "node"（客户端节点）、"server_timeout"、
//   "bt_loop_duration"、"wait_for_service_timeout" 等键，
//   供所有 BtActionNode/BtServiceNode 子类构造时读取。
// =============================================================================

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_

#include <memory>
#include <string>
#include <fstream>
#include <set>
#include <exception>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behavior_tree
{

// ---- 构造函数：仅保存参数并声明 ROS2 参数，真正的初始化在 on_configure ----
template<class ActionT>
BtActionServer<ActionT>::BtActionServer(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & action_name,
  const std::vector<std::string> & plugin_lib_names,
  const std::string & default_bt_xml_filename,
  OnGoalReceivedCallback on_goal_received_callback,
  OnLoopCallback on_loop_callback,
  OnPreemptCallback on_preempt_callback,
  OnCompletionCallback on_completion_callback)
: action_name_(action_name),
  default_bt_xml_filename_(default_bt_xml_filename),
  plugin_lib_names_(plugin_lib_names),
  node_(parent),
  on_goal_received_callback_(on_goal_received_callback),
  on_loop_callback_(on_loop_callback),
  on_preempt_callback_(on_preempt_callback),
  on_completion_callback_(on_completion_callback)
{
  // 提升父节点弱引用，取日志器和时钟。
  auto node = node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Declare this node's parameters
  // 声明本服务器需要的 ROS2 参数（若尚未声明，避免重复声明报错）：
  // bt_loop_duration = 单轮 tick 周期预算（ms，默认 10）
  if (!node->has_parameter("bt_loop_duration")) {
    node->declare_parameter("bt_loop_duration", 10);
  }
  // default_server_timeout = BT 节点等待服务器应答的默认超时（ms，默认 20）
  if (!node->has_parameter("default_server_timeout")) {
    node->declare_parameter("default_server_timeout", 20);
  }
  // always_reload_bt_xml = 即使同名 XML 也强制重建树（调试用，默认 false）
  if (!node->has_parameter("always_reload_bt_xml")) {
    node->declare_parameter("always_reload_bt_xml", false);
  }
  // wait_for_service_timeout = BT 节点等待服务上线的超时（ms，默认 1000）
  if (!node->has_parameter("wait_for_service_timeout")) {
    node->declare_parameter("wait_for_service_timeout", 1000);
  }
}

// ---- 析构函数：成员均为智能指针，无需手动释放 ----
template<class ActionT>
BtActionServer<ActionT>::~BtActionServer()
{}

// ---- 生命周期 configure：创建客户端节点、action 服务器、BT 引擎和黑板 ----
template<class ActionT>
bool BtActionServer<ActionT>::on_configure()
{
  // 提升父节点弱引用；父节点已销毁则无法配置。
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Name client node after action name
  // 用 action 名给客户端节点命名（把 '/' 替换为 '_' 以符合命名规则），
  // 例如 "navigate_to_pose" -> 节点 "bt_navigator_navigate_to_pose_rclcpp_node"。
  std::string client_node_name = action_name_;
  std::replace(client_node_name.begin(), client_node_name.end(), '/', '_');
  // Use suffix '_rclcpp_node' to keep parameter file consistency #1773
  // 通过命令行参数重映射节点名（保持参数文件命名一致性，见 issue #1773）。
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args",
      "-r",
      std::string("__node:=") +
      std::string(node->get_name()) + "_" + client_node_name + "_rclcpp_node",
      "--"});

  // Support for handling the topic-based goal pose from rviz
  // 创建独立的普通 ROS2 节点（非生命周期节点）：
  // BT 内部节点（发布者/订阅者/action 客户端）都挂在这个节点上，
  // 这样父生命周期节点处于 inactive 时 BT 仍可通信；
  // 同时该节点可订阅 rviz 的 "goal_pose" 等话题。
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  // Declare parameters for common client node applications to share with BT nodes
  // Declare if not declared in case being used an external application, then copying
  // 声明并复制公共参数到客户端节点，让 BT 节点能直接读取：
  // global_frame / robot_base_frame / transform_tolerance。
  nav2_util::declare_parameter_if_not_declared(
    node, "global_frame", rclcpp::ParameterValue(std::string("map")));
  nav2_util::declare_parameter_if_not_declared(
    node, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  nav2_util::declare_parameter_if_not_declared(
    node, "transform_tolerance", rclcpp::ParameterValue(0.1));
  nav2_util::copy_all_parameters(node, client_node_);

  // 创建底层 action 服务器：绑定父生命周期节点的各接口
  // （base/clock/logging/waitables）和 action 名，
  // 收到 goal 时在专用线程调用本类的 executeCallback。
  action_server_ = std::make_shared<ActionServer>(
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    action_name_, std::bind(&BtActionServer<ActionT>::executeCallback, this));

  // Get parameters for BT timeouts
  // 读取构造时声明的超时参数，转成毫秒时长存入成员。
  int timeout;
  node->get_parameter("bt_loop_duration", timeout);
  bt_loop_duration_ = std::chrono::milliseconds(timeout);
  node->get_parameter("default_server_timeout", timeout);
  default_server_timeout_ = std::chrono::milliseconds(timeout);
  int wait_for_service_timeout;
  node->get_parameter("wait_for_service_timeout", wait_for_service_timeout);
  wait_for_service_timeout_ = std::chrono::milliseconds(wait_for_service_timeout);
  node->get_parameter("always_reload_bt_xml", always_reload_bt_xml_);

  // Create the class that registers our custom nodes and executes the BT
  // 创建行为树引擎：动态加载全部 BT 插件库并注册节点类型。
  bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);

  // Create the blackboard that will be shared by all of the nodes in the tree
  // 创建黑板：整棵树所有节点共享的数据仓库。
  blackboard_ = BT::Blackboard::create();

  // Put items on the blackboard
  // 往黑板写入各 BT 节点构造时需要的公共键：
  // "node" = 客户端节点指针（BT 节点用它创建发布者/客户端等）
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
  // "server_timeout" = 服务器应答默认超时
  blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);  // NOLINT
  // "bt_loop_duration" = 单轮 tick 周期预算
  blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);  // NOLINT
  // "wait_for_service_timeout" = 等待服务上线超时
  blackboard_->set<std::chrono::milliseconds>(
    "wait_for_service_timeout",
    wait_for_service_timeout_);

  return true;
}

// ---- 生命周期 activate：加载默认行为树并启动 action 服务器 ----
template<class ActionT>
bool BtActionServer<ActionT>::on_activate()
{
  // 从默认 XML 文件构建行为树；失败则激活失败。
  if (!loadBehaviorTree(default_bt_xml_filename_)) {
    RCLCPP_ERROR(logger_, "Error loading XML file: %s", default_bt_xml_filename_.c_str());
    return false;
  }
  // 激活 action 服务器：开始接收并接受 goal。
  action_server_->activate();
  return true;
}

// ---- 生命周期 deactivate：停用 action 服务器（不再接收新 goal） ----
template<class ActionT>
bool BtActionServer<ActionT>::on_deactivate()
{
  action_server_->deactivate();
  return true;
}

// ---- 生命周期 cleanup：复位全部成员，回到未配置状态 ----
template<class ActionT>
bool BtActionServer<ActionT>::on_cleanup()
{
  // 依次释放客户端节点、action 服务器、日志发布器。
  client_node_.reset();
  action_server_.reset();
  topic_logger_.reset();
  // 清空插件库名列表和当前 XML 文件名。
  plugin_lib_names_.clear();
  current_bt_xml_filename_.clear();
  // 释放黑板（各 BT 节点已析构，不再需要）。
  blackboard_.reset();
  // 先 halt 树中所有节点（触发清理，如取消进行中的 action）再销毁引擎。
  bt_->haltAllActions(tree_.rootNode());
  bt_.reset();
  return true;
}

// ---- 加载/切换行为树：读 XML 文件、重建树、重置黑板键、挂日志器 ----
template<class ActionT>
bool BtActionServer<ActionT>::loadBehaviorTree(const std::string & bt_xml_filename)
{
  // Empty filename is default for backward compatibility
  // 空文件名视为"使用默认文件"（向后兼容）。
  auto filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

  // Use previous BT if it is the existing one and always reload flag is not set to true
  // 若请求的文件就是当前已加载的文件且未开启强制重载：直接复用旧树。
  if (!always_reload_bt_xml_ && current_bt_xml_filename_ == filename) {
    RCLCPP_DEBUG(logger_, "BT will not be reloaded as the given xml is already loaded");
    return true;
  }

  // 打开 XML 文件。
  std::ifstream xml_file(filename);

  if (!xml_file.good()) {
    // 文件不存在或不可读：加载失败（保留旧树）。
    RCLCPP_ERROR(logger_, "Couldn't open input XML file: %s", filename.c_str());
    return false;
  }

  // 用流迭代器把整个文件读入字符串。
  auto xml_string = std::string(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>());

  // Create the Behavior Tree from the XML input
  // 由引擎从 XML 文本创建行为树；同时向树的所有黑板栈层
  // （blackboard_stack：子树各有自己的黑板层）重新注入公共键，
  // 保证每层黑板都能取到 node/超时配置。
  try {
    tree_ = bt_->createTreeFromText(xml_string, blackboard_);
    for (auto & blackboard : tree_.blackboard_stack) {
      blackboard->set<rclcpp::Node::SharedPtr>("node", client_node_);
      blackboard->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);
      blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
      blackboard->set<std::chrono::milliseconds>(
        "wait_for_service_timeout",
        wait_for_service_timeout_);
    }
  } catch (const std::exception & e) {
    // XML 解析或节点实例化失败：记日志并保留旧树。
    RCLCPP_ERROR(logger_, "Exception when loading BT: %s", e.what());
    return false;
  }

  // 为新树挂上 ROS 话题日志器（状态变迁发到 /behavior_tree_log）。
  topic_logger_ = std::make_unique<RosTopicLogger>(client_node_, tree_);

  // 记录当前生效的 XML 文件名。
  current_bt_xml_filename_ = filename;
  return true;
}

// ---- action 服务器主回调：goal 到达后驱动行为树直至结束 ----
template<class ActionT>
void BtActionServer<ActionT>::executeCallback()
{
  // 先交给用户的 goal 校验回调；拒绝则直接终止该 goal。
  if (!on_goal_received_callback_(action_server_->get_current_goal())) {
    action_server_->terminate_current();
    return;
  }

  // 取消判定 lambda：引擎每轮循环都会调用它检查是否应中断。
  auto is_canceling = [&]() {
      // 服务器对象已销毁（如清理阶段）：视为取消。
      if (action_server_ == nullptr) {
        RCLCPP_DEBUG(logger_, "Action server unavailable. Canceling.");
        return true;
      }
      // 服务器已停用（deactivate）：视为取消。
      if (!action_server_->is_server_active()) {
        RCLCPP_DEBUG(logger_, "Action server is inactive. Canceling.");
        return true;
      }
      // 正常路径：检查客户端是否发送了取消请求。
      return action_server_->is_cancel_requested();
    };

  // 每轮循环附加执行的 lambda：
  auto on_loop = [&]() {
      // 若有抢占请求（新 goal 到达），调用用户的抢占回调
      // （用户自行决定接受/拒绝新 goal、如何处理旧 goal）。
      if (action_server_->is_preempt_requested() && on_preempt_callback_) {
        on_preempt_callback_(action_server_->get_pending_goal());
      }
      // 把缓存的状态变迁日志发布到话题。
      topic_logger_->flush();
      // 调用用户的每周期回调（如发布 action 反馈）。
      on_loop_callback_();
    };

  // 驱动行为树执行直到 SUCCEEDED/FAILED/CANCELED。
  nav2_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);

  // Make sure that the Bt is not in a running state from a previous execution
  // 复位整棵树：halt 所有节点，保证下次执行从干净的初始状态开始
  // （例如取消正在执行的 FollowPath action）。
  bt_->haltAllActions(tree_.rootNode());

  // Give server an opportunity to populate the result message or simple give
  // an indication that the action is complete.
  // 让用户回调填充结果消息（如写入错误码），再根据树状态回报 action 结果。
  auto result = std::make_shared<typename ActionT::Result>();
  on_completion_callback_(result, rc);

  switch (rc) {
    // 树成功：action 以成功结束。
    case nav2_behavior_tree::BtStatus::SUCCEEDED:
      action_server_->succeeded_current(result);
      RCLCPP_INFO(logger_, "Goal succeeded");
      break;

    // 树失败：终止当前 goal（向客户端返回失败结果）。
    case nav2_behavior_tree::BtStatus::FAILED:
      action_server_->terminate_current(result);
      RCLCPP_ERROR(logger_, "Goal failed");
      break;

    // 树被取消：终止服务器上所有相关 goal。
    case nav2_behavior_tree::BtStatus::CANCELED:
      action_server_->terminate_all(result);
      RCLCPP_INFO(logger_, "Goal canceled");
      break;
  }
}

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_
