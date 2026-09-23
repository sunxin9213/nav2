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
// 【文件说明】bt_action_server.hpp
// 【所属模块】nav2_behavior_tree —— Nav2 的行为树核心模块
//
// 本文件声明了 BtActionServer<ActionT> 模板类——"用行为树实现业务逻辑"
// 的 ROS2 Action 服务器封装。
//
// 【在 Nav2 导航系统中的位置和作用】
// Nav2 的几个顶层节点（bt_navigator、navigate_to_pose 等）并不是手工编写
// action 服务器逻辑，而是把整个导航流程表达成一棵行为树 XML。BtActionServer
// 就是连接两者的桥梁：
//   - 对外：作为 ROS2 Action 服务器接收 goal（如 NavigateToPose）；
//   - 对内：把 goal 写入黑板，循环 tick 行为树直到完成/失败/被取消，
//     并把树执行结果转换为 action 结果返回给客户端；
//   - 同时管理行为树引擎（BehaviorTreeEngine）、动态加载 BT 插件库、
//     支持运行时热切换行为树 XML、发布 BT 日志话题（供 Groot 监控）。
// 它实现了 ROS2 生命周期语义（on_configure/on_activate/on_deactivate/
// on_cleanup），与 Nav2 的生命周期管理器（lifecycle_manager）配合工作。
//
// 【BehaviorTree.CPP 核心概念】
// - 行为树由 XML 描述，节点类型在插件库(.so)中通过 BT_REGISTER_NODES 宏注册；
//   本类的 plugin_lib_names 参数就是这些 .so 的列表，引擎会动态加载它们。
// - blackboard（黑板）：树的共享数据仓库；goal、路径、机器人状态等都在
//   黑板中流转，getBlackboard() 可取出供外部读写。
// - 树的执行是"逐周期 tick"驱动的，由 executeCallback() 在专用线程中循环。
//
// 【用户回调钩子】（由使用方在构造时注入）
// - OnGoalReceivedCallback：收到新 goal 时（如校验 goal 合法性）；
// - OnLoopCallback：每个 BT 执行周期调用（如发布反馈、检查抢占）；
// - OnPreemptCallback：收到抢占（新 goal 打断旧 goal）时；
// - OnCompletionCallback：树执行结束后（组装 action 结果）。
//
// 【典型使用者】nav2_bt_navigator（NavigateToPose/NavigateThroughPoses）。
// 实现细节见 bt_action_server_impl.hpp（模板类实现与声明分离）。
// =============================================================================

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "nav2_behavior_tree/ros_topic_logger.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"

namespace nav2_behavior_tree
{
/**
 * @class nav2_behavior_tree::BtActionServer
 * @brief An action server that uses behavior tree to execute an action
 *        （"以行为树为业务逻辑"的 ROS2 Action 服务器封装）
 */
template<class ActionT>
class BtActionServer
{
public:
  // 底层 action 服务器实现（nav2_util 提供的轻量封装）。
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  // 收到新 goal 时的回调：参数为 goal，返回 bool 表示是否接受。
  typedef std::function<bool (typename ActionT::Goal::ConstSharedPtr)> OnGoalReceivedCallback;
  // 每个 BT 执行周期调用的回调（无参）。
  typedef std::function<void ()> OnLoopCallback;
  // goal 被抢占（新 goal 到达）时的回调。
  typedef std::function<void (typename ActionT::Goal::ConstSharedPtr)> OnPreemptCallback;
  // 树执行结束时的回调：参数为 action 结果与树执行状态。
  typedef std::function<void (typename ActionT::Result::SharedPtr,
      nav2_behavior_tree::BtStatus)> OnCompletionCallback;

  /**
   * @brief A constructor for nav2_behavior_tree::BtActionServer class
   *        （构造函数：保存父节点、action 名、插件库列表、默认 BT XML
   *         文件名和四个用户回调；实际初始化在 on_configure 中进行）
   */
  explicit BtActionServer(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & action_name,
    const std::vector<std::string> & plugin_lib_names,
    const std::string & default_bt_xml_filename,
    OnGoalReceivedCallback on_goal_received_callback,
    OnLoopCallback on_loop_callback,
    OnPreemptCallback on_preempt_callback,
    OnCompletionCallback on_completion_callback);

  /**
   * @brief A destructor for nav2_behavior_tree::BtActionServer class
   *        （析构函数）
   */
  ~BtActionServer();

  /**
   * @brief Configures member variables
   * Initializes action server for, builds behavior tree from xml file,
   * and calls user-defined onConfigure.
   *        （生命周期 configure 阶段：创建客户端节点、初始化黑板、加载
   *         BT 插件库、从 XML 构建行为树）
   * @return bool true on SUCCESS and false on FAILURE
   */
  bool on_configure();

  /**
   * @brief Activates action server
   *        （生命周期 activate 阶段：启动 action 服务器开始接收 goal）
   * @return bool true on SUCCESS and false on FAILURE
   */
  bool on_activate();

  /**
   * @brief Deactivates action server
   *        （生命周期 deactivate 阶段：停止接收新 goal，中断当前执行）
   * @return bool true on SUCCESS and false on FAILURE
   */
  bool on_deactivate();

  /**
   * @brief Resets member variables
   *        （生命周期 cleanup 阶段：复位所有成员，回到未配置状态）
   * @return bool true on SUCCESS and false on FAILURE
   */
  bool on_cleanup();

  /**
   * @brief Replace current BT with another one
   *        （加载/热切换行为树：按 XML 文件重建整棵树）
   * @param bt_xml_filename The file containing the new BT, uses default filename if empty
   *        （新的 BT XML 文件路径；为空则使用默认文件）
   * @return bool true if the resulting BT correspond to the one in bt_xml_filename. false
   * if something went wrong, and previous BT is maintained
   *        （true = 切换成功；false = 失败且保留旧树）
   */
  bool loadBehaviorTree(const std::string & bt_xml_filename = "");

  /**
   * @brief Getter function for BT Blackboard
   *        （获取黑板指针：外部可往黑板写 goal、读路径等共享数据）
   * @return BT::Blackboard::Ptr Shared pointer to current BT blackboard
   */
  BT::Blackboard::Ptr getBlackboard() const
  {
    return blackboard_;
  }

  /**
   * @brief Getter function for current BT XML filename
   *        （获取当前实际使用的行为树 XML 文件路径）
   * @return string Containing current BT XML filename
   */
  std::string getCurrentBTFilename() const
  {
    return current_bt_xml_filename_;
  }

  /**
   * @brief Getter function for default BT XML filename
   *        （获取默认的行为树 XML 文件路径，来自参数/包内默认文件）
   * @return string Containing default BT XML filename
   */
  std::string getDefaultBTFilename() const
  {
    return default_bt_xml_filename_;
  }

  /**
   * @brief Wrapper function to accept pending goal if a preempt has been requested
   *        （若发生抢占：接受挂起的 goal，使其成为当前 goal）
   * @return Shared pointer to pending action goal
   */
  const std::shared_ptr<const typename ActionT::Goal> acceptPendingGoal()
  {
    return action_server_->accept_pending_goal();
  }

  /**
   * @brief Wrapper function to terminate pending goal if a preempt has been requested
   *        （若发生抢占：拒绝挂起的 goal，继续执行当前 goal）
   */
  void terminatePendingGoal()
  {
    action_server_->terminate_pending_goal();
  }

  /**
   * @brief Wrapper function to get current goal
   *        （获取当前正在执行的 goal）
   * @return Shared pointer to current action goal
   */
  const std::shared_ptr<const typename ActionT::Goal> getCurrentGoal() const
  {
    return action_server_->get_current_goal();
  }

  /**
   * @brief Wrapper function to get pending goal
   *        （获取抢占时新到达、尚未接受的挂起 goal）
   * @return Shared pointer to pending action goal
   */
  const std::shared_ptr<const typename ActionT::Goal> getPendingGoal() const
  {
    return action_server_->get_pending_goal();
  }

  /**
   * @brief Wrapper function to publish action feedback
   *        （发布 action 过程反馈：如机器人当前位姿、剩余路程等）
   */
  void publishFeedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)
  {
    action_server_->publish_feedback(feedback);
  }

  /**
   * @brief Getter function for the current BT tree
   *        （获取当前行为树对象的只读引用，可用于遍历/Groot 发布）
   * @return BT::Tree Current behavior tree
   */
  const BT::Tree & getTree() const
  {
    return tree_;
  }

  /**
   * @brief Function to halt the current tree. It will interrupt the execution of RUNNING nodes
   * by calling their halt() implementation (only for Async nodes that may return RUNNING)
   *        （中断整棵树：从根节点递归 halt，触发所有 RUNNING 节点的清理逻辑
   *         —— 例如取消正在执行的 ROS2 action goal）
   */
  void haltTree()
  {
    tree_.rootNode()->halt();
  }

protected:
  /**
   * @brief Action server callback
   *        （action 服务器的主执行回调：goal 到达后在专用线程中运行，
   *         循环 tick 行为树直到完成/失败/取消，详见 impl 文件）
   */
  void executeCallback();

  // ---- 成员变量说明 ----
  // 对外提供的 action 名称（如 "navigate_to_pose"）。
  std::string action_name_;

  // 底层 action 服务器（nav2_util::SimpleActionServer）。
  std::shared_ptr<ActionServer> action_server_;

  // 当前已构建的行为树。
  BT::Tree tree_;

  // 行为树黑板：goal、路径等共享数据在此流转。
  BT::Blackboard::Ptr blackboard_;

  // 当前/默认的行为树 XML 文件路径（支持运行时切换）。
  std::string current_bt_xml_filename_;
  std::string default_bt_xml_filename_;

  // 行为树引擎：负责加载插件库、从 XML 创建树、驱动 tick 循环。
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;

  // 待动态加载的 BT 插件库(.so)名称列表。
  std::vector<std::string> plugin_lib_names_;

  // 供 BT 节点使用的普通 ROS2 节点（作为黑板 "node" 键分发给各节点，
  // 与父生命周期节点分离，避免生命周期状态影响 BT 内部通信）。
  rclcpp::Node::SharedPtr client_node_;

  // 父生命周期节点的弱引用（不延长生命周期）。
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // 共享时钟（打时间戳用）。
  rclcpp::Clock::SharedPtr clock_;

  // 日志器。
  rclcpp::Logger logger_{rclcpp::get_logger("BtActionServer")};

  // BT 状态变化日志发布器：把树的状态变迁发到 /behavior_tree_log 话题。
  std::unique_ptr<RosTopicLogger> topic_logger_;

  // 单次树 tick 循环的周期预算（毫秒）。
  std::chrono::milliseconds bt_loop_duration_;

  // 服务器应答默认超时（毫秒，写入黑板供各 BT 节点读取）。
  std::chrono::milliseconds default_server_timeout_;

  // BT 节点等待服务器上线的默认超时（毫秒）。
  std::chrono::milliseconds wait_for_service_timeout_;

  // should the BT be reloaded even if the same xml filename is requested?
  // 即使请求加载的是同名 XML 文件也强制重建树（便于开发调试时改文件即生效）。
  bool always_reload_bt_xml_ = false;

  // User-provided callbacks
  // 用户注入的四个业务回调（见文件头说明）。
  OnGoalReceivedCallback on_goal_received_callback_;
  OnLoopCallback on_loop_callback_;
  OnPreemptCallback on_preempt_callback_;
  OnCompletionCallback on_completion_callback_;
};

}  // namespace nav2_behavior_tree

#include <nav2_behavior_tree/bt_action_server_impl.hpp>  // NOLINT(build/include_order)
#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_HPP_
