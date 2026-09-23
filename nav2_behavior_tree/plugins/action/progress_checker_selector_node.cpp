// Copyright (c) 2024 Open Navigation LLC
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
// 【文件说明】progress_checker_selector_node.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 ProgressCheckerSelector 行为树节点：动态选择"进度检查器"
// （progress checker，判断机器人是否在正常取得进度、有无卡死）。
// 选择来源：订阅外部话题（默认 "progress_checker_selector"）或
// XML 中的 default_progress_checker 端口。
// 选中的检查器名经输出端口 selected_progress_checker 写入黑板，
// 供 FollowPath 节点的 progress_checker_id 端口使用。
// 与 PlannerSelector/ControllerSelector/GoalCheckerSelector 同族。
//
// 【行为树核心概念】
// - 同步动作节点：单次 tick 完成。
// - "必须选择模式"：话题与默认值都为空时返回 FAILURE。
// - XML 使用示例：
//   <ProgressCheckerSelector selected_progress_checker="{selected_progress_checker}"
//                            default_progress_checker="progress_checker"/>
// =============================================================================

#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/progress_checker_selector_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

// 构造函数：创建 ROS2 订阅监听外部下发的进度检查器选择。
ProgressCheckerSelector::ProgressCheckerSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  // 从黑板取共享 ROS2 节点。
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // 读取选择话题名端口。
  getInput("topic_name", topic_name_);

  // QoS：KeepLast(1) + transient_local + reliable（持久可靠，只留最新）。
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  // 订阅选择话题（String 消息）。
  progress_checker_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
    topic_name_, qos, std::bind(&ProgressCheckerSelector::callbackProgressCheckerSelect, this, _1));
}

// tick 主函数：单次完成选择并输出。
BT::NodeStatus ProgressCheckerSelector::tick()
{
  // 泵取节点上的挂起回调。
  rclcpp::spin_some(node_);

  // This behavior always use the last selected progress checker received from the topic input.
  // When no input is specified it uses the default goaprogressl checker.
  // If the default progress checker is not specified then we work in
  // "required progress checker mode": In this mode, the behavior returns failure if the progress
  // checker selection is not received from the topic input.
  // （优先用话题最近下发；没有则用默认值；都没有则失败。）
  if (last_selected_progress_checker_.empty()) {
    std::string default_progress_checker;
    getInput("default_progress_checker", default_progress_checker);
    if (default_progress_checker.empty()) {
      return BT::NodeStatus::FAILURE;
    } else {
      last_selected_progress_checker_ = default_progress_checker;
    }
  }

  // 把选中的检查器名写入输出端口（黑板）。
  setOutput("selected_progress_checker", last_selected_progress_checker_);

  return BT::NodeStatus::SUCCESS;
}

// 话题回调：缓存外部下发的检查器名。
void
ProgressCheckerSelector::callbackProgressCheckerSelect(const std_msgs::msg::String::SharedPtr msg)
{
  last_selected_progress_checker_ = msg->data;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "ProgressCheckerSelector" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ProgressCheckerSelector>("ProgressCheckerSelector");
}
