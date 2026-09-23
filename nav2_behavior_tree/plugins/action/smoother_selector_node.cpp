// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Pablo Iñigo Blasco
// Copyright (c) 2022 Owen Hooper
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
// 【文件说明】smoother_selector_node.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 SmootherSelector 行为树节点：动态选择路径平滑器
// （如 "simple_smoother"）。选择来源：订阅外部话题
// （默认 "smoother_selector"）或 XML 中的 default_smoother 端口。
// 选中的平滑器名经输出端口 selected_smoother 写入黑板，
// 供 SmoothPath 节点的 smoother_id 端口使用。
// 与 PlannerSelector 等同族，结构一致（本节点使用专属回调组 +
// 单线程执行器泵取回调）。
//
// 【XML 使用示例】
//   <SmootherSelector selected_smoother="{selected_smoother}"
//                     default_smoother="simple_smoother"/>
// =============================================================================

#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/smoother_selector_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

// 构造函数：创建 ROS2 订阅监听外部下发的平滑器选择。
SmootherSelector::SmootherSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  // 从黑板取共享 ROS2 节点。
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  // 创建本节点专属回调组（false = 不加入默认执行器），隔离回调并发。
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  // 读取选择话题名端口。
  getInput("topic_name", topic_name_);

  // QoS：KeepLast(1) + transient_local + reliable（持久可靠，只留最新）。
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  // 订阅选择话题（String 消息），回调挂到专属回调组。
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  smoother_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
    topic_name_,
    qos,
    std::bind(&SmootherSelector::callbackSmootherSelect, this, _1),
    sub_option);
}

// tick 主函数：单次完成选择并输出。
BT::NodeStatus SmootherSelector::tick()
{
  // 泵取回调：刷入话题上待处理的最新选择。
  callback_group_executor_.spin_some();

  // This behavior always use the last selected smoother received from the topic input.
  // When no input is specified it uses the default smoother.
  // If the default smoother is not specified then we work in "required smoother mode":
  // In this mode, the behavior returns failure if the smoother selection is not received from
  // the topic input.
  // （优先用话题最近下发；没有则用默认值；都没有则失败。）
  if (last_selected_smoother_.empty()) {
    std::string default_smoother;
    getInput("default_smoother", default_smoother);
    if (default_smoother.empty()) {
      return BT::NodeStatus::FAILURE;
    } else {
      last_selected_smoother_ = default_smoother;
    }
  }

  // 把选中的平滑器名写入输出端口（黑板）。
  setOutput("selected_smoother", last_selected_smoother_);

  return BT::NodeStatus::SUCCESS;
}

// 话题回调：缓存外部下发的平滑器名。
void
SmootherSelector::callbackSmootherSelect(const std_msgs::msg::String::SharedPtr msg)
{
  last_selected_smoother_ = msg->data;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "SmootherSelector" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SmootherSelector>("SmootherSelector");
}
