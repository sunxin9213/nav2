// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Pablo Iñigo Blasco
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
// 【文件说明】controller_selector_node.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 ControllerSelector 行为树节点：动态选择本次导航使用哪个控制器
// （如 "FollowPath" / "GracefulController"）。选择来源：
//   1. 订阅外部话题（默认 "controller_selector"）——rviz/上位机运行时下发；
//   2. XML 中的 default_controller 端口——话题无输入时的默认值。
// 选中的控制器名经输出端口 selected_controller 写入黑板，
// 供下游 FollowPath 节点的 controller_id 端口使用。
// 与 PlannerSelector 结构完全相同（同族节点）。
//
// 【行为树核心概念】
// - 同步动作节点（BT::SyncActionNode）：单次 tick 完成。
// - "必须选择模式"：话题与默认值都为空时返回 FAILURE。
// - XML 使用示例：
//   <ControllerSelector selected_controller="{selected_controller}"
//                       default_controller="FollowPath" topic_name="controller_selector"/>
// =============================================================================

#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/controller_selector_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

// 构造函数：创建 ROS2 订阅监听外部下发的控制器选择。
ControllerSelector::ControllerSelector(
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

  // QoS：KeepLast(1) 只留最新；transient_local + reliable 持久可靠——
  // 后加入的订阅者也能收到最后一次下发的选择。
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  // 订阅选择话题（String 消息），回调挂到专属回调组。
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  controller_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
    topic_name_,
    qos,
    std::bind(&ControllerSelector::callbackControllerSelect, this, _1),
    sub_option);
}

// tick 主函数：单次完成选择并输出。
BT::NodeStatus ControllerSelector::tick()
{
  // 泵取回调：刷入话题上待处理的最新选择。
  callback_group_executor_.spin_some();

  // This behavior always use the last selected controller received from the topic input.
  // When no input is specified it uses the default controller.
  // If the default controller is not specified then we work in "required controller mode":
  // In this mode, the behavior returns failure if the controller selection is not received from
  // the topic input.
  // （优先用话题最近下发；没有则用默认值；都没有则失败。）
  if (last_selected_controller_.empty()) {
    std::string default_controller;
    getInput("default_controller", default_controller);
    if (default_controller.empty()) {
      return BT::NodeStatus::FAILURE;
    } else {
      last_selected_controller_ = default_controller;
    }
  }

  // 把选中的控制器名写入输出端口（黑板）。
  setOutput("selected_controller", last_selected_controller_);

  return BT::NodeStatus::SUCCESS;
}

// 话题回调：缓存外部下发的控制器名。
void
ControllerSelector::callbackControllerSelect(const std_msgs::msg::String::SharedPtr msg)
{
  last_selected_controller_ = msg->data;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "ControllerSelector" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ControllerSelector>("ControllerSelector");
}
