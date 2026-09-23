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
// 【文件说明】planner_selector_node.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 PlannerSelector 行为树节点：动态选择本次导航使用哪个全局规划器
// （如 "GridBased" / "ThetaStar"）。选择来源有两个：
//   1. 订阅外部话题（默认 "planner_selector"）——外部程序（如 rviz 按钮、
//      上位机）运行时下发规划器名；
//   2. XML 中的 default_planner 端口——话题无输入时的默认值。
// 选中的规划器名通过输出端口 selected_planner 写入黑板，
// 供下游 ComputePathToPose 节点的 planner_id 端口使用。
//
// 【行为树核心概念】
// - 本节点是"同步动作节点"（BT::SyncActionNode）：tick 一次即完成，
//   不会返回 RUNNING。
// - 输入端口：topic_name（选择话题名）、default_planner（默认规划器）；
//   输出端口：selected_planner（写黑板）。
// - "必须选择模式"：若既无话题输入又无默认值，返回 FAILURE——
//   强制要求外部指定规划器。
// - XML 使用示例：
//   <PlannerSelector selected_planner="{selected_planner}"
//                    default_planner="GridBased" topic_name="planner_selector"/>
// =============================================================================

#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/planner_selector_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

// 构造函数：创建 ROS2 订阅监听外部下发的规划器选择。
PlannerSelector::PlannerSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  // 从黑板取共享 ROS2 节点。
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  // 创建本节点专属回调组（false = 不加入默认执行器），
  // 订阅回调与本节点私有执行器绑定，tick 中手动泵取。
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  // 读取选择话题名端口。
  getInput("topic_name", topic_name_);

  // QoS 配置：KeepLast(1) 只保留最新一条；
  // transient_local（持久性）+ reliable（可靠）——
  // 晚加入的订阅者也能收到最后一次下发的选择（小话题标配组合）。
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  // 订阅选择话题：消息为 String（规划器名），回调挂到专属回调组。
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  planner_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
    topic_name_,
    qos,
    std::bind(&PlannerSelector::callbackPlannerSelect, this, _1),
    sub_option);
}

// tick 主函数：单次调用即完成选择并输出结果。
BT::NodeStatus PlannerSelector::tick()
{
  // 手动泵取回调：把话题上待处理的最新选择刷进来。
  callback_group_executor_.spin_some();

  // This behavior always use the last selected planner received from the topic input.
  // When no input is specified it uses the default planner.
  // If the default planner is not specified then we work in "required planner mode":
  // In this mode, the behavior returns failure if the planner selection is not received from
  // the topic input.
  // （优先使用话题最近下发的选择；没有则用默认值；
  //  默认值也没有则进入"必须选择模式"——直接返回失败。）
  if (last_selected_planner_.empty()) {
    std::string default_planner;
    getInput("default_planner", default_planner);
    if (default_planner.empty()) {
      // 话题和默认值都为空：无法选择，返回 FAILURE。
      return BT::NodeStatus::FAILURE;
    } else {
      // 使用默认规划器。
      last_selected_planner_ = default_planner;
    }
  }

  // 把选中的规划器名写入输出端口（黑板）。
  setOutput("selected_planner", last_selected_planner_);

  return BT::NodeStatus::SUCCESS;
}

// 话题回调：收到外部下发的规划器名时更新缓存
// （在专属回调组的执行器线程中触发，tick 的 spin_some 里被驱动）。
void
PlannerSelector::callbackPlannerSelect(const std_msgs::msg::String::SharedPtr msg)
{
  last_selected_planner_ = msg->data;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "PlannerSelector" 标签注册进 factory
// （默认构建器即可，构造签名标准）。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PlannerSelector>("PlannerSelector");
}
