// Copyright (c) 2019 Intel Corporation
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
// 【文件说明】time_expired_condition.cpp
// 【所属模块】nav2_behavior_tree/plugins/condition —— Nav2 行为树的条件节点插件
//
// 实现 TimeExpired 行为树节点：周期性定时器条件——从计时起点开始，
// 每 period 秒"到期"一次（到期时返回 SUCCESS 并重置计时器），
// 未到期返回 FAILURE。
//
// 【在 Nav2 导航流程中的位置】
// 可用于周期性触发的检查/刷新逻辑，或与其他节点组合实现超时控制。
//
// 【BehaviorTree.CPP 与 ROS2 API 要点】
// - 条件节点：到期一次 SUCCESS，其余时间 FAILURE。
// - 输入端口：seconds（周期，秒）。
// - 使用 ROS2 节点时钟（node_->now()）计时；节点从 IDLE 启动时
//   重置计时起点（首次 tick 只启动计时，返回 FAILURE）。
// - XML 使用示例：
//   <TimeExpired seconds="10"/>
// =============================================================================

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"

#include "nav2_behavior_tree/plugins/condition/time_expired_condition.hpp"

namespace nav2_behavior_tree
{

// 构造函数：读取周期端口并记录起始时刻。
TimeExpiredCondition::TimeExpiredCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  period_(1.0)
{
  // 周期（秒），默认 1.0。
  getInput("seconds", period_);
  // 从黑板取共享 ROS2 节点，用它的时钟打时间戳。
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  start_ = node_->now();
}

// tick 主函数：周期到期判定。
BT::NodeStatus TimeExpiredCondition::tick()
{
  // 节点从 IDLE 启动（新一轮执行）：重置计时起点，
  // 本次返回 FAILURE（先开始计时）。
  if (status() == BT::NodeStatus::IDLE) {
    start_ = node_->now();
    return BT::NodeStatus::FAILURE;
  }

  // Determine how long its been since we've started this iteration
  // 计算自计时起点以来的流逝时间。
  auto elapsed = node_->now() - start_;

  // Now, get that in seconds
  auto seconds = elapsed.seconds();

  // 未到期：FAILURE。
  if (seconds < period_) {
    return BT::NodeStatus::FAILURE;
  }

  // 到期：重置计时器（下一个周期重新开始），返回 SUCCESS。
  start_ = node_->now();  // Reset the timer
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "TimeExpired" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TimeExpiredCondition>("TimeExpired");
}
