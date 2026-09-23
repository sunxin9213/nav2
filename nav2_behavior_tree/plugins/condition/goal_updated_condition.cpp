// Copyright (c) 2020 Aitor Miguel Blanco
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
// 【文件说明】goal_updated_condition.cpp
// 【所属模块】nav2_behavior_tree/plugins/condition —— Nav2 行为树的条件节点插件
//
// 实现 GoalUpdated 行为树节点：检测黑板上的目标（"goal" 单目标 /
// "goals" 目标数组）自上次检查以来是否发生了变化。
// 用于在导航过程中响应新目标（抢占式导航）。
//
// 【在 Nav2 导航流程中的位置】
// 常与 GoalUpdater 装饰器配合：GoalUpdated 条件为真时触发重规划，
// 让机器人转向新目标。
//
// 【BehaviorTree.CPP 核心概念】
// - 条件节点：目标变化 -> SUCCESS；未变化 -> FAILURE。
// - 通过快照比较实现变化检测：首次 tick（IDLE 状态）记录快照并返回
//   FAILURE（首次不算"变化"）；此后每 tick 比较黑板当前值与快照。
// - 无端口：直接读写黑板键，不需要 XML 属性。
// - XML 使用示例：
//   <GoalUpdated/>
// =============================================================================

#include <string>
#include <vector>
#include "nav2_behavior_tree/plugins/condition/goal_updated_condition.hpp"

namespace nav2_behavior_tree
{

// 构造函数：无额外初始化。
GoalUpdatedCondition::GoalUpdatedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{}

// tick 主函数：快照比较检测目标变化。
BT::NodeStatus GoalUpdatedCondition::tick()
{
  // 首次 tick（节点处于 IDLE）：记录当前目标快照，
  // 返回 FAILURE（首次执行不视为"目标更新"事件）。
  if (status() == BT::NodeStatus::IDLE) {
    config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", goals_);
    config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", goal_);
    return BT::NodeStatus::FAILURE;
  }

  // 读取黑板当前的目标值。
  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  // 与快照比较：任一发生变化即目标已更新，
  // 刷新快照并返回 SUCCESS。
  if (goal_ != current_goal || goals_ != current_goals) {
    goal_ = current_goal;
    goals_ = current_goals;
    return BT::NodeStatus::SUCCESS;
  }

  // 目标未变化：FAILURE。
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "GoalUpdated" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalUpdatedCondition>("GoalUpdated");
}
