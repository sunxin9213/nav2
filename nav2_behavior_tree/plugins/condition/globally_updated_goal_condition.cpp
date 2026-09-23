// Copyright (c) 2021 Joshua Wallace
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
// 【文件说明】globally_updated_goal_condition.cpp
// 【所属模块】nav2_behavior_tree/plugins/condition —— Nav2 行为树的条件节点插件
//
// 实现 GlobalUpdatedGoal 行为树节点：检测黑板上的目标
// （"goal" / "goals"）自上次检查以来是否变化。
// 与 GoalUpdated 的区别：首次 tick 返回 SUCCESS（首次执行也视为一次
// "更新事件"，让子分支先跑一遍），此后才是变化时 SUCCESS。
//
// 【在 Nav2 导航流程中的位置】
// 多点导航树（NavigateThroughPoses）的入口条件：新任务到达（或目标
// 数组变化）时触发规划分支。
//
// 【XML 使用示例】
//   <GlobalUpdatedGoal/>
// =============================================================================

#include <vector>
#include <string>

#include "nav2_behavior_tree/plugins/condition/globally_updated_goal_condition.hpp"

namespace nav2_behavior_tree
{

// 构造函数：从黑板取共享 ROS2 节点（备用），first_time 初始为 true。
GloballyUpdatedGoalCondition::GloballyUpdatedGoalCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  first_time(true)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

// tick 主函数：首次返回 SUCCESS，之后做目标变化检测。
BT::NodeStatus GloballyUpdatedGoalCondition::tick()
{
  // 首次 tick：记录目标快照并直接返回 SUCCESS
  // （首次执行视为一次更新事件）。
  if (first_time) {
    first_time = false;
    config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", goals_);
    config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", goal_);
    return BT::NodeStatus::SUCCESS;
  }

  // 读取黑板当前目标并与快照比较。
  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  // 目标变化：刷新快照并返回 SUCCESS。
  if (goal_ != current_goal || goals_ != current_goals) {
    goal_ = current_goal;
    goals_ = current_goals;
    return BT::NodeStatus::SUCCESS;
  }

  // 无变化：FAILURE。
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "GlobalUpdatedGoal" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GloballyUpdatedGoalCondition>("GlobalUpdatedGoal");
}
