// Copyright (c) 2018 Intel Corporation
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
// 【文件说明】goal_updated_controller.cpp
// 【所属模块】nav2_behavior_tree/plugins/decorator —— Nav2 行为树的装饰节点插件
//
// 实现 GoalUpdatedController（目标更新触发器）装饰节点：仅当黑板上的
// 目标（"goal" / "goals" 键）发生变化（或子节点仍在 RUNNING）时才 tick
// 子节点；目标未变化时不 tick，保持状态。
//
// 【在 Nav2 导航流程中的位置】
// 用于多点导航树：只有目标集变化（新的 NavigateThroughPoses goal 到达）
// 时才重新执行子分支（如重新规划），避免同一目标下反复执行。
//
// 【BehaviorTree.CPP 核心概念】
// - 装饰节点：通过黑板比较实现"事件驱动"的子节点执行。
// - 从黑板读取 "goal"（单目标）和 "goals"（目标数组）两个键，
//   与上一次记录的值比较（PoseStamped/数组的 operator==）。
// - XML 使用示例：
//   <GoalUpdatedController><ComputePathThroughPoses .../></GoalUpdatedController>
// =============================================================================

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "nav2_behavior_tree/plugins/decorator/goal_updated_controller.hpp"


namespace nav2_behavior_tree
{

// 构造函数：无额外初始化。
GoalUpdatedController::GoalUpdatedController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
}

// tick 主函数：目标变化时才透传 tick 给子节点。
BT::NodeStatus GoalUpdatedController::tick()
{
  // 节点从 IDLE 启动（新一轮执行）：快照当前黑板目标，
  // 并标记"目标已更新"（首次总是要执行一次子节点）。
  if (status() == BT::NodeStatus::IDLE) {
    // Reset since we're starting a new iteration of
    // the goal updated controller (moving from IDLE to RUNNING)

    config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", goals_);
    config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", goal_);

    goal_was_updated_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  // 读取黑板当前的目标（可能与快照不同——外部更新了目标）。
  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  // 与快照比较：任一发生变化即认为目标已更新，刷新快照。
  if (goal_ != current_goal || goals_ != current_goals) {
    goal_ = current_goal;
    goals_ = current_goals;
    goal_was_updated_ = true;
  }

  // The child gets ticked the first time through and any time the goal has
  // changed or preempted. In addition, once the child begins to run, it is ticked each time
  // 'til completion
  // （透传条件：子节点仍在 RUNNING，或目标发生了变化。）
  if ((child_node_->status() == BT::NodeStatus::RUNNING) || goal_was_updated_) {
    goal_was_updated_ = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      // 子节点仍在运行：本节点 RUNNING。
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      // 子节点成功/失败：透传对应状态。
      case BT::NodeStatus::SUCCESS:
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  // 目标未变化且子节点未在运行：不 tick 子节点，保持当前状态。
  return status();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "GoalUpdatedController" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalUpdatedController>("GoalUpdatedController");
}
