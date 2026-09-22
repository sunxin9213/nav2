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

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "nav2_behavior_tree/plugins/decorator/goal_updated_controller.hpp"


namespace nav2_behavior_tree
{

GoalUpdatedController::GoalUpdatedController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
// 实现 DecoratorNode 的控制或装饰逻辑，管理子节点执行顺序和状态转换。
: BT::DecoratorNode(name, conf)
{
}

// 初始化 NodeStatus GoalUpdatedController::tick 对象，读取参数并准备运行时依赖。
BT::NodeStatus GoalUpdatedController::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    // Reset since we're starting a new iteration of
    // the goal updated controller (moving from IDLE to RUNNING)

    config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", goals_);
    config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", goal_);

    goal_was_updated_ = true;
  }

// 初始化 NodeStatus::RUNNING); 对象，读取参数并准备运行时依赖。
  setStatus(BT::NodeStatus::RUNNING);

  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  if (goal_ != current_goal || goals_ != current_goals) {
    goal_ = current_goal;
    goals_ = current_goals;
    goal_was_updated_ = true;
  }

  // The child gets ticked the first time through and any time the goal has
  // changed or preempted. In addition, once the child begins to run, it is ticked each time
  // 'til completion
  if ((child_node_->status() == BT::NodeStatus::RUNNING) || goal_was_updated_) {
    goal_was_updated_ = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 将当前节点类型注册到 BehaviorTreeFactory，使 XML 能直接实例化该节点。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalUpdatedController>("GoalUpdatedController");
}
