// Copyright (c) 2025 Open Navigation LLC
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

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/compute_and_track_route_action.hpp"

namespace nav2_behavior_tree
{

ComputeAndTrackRouteAction::ComputeAndTrackRouteAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
// 初始化 BtActionNode<Action> 对象，读取参数并准备运行时依赖。
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
  nav_msgs::msg::Path empty_path;
  nav2_msgs::msg::Route empty_route;
  feedback_.last_node_id = 0;
  feedback_.next_node_id = 0;
  feedback_.current_edge_id = 0;
  feedback_.route = empty_route;
  feedback_.path = empty_path;
  feedback_.rerouted = false;
}

// 在每次 tick 时读取输入并准备下一步执行状态。
void ComputeAndTrackRouteAction::on_tick()
{
  bool use_poses = false, use_start = false;
// 实现该逻辑，并处理相关状态转换。
  getInput("use_poses", use_poses);
  if (use_poses) {
    goal_.use_poses = true;
// 实现该逻辑，并处理相关状态转换。
    getInput("goal", goal_.goal);

    goal_.use_start = false;
// 实现该逻辑，并处理相关状态转换。
    getInput("use_start", use_start);
    if (use_start) {
// 实现该逻辑，并处理相关状态转换。
      getInput("start", goal_.start);
      goal_.use_start = true;
    }
  } else {
// 实现该逻辑，并处理相关状态转换。
    getInput("start_id", goal_.start_id);
// 实现该逻辑，并处理相关状态转换。
    getInput("goal_id", goal_.goal_id);
    goal_.use_start = false;
    goal_.use_poses = false;
  }
}

// 初始化 NodeStatus ComputeAndTrackRouteAction::on_success 对象，读取参数并准备运行时依赖。
BT::NodeStatus ComputeAndTrackRouteAction::on_success()
{
// 实现该逻辑，并处理相关状态转换。
  resetFeedbackAndOutputPorts();
  setOutput("execution_duration", result_.result->execution_duration);
  return BT::NodeStatus::SUCCESS;
}

// 初始化 NodeStatus ComputeAndTrackRouteAction::on_aborted 对象，读取参数并准备运行时依赖。
BT::NodeStatus ComputeAndTrackRouteAction::on_aborted()
{
// 实现该逻辑，并处理相关状态转换。
  resetFeedbackAndOutputPorts();
// 实现该逻辑，并处理相关状态转换。
  setOutput("execution_duration", builtin_interfaces::msg::Duration());
  return BT::NodeStatus::FAILURE;
}

// 初始化 NodeStatus ComputeAndTrackRouteAction::on_cancelled 对象，读取参数并准备运行时依赖。
BT::NodeStatus ComputeAndTrackRouteAction::on_cancelled()
{
// 实现该逻辑，并处理相关状态转换。
  resetFeedbackAndOutputPorts();
// 实现该逻辑，并处理相关状态转换。
  setOutput("execution_duration", builtin_interfaces::msg::Duration());
  return BT::NodeStatus::SUCCESS;
}

void ComputeAndTrackRouteAction::on_wait_for_result(
  std::shared_ptr<const Action::Feedback> feedback)
{
  // Check for request updates to the goal
  bool use_poses = false, use_start = false;
// 实现该逻辑，并处理相关状态转换。
  getInput("use_start", use_start);
// 实现该逻辑，并处理相关状态转换。
  getInput("use_poses", use_poses);

  if (goal_.use_poses != use_poses) {
    goal_updated_ = true;
  }

  if (use_poses) {
    geometry_msgs::msg::PoseStamped goal;
// 实现该逻辑，并处理相关状态转换。
    getInput("goal", goal);
    if (goal_.goal != goal) {
      goal_updated_ = true;
    }

    if (goal_.use_start != use_start) {
      goal_updated_ = true;
    }
    if (use_start) {
      geometry_msgs::msg::PoseStamped start;
// 实现该逻辑，并处理相关状态转换。
      getInput("start", start);
      if (goal_.start != start) {
        goal_updated_ = true;
      }
    }
  } else {
    // Check if the start and goal IDs have changed
    unsigned int start_id = 0;
    unsigned int goal_id = 0;
// 实现该逻辑，并处理相关状态转换。
    getInput("start_id", start_id);
// 实现该逻辑，并处理相关状态转换。
    getInput("goal_id", goal_id);
    if (goal_.start_id != start_id) {
      goal_updated_ = true;
    }
    if (goal_.goal_id != goal_id) {
      goal_updated_ = true;
    }
  }

  // If we're updating the request, we need to fully update the goal
  // Easier to call on_tick() again than to duplicate the code
  if (goal_updated_) {
// 在每次 tick 时读取输入并准备下一步执行状态。
    on_tick();
  }

  if (feedback) {
    feedback_ = *feedback;
// 实现该逻辑，并处理相关状态转换。
    setOutput("last_node_id", feedback_.last_node_id);
// 实现该逻辑，并处理相关状态转换。
    setOutput("next_node_id", feedback_.next_node_id);
// 实现该逻辑，并处理相关状态转换。
    setOutput("current_edge_id", feedback_.current_edge_id);
// 实现该逻辑，并处理相关状态转换。
    setOutput("route", feedback_.route);
// 实现该逻辑，并处理相关状态转换。
    setOutput("path", feedback_.path);
// 实现该逻辑，并处理相关状态转换。
    setOutput("rerouted", feedback_.rerouted);
  }
}

// 实现该逻辑，并处理相关状态转换。
void ComputeAndTrackRouteAction::resetFeedbackAndOutputPorts()
{
  nav_msgs::msg::Path empty_path;
  nav2_msgs::msg::Route empty_route;
  feedback_.last_node_id = 0;
  feedback_.next_node_id = 0;
  feedback_.current_edge_id = 0;
  feedback_.route = empty_route;
  feedback_.path = empty_path;
  feedback_.rerouted = false;
// 实现该逻辑，并处理相关状态转换。
  setOutput("last_node_id", feedback_.last_node_id);
// 实现该逻辑，并处理相关状态转换。
  setOutput("next_node_id", feedback_.next_node_id);
// 实现该逻辑，并处理相关状态转换。
  setOutput("current_edge_id", feedback_.current_edge_id);
// 实现该逻辑，并处理相关状态转换。
  setOutput("route", feedback_.route);
// 实现该逻辑，并处理相关状态转换。
  setOutput("path", feedback_.path);
// 实现该逻辑，并处理相关状态转换。
  setOutput("rerouted", feedback_.rerouted);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 将当前节点类型注册到 BehaviorTreeFactory，使 XML 能直接实例化该节点。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputeAndTrackRouteAction>(
        name, "compute_and_track_route", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputeAndTrackRouteAction>(
    "ComputeAndTrackRoute", builder);
}
