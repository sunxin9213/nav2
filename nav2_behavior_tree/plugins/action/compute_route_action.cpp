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

#include "nav2_behavior_tree/plugins/action/compute_route_action.hpp"

namespace nav2_behavior_tree
{

ComputeRouteAction::ComputeRouteAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
// 初始化 BtActionNode<Action> 对象，读取参数并准备运行时依赖。
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

// 在每次 tick 时读取输入并准备下一步执行状态。
void ComputeRouteAction::on_tick()
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

// 初始化 NodeStatus ComputeRouteAction::on_success 对象，读取参数并准备运行时依赖。
BT::NodeStatus ComputeRouteAction::on_success()
{
  setOutput("path", result_.result->path);
  setOutput("route", result_.result->route);
  setOutput("planning_time", result_.result->planning_time);
  return BT::NodeStatus::SUCCESS;
}

// 实现该逻辑，并处理相关状态转换。
void ComputeRouteAction::resetPorts()
{
  nav_msgs::msg::Path empty_path;
// 实现该逻辑，并处理相关状态转换。
  setOutput("path", empty_path);
  nav2_msgs::msg::Route empty_route;
// 实现该逻辑，并处理相关状态转换。
  setOutput("route", empty_route);
// 实现该逻辑，并处理相关状态转换。
  setOutput("planning_time", builtin_interfaces::msg::Duration());
}

// 初始化 NodeStatus ComputeRouteAction::on_aborted 对象，读取参数并准备运行时依赖。
BT::NodeStatus ComputeRouteAction::on_aborted()
{
// 实现该逻辑，并处理相关状态转换。
  resetPorts();
  return BT::NodeStatus::FAILURE;
}

// 初始化 NodeStatus ComputeRouteAction::on_cancelled 对象，读取参数并准备运行时依赖。
BT::NodeStatus ComputeRouteAction::on_cancelled()
{
// 实现该逻辑，并处理相关状态转换。
  resetPorts();
  return BT::NodeStatus::SUCCESS;
}

// 立即中断当前节点执行，并让上游状态按树结构继续传播。
void ComputeRouteAction::halt()
{
// 实现该逻辑，并处理相关状态转换。
  resetPorts();
// 立即中断当前节点执行，并让上游状态按树结构继续传播。
  BtActionNode::halt();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 将当前节点类型注册到 BehaviorTreeFactory，使 XML 能直接实例化该节点。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputeRouteAction>(
        name, "compute_route", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputeRouteAction>(
    "ComputeRoute", builder);
}
