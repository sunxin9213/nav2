// Copyright (c) 2021 Samsung Research America
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
#include <vector>

#include "nav2_behavior_tree/plugins/action/compute_path_through_poses_action.hpp"

namespace nav2_behavior_tree
{

ComputePathThroughPosesAction::ComputePathThroughPosesAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
// 实现该逻辑，并处理相关状态转换。
: BtActionNode<nav2_msgs::action::ComputePathThroughPoses>(xml_tag_name, action_name, conf)
{
}

// 在每次 tick 时读取输入并准备下一步执行状态。
void ComputePathThroughPosesAction::on_tick()
{
// 实现该逻辑，并处理相关状态转换。
  getInput("goals", goal_.goals);
// 实现该逻辑，并处理相关状态转换。
  getInput("planner_id", goal_.planner_id);
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }
}

// 初始化 NodeStatus ComputePathThroughPosesAction::on_success 对象，读取参数并准备运行时依赖。
BT::NodeStatus ComputePathThroughPosesAction::on_success()
{
  setOutput("path", result_.result->path);
  return BT::NodeStatus::SUCCESS;
}

// 初始化 NodeStatus ComputePathThroughPosesAction::on_aborted 对象，读取参数并准备运行时依赖。
BT::NodeStatus ComputePathThroughPosesAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
// 实现该逻辑，并处理相关状态转换。
  setOutput("path", empty_path);
  return BT::NodeStatus::FAILURE;
}

// 初始化 NodeStatus ComputePathThroughPosesAction::on_cancelled 对象，读取参数并准备运行时依赖。
BT::NodeStatus ComputePathThroughPosesAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
// 实现该逻辑，并处理相关状态转换。
  setOutput("path", empty_path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 将当前节点类型注册到 BehaviorTreeFactory，使 XML 能直接实例化该节点。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputePathThroughPosesAction>(
        name, "compute_path_through_poses", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputePathThroughPosesAction>(
    "ComputePathThroughPoses", builder);
}
