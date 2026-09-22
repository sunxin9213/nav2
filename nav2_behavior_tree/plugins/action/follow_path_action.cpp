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

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/follow_path_action.hpp"

namespace nav2_behavior_tree
{

FollowPathAction::FollowPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
// 实现该逻辑，并处理相关状态转换。
: BtActionNode<nav2_msgs::action::FollowPath>(xml_tag_name, action_name, conf)
{
}

// 在每次 tick 时读取输入并准备下一步执行状态。
void FollowPathAction::on_tick()
{
// 实现该逻辑，并处理相关状态转换。
  getInput("path", goal_.path);
// 实现该逻辑，并处理相关状态转换。
  getInput("controller_id", goal_.controller_id);
// 实现该逻辑，并处理相关状态转换。
  getInput("goal_checker_id", goal_.goal_checker_id);
}

void FollowPathAction::on_wait_for_result(
  std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback>/*feedback*/)
{
  nav_msgs::msg::Path new_path;
// 实现该逻辑，并处理相关状态转换。
  getInput("path", new_path);

  // Check if it is not same with the current one
  if (goal_.path != new_path && new_path != nav_msgs::msg::Path()) {
    // the action server on the next loop iteration
    goal_.path = new_path;
    goal_updated_ = true;
  }

  std::string new_controller_id;
// 实现该逻辑，并处理相关状态转换。
  getInput("controller_id", new_controller_id);

  if (goal_.controller_id != new_controller_id) {
    goal_.controller_id = new_controller_id;
    goal_updated_ = true;
  }

  std::string new_goal_checker_id;
// 实现该逻辑，并处理相关状态转换。
  getInput("goal_checker_id", new_goal_checker_id);

  if (goal_.goal_checker_id != new_goal_checker_id) {
    goal_.goal_checker_id = new_goal_checker_id;
    goal_updated_ = true;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 将当前节点类型注册到 BehaviorTreeFactory，使 XML 能直接实例化该节点。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::FollowPathAction>(
        name, "follow_path", config);
    };

  factory.registerBuilder<nav2_behavior_tree::FollowPathAction>(
    "FollowPath", builder);
}
