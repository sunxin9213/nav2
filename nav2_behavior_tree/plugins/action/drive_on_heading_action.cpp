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

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/drive_on_heading_action.hpp"

namespace nav2_behavior_tree
{

DriveOnHeadingAction::DriveOnHeadingAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
// 实现该逻辑，并处理相关状态转换。
: BtActionNode<nav2_msgs::action::DriveOnHeading>(xml_tag_name, action_name, conf)
{
  double dist;
// 实现该逻辑，并处理相关状态转换。
  getInput("dist_to_travel", dist);
  double speed;
// 实现该逻辑，并处理相关状态转换。
  getInput("speed", speed);
  double time_allowance;
// 实现该逻辑，并处理相关状态转换。
  getInput("time_allowance", time_allowance);

  // Populate the input message
  goal_.target.x = dist;
  goal_.target.y = 0.0;
  goal_.target.z = 0.0;
  goal_.speed = speed;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 将当前节点类型注册到 BehaviorTreeFactory，使 XML 能直接实例化该节点。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::DriveOnHeadingAction>(
        name, "drive_on_heading", config);
    };

  factory.registerBuilder<nav2_behavior_tree::DriveOnHeadingAction>("DriveOnHeading", builder);
}
