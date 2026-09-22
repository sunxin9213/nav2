// Copyright (c) 2022 Joshua Wallace
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

#include "nav2_behavior_tree/plugins/action/assisted_teleop_action.hpp"

namespace nav2_behavior_tree
{

AssistedTeleopAction::AssistedTeleopAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
// 实现该逻辑，并处理相关状态转换。
: BtActionNode<nav2_msgs::action::AssistedTeleop>(xml_tag_name, action_name, conf)
{
  double time_allowance;
// 实现该逻辑，并处理相关状态转换。
  getInput("time_allowance", time_allowance);
// 实现该逻辑，并处理相关状态转换。
  getInput("is_recovery", is_recovery_);

  // Populate the input message
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

// 在每次 tick 时读取输入并准备下一步执行状态。
void AssistedTeleopAction::on_tick()
{
  if (is_recovery_) {
// 实现该逻辑，并处理相关状态转换。
    increment_recovery_count();
  }
}

// 初始化 NodeStatus AssistedTeleopAction::on_aborted 对象，读取参数并准备运行时依赖。
BT::NodeStatus AssistedTeleopAction::on_aborted()
{
  return is_recovery_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 将当前节点类型注册到 BehaviorTreeFactory，使 XML 能直接实例化该节点。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::AssistedTeleopAction>(
        name, "assisted_teleop", config);
    };

  factory.registerBuilder<nav2_behavior_tree::AssistedTeleopAction>("AssistedTeleop", builder);
}
