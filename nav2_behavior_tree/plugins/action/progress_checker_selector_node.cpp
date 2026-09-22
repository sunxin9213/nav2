// Copyright (c) 2024 Open Navigation LLC
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

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/progress_checker_selector_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

ProgressCheckerSelector::ProgressCheckerSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
// 实现 SyncActionNode 的控制或装饰逻辑，管理子节点执行顺序和状态转换。
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

// 实现该逻辑，并处理相关状态转换。
  getInput("topic_name", topic_name_);

// 初始化 QoS qos 对象，读取参数并准备运行时依赖。
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  progress_checker_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
    topic_name_, qos, std::bind(&ProgressCheckerSelector::callbackProgressCheckerSelect, this, _1));
}

// 初始化 NodeStatus ProgressCheckerSelector::tick 对象，读取参数并准备运行时依赖。
BT::NodeStatus ProgressCheckerSelector::tick()
{
// 实现该逻辑，并处理相关状态转换。
  rclcpp::spin_some(node_);

  // This behavior always use the last selected progress checker received from the topic input.
  // When no input is specified it uses the default goaprogressl checker.
  // If the default progress checker is not specified then we work in
  // "required progress checker mode": In this mode, the behavior returns failure if the progress
  // checker selection is not received from the topic input.
  if (last_selected_progress_checker_.empty()) {
    std::string default_progress_checker;
// 实现该逻辑，并处理相关状态转换。
    getInput("default_progress_checker", default_progress_checker);
    if (default_progress_checker.empty()) {
      return BT::NodeStatus::FAILURE;
    } else {
      last_selected_progress_checker_ = default_progress_checker;
    }
  }

// 实现该逻辑，并处理相关状态转换。
  setOutput("selected_progress_checker", last_selected_progress_checker_);

  return BT::NodeStatus::SUCCESS;
}

void
// 实现该逻辑，并处理相关状态转换。
ProgressCheckerSelector::callbackProgressCheckerSelect(const std_msgs::msg::String::SharedPtr msg)
{
  last_selected_progress_checker_ = msg->data;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 将当前节点类型注册到 BehaviorTreeFactory，使 XML 能直接实例化该节点。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ProgressCheckerSelector>("ProgressCheckerSelector");
}
