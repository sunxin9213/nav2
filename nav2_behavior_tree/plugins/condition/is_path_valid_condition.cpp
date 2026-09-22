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

#include "nav2_behavior_tree/plugins/condition/is_path_valid_condition.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace nav2_behavior_tree
{

IsPathValidCondition::IsPathValidCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
// 实现 ConditionNode 的控制或装饰逻辑，管理子节点执行顺序和状态转换。
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<nav2_msgs::srv::IsPathValid>("is_path_valid");

  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
// 实现该逻辑，并处理相关状态转换。
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
}

// 初始化 NodeStatus IsPathValidCondition::tick 对象，读取参数并准备运行时依赖。
BT::NodeStatus IsPathValidCondition::tick()
{
  nav_msgs::msg::Path path;
// 实现该逻辑，并处理相关状态转换。
  getInput("path", path);

  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();

  request->path = path;
  auto result = client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, result, server_timeout_) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->is_valid) {
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 将当前节点类型注册到 BehaviorTreeFactory，使 XML 能直接实例化该节点。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsPathValidCondition>("IsPathValid");
}
