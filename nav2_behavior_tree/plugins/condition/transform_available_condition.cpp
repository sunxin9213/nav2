// Copyright (c) 2020 Samsung Research America
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
#include <chrono>
#include <memory>

#include "nav2_behavior_tree/plugins/condition/transform_available_condition.hpp"

using namespace std::chrono_literals; // NOLINT

namespace nav2_behavior_tree
{

TransformAvailableCondition::TransformAvailableCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
// 实现该逻辑，并处理相关状态转换。
  was_found_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

// 实现该逻辑，并处理相关状态转换。
  getInput("child", child_frame_);
// 实现该逻辑，并处理相关状态转换。
  getInput("parent", parent_frame_);

  if (child_frame_.empty() || parent_frame_.empty()) {
    RCLCPP_FATAL(
      node_->get_logger(), "Child frame (%s) or parent frame (%s) were empty.",
      child_frame_.c_str(), parent_frame_.c_str());
// 实现该逻辑，并处理相关状态转换。
    exit(-1);
  }

  RCLCPP_DEBUG(node_->get_logger(), "Initialized an TransformAvailableCondition BT node");
}

// 评估当前条件是否满足 ~TransformAvailableCondition 所代表的判定逻辑。
TransformAvailableCondition::~TransformAvailableCondition()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down TransformAvailableCondition BT node");
}

// 初始化 NodeStatus TransformAvailableCondition::tick 对象，读取参数并准备运行时依赖。
BT::NodeStatus TransformAvailableCondition::tick()
{
  if (was_found_) {
    return BT::NodeStatus::SUCCESS;
  }

  std::string tf_error;
  bool found = tf_->canTransform(
    child_frame_, parent_frame_, tf2::TimePointZero, &tf_error);

  if (found) {
    was_found_ = true;
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Transform from %s to %s was not found, tf error: %s",
    child_frame_.c_str(), parent_frame_.c_str(), tf_error.c_str());

  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 将当前节点类型注册到 BehaviorTreeFactory，使 XML 能直接实例化该节点。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TransformAvailableCondition>("TransformAvailable");
}
