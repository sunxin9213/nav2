// Copyright (c) 2019 Intel Corporation
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

#include "nav2_behavior_tree/plugins/control/round_robin_node.hpp"

namespace nav2_behavior_tree
{

// 实现 RoundRobinNode 的控制或装饰逻辑，管理子节点执行顺序和状态转换。
RoundRobinNode::RoundRobinNode(const std::string & name)
// 实现 ControlNode::ControlNode 的控制或装饰逻辑，管理子节点执行顺序和状态转换。
: BT::ControlNode::ControlNode(name, {})
{
}

RoundRobinNode::RoundRobinNode(
  const std::string & name,
  const BT::NodeConfiguration & config)
// 实现 ControlNode 的控制或装饰逻辑，管理子节点执行顺序和状态转换。
: BT::ControlNode(name, config)
{
}

// 初始化 NodeStatus RoundRobinNode::tick 对象，读取参数并准备运行时依赖。
BT::NodeStatus RoundRobinNode::tick()
{
  const auto num_children = children_nodes_.size();

// 初始化 NodeStatus::RUNNING); 对象，读取参数并准备运行时依赖。
  setStatus(BT::NodeStatus::RUNNING);

  while (num_failed_children_ < num_children) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    switch (child_status) {
      case BT::NodeStatus::SUCCESS:
        {
          // Wrap around to the first child
          if (++current_child_idx_ >= num_children) {
            current_child_idx_ = 0;
          }
          num_failed_children_ = 0;
// 实现该逻辑，并处理相关状态转换。
          ControlNode::haltChildren();
          return BT::NodeStatus::SUCCESS;
        }

      case BT::NodeStatus::FAILURE:
        {
          if (++current_child_idx_ >= num_children) {
            current_child_idx_ = 0;
          }
          num_failed_children_++;
          break;
        }

      case BT::NodeStatus::RUNNING:
        {
          return BT::NodeStatus::RUNNING;
        }

      default:
        {
// 初始化 LogicError 对象，读取参数并准备运行时依赖。
          throw BT::LogicError("Invalid status return from BT node");
        }
    }
  }

// 立即中断当前节点执行，并让上游状态按树结构继续传播。
  halt();
  return BT::NodeStatus::FAILURE;
}

// 立即中断当前节点执行，并让上游状态按树结构继续传播。
void RoundRobinNode::halt()
{
// 立即中断当前节点执行，并让上游状态按树结构继续传播。
  ControlNode::halt();
  current_child_idx_ = 0;
  num_failed_children_ = 0;
}

}  // namespace nav2_behavior_tree

// 将当前节点类型注册到 BehaviorTreeFactory，使 XML 能直接实例化该节点。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RoundRobinNode>("RoundRobin");
}
