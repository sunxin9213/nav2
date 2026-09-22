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
#include "nav2_behavior_tree/plugins/control/recovery_node.hpp"

namespace nav2_behavior_tree
{

RecoveryNode::RecoveryNode(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf),
  current_child_idx_(0),
  number_of_retries_(1),
// 实现该逻辑，并处理相关状态转换。
  retry_count_(0)
{
// 实现该逻辑，并处理相关状态转换。
  getInput("number_of_retries", number_of_retries_);
}

// 初始化 NodeStatus RecoveryNode::tick 对象，读取参数并准备运行时依赖。
BT::NodeStatus RecoveryNode::tick()
{
  const unsigned children_count = children_nodes_.size();

  if (children_count != 2) {
// 初始化 BehaviorTreeException 对象，读取参数并准备运行时依赖。
    throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
  }

// 初始化 NodeStatus::RUNNING); 对象，读取参数并准备运行时依赖。
  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx_ < children_count && retry_count_ <= number_of_retries_) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    if (current_child_idx_ == 0) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          {
// 立即中断当前节点执行，并让上游状态按树结构继续传播。
            halt();
            return BT::NodeStatus::SUCCESS;
          }

        case BT::NodeStatus::FAILURE:
          {
            if (retry_count_ < number_of_retries_) {
// 实现该逻辑，并处理相关状态转换。
              ControlNode::haltChild(0);
              current_child_idx_++;
              break;
            } else {
// 立即中断当前节点执行，并让上游状态按树结构继续传播。
              halt();
              return BT::NodeStatus::FAILURE;
            }
          }

        case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }

        default:
          {
// 初始化 LogicError 对象，读取参数并准备运行时依赖。
            throw BT::LogicError("A child node must never return IDLE");
          }
      }  // end switch

    } else if (current_child_idx_ == 1) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          {
// 实现该逻辑，并处理相关状态转换。
            ControlNode::haltChild(1);
            retry_count_++;
            current_child_idx_--;
          }
          break;

        case BT::NodeStatus::FAILURE:
          {
// 立即中断当前节点执行，并让上游状态按树结构继续传播。
            halt();
            return BT::NodeStatus::FAILURE;
          }

        case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }

        default:
          {
// 初始化 LogicError 对象，读取参数并准备运行时依赖。
            throw BT::LogicError("A child node must never return IDLE");
          }
      }  // end switch
    }
  }  // end while loop

// 立即中断当前节点执行，并让上游状态按树结构继续传播。
  halt();
  return BT::NodeStatus::FAILURE;
}

// 立即中断当前节点执行，并让上游状态按树结构继续传播。
void RecoveryNode::halt()
{
// 立即中断当前节点执行，并让上游状态按树结构继续传播。
  ControlNode::halt();
  retry_count_ = 0;
  current_child_idx_ = 0;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 将当前节点类型注册到 BehaviorTreeFactory，使 XML 能直接实例化该节点。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RecoveryNode>("RecoveryNode");
}
