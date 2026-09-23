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

// =============================================================================
// 【文件说明】round_robin_node.cpp
// 【所属模块】nav2_behavior_tree/plugins/control —— Nav2 行为树的控制节点插件
//
// 实现 RoundRobin（轮转）控制节点：按顺序逐个尝试子节点，
// 失败则轮转到下一个；只要有一个成功即整体成功；
// 全部子节点都连续失败才整体失败。
// 与 Fallback 的区别：Fallback 从头开始逐个尝试，RoundRobin 会记住
// 上次的轮转位置，从失败的地方继续（不会反复从头重试同一个坏节点）。
//
// 【典型用途】
// 恢复策略轮询（多个恢复手段依次尝试）或多个候选方案逐个试探。
//
// 【XML 使用示例】
//   <RoundRobin>
//     <Spin spin_dist="1.57"/>
//     <BackUp backup_dist="0.3" backup_speed="0.05"/>
//     <ClearEntireCostmap .../>
//   </RoundRobin>
// =============================================================================

#include <string>

#include "nav2_behavior_tree/plugins/control/round_robin_node.hpp"

namespace nav2_behavior_tree
{

// 两个构造函数：委托 BT::ControlNode（控制节点基类）。
RoundRobinNode::RoundRobinNode(const std::string & name)
: BT::ControlNode::ControlNode(name, {})
{
}

RoundRobinNode::RoundRobinNode(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

// tick 主函数：实现轮转语义（见文件头说明）。
BT::NodeStatus RoundRobinNode::tick()
{
  const auto num_children = children_nodes_.size();

  // 标记本节点进入 RUNNING 状态。
  setStatus(BT::NodeStatus::RUNNING);

  // 循环条件：连续失败的子节点数还没达到子节点总数。
  // （每轮 tick 至多轮转一圈；全部失败才退出循环。）
  while (num_failed_children_ < num_children) {
    // tick 当前轮转位置的子节点。
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    switch (child_status) {
      // 子节点成功：整体成功。
      case BT::NodeStatus::SUCCESS:
        {
          // Wrap around to the first child
          // 轮转位置推进一格（到末尾则回到开头），为下次执行做准备；
          // 清零失败计数并 halt 全部子节点（复位它们的状态）。
          if (++current_child_idx_ >= num_children) {
            current_child_idx_ = 0;
          }
          num_failed_children_ = 0;
          ControlNode::haltChildren();
          return BT::NodeStatus::SUCCESS;
        }

      // 子节点失败：轮转到下一个子节点继续尝试，失败计数 +1。
      case BT::NodeStatus::FAILURE:
        {
          if (++current_child_idx_ >= num_children) {
            current_child_idx_ = 0;
          }
          num_failed_children_++;
          break;
        }

      // 子节点仍在运行：本节点继续 RUNNING（下个周期从同一子节点继续）。
      case BT::NodeStatus::RUNNING:
        {
          return BT::NodeStatus::RUNNING;
        }

      // 非法状态：抛出异常。
      default:
        {
          throw BT::LogicError("Invalid status return from BT node");
        }
    }
  }

  // 所有子节点在本轮都失败：halt 自身（复位轮转状态），整节点失败。
  halt();
  return BT::NodeStatus::FAILURE;
}

// halt 钩子：复位基类（halt 所有子节点）并复位轮转位置和失败计数。
void RoundRobinNode::halt()
{
  ControlNode::halt();
  current_child_idx_ = 0;
  num_failed_children_ = 0;
}

}  // namespace nav2_behavior_tree

// 节点注册：动态库加载时把 "RoundRobin" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RoundRobinNode>("RoundRobin");
}
