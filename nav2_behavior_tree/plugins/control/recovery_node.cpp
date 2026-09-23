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
// 【文件说明】recovery_node.cpp
// 【所属模块】nav2_behavior_tree/plugins/control —— Nav2 行为树的控制节点插件
//
// 实现 RecoveryNode（恢复节点）——Nav2 容错导航的核心控制节点。
// 必须恰好有两个子节点：
//   子节点 0 = 主任务（如 PipelineSequence 导航流水线）；
//   子节点 1 = 恢复行为（如 Spin/BackUp/ClearCostmap 组合）。
//
// 【执行语义（状态机）】
//   1. tick 主任务；
//   2. 主任务 SUCCESS -> 整节点 SUCCESS（恢复计数清零）；
//   3. 主任务 FAILURE 且还有重试额度 -> halt 主任务，tick 恢复行为；
//   4. 恢复行为 SUCCESS（脱困成功）-> 重试计数 +1，回到第 1 步重试主任务；
//   5. 恢复行为 FAILURE 或重试额度耗尽 -> 整节点 FAILURE（导航失败）。
// 即：失败后先恢复再重试，最多重试 number_of_retries 次。
//
// 【XML 使用示例】（Nav2 默认导航树顶层结构）
//   <RecoveryNode number_of_retries="6">
//     <PipelineSequence> ... 主导航流水线 ... </PipelineSequence>
//     <ReactiveFallback> ... 恢复行为（Spin/BackUp/清图）... </ReactiveFallback>
//   </RecoveryNode>
// =============================================================================

#include <string>
#include "nav2_behavior_tree/plugins/control/recovery_node.hpp"

namespace nav2_behavior_tree
{

// 构造函数：初始化状态变量并读取重试次数端口。
RecoveryNode::RecoveryNode(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf),
  current_child_idx_(0),
  number_of_retries_(1),
  retry_count_(0)
{
  // number_of_retries：主任务失败后最多恢复+重试的次数（默认 1）。
  getInput("number_of_retries", number_of_retries_);
}

// tick 主函数：实现"失败->恢复->重试"状态机（见文件头说明）。
BT::NodeStatus RecoveryNode::tick()
{
  const unsigned children_count = children_nodes_.size();

  // 结构约束：必须恰好 2 个子节点（主任务 + 恢复行为）。
  if (children_count != 2) {
    throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
  }

  // 标记本节点进入 RUNNING 状态。
  setStatus(BT::NodeStatus::RUNNING);

  // 主循环：还没轮换完两个子节点、且重试额度未耗尽。
  while (current_child_idx_ < children_count && retry_count_ <= number_of_retries_) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    // ---- 当前正在执行子节点 0（主任务）----
    if (current_child_idx_ == 0) {
      switch (child_status) {
        // 主任务成功：halt 自身（复位全部状态），整节点成功。
        case BT::NodeStatus::SUCCESS:
          {
            halt();
            return BT::NodeStatus::SUCCESS;
          }

        // 主任务失败：
        case BT::NodeStatus::FAILURE:
          {
            if (retry_count_ < number_of_retries_) {
              // 还有重试额度：halt 主任务（复位），切换到恢复行为子节点。
              ControlNode::haltChild(0);
              current_child_idx_++;
              break;
            } else {
              // 额度耗尽：halt 自身，整节点失败（导航最终失败）。
              halt();
              return BT::NodeStatus::FAILURE;
            }
          }

        // 主任务仍在运行：本节点继续 RUNNING。
        case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }

        // 非法状态：executeTick 的返回值不可能是 IDLE。
        default:
          {
            throw BT::LogicError("A child node must never return IDLE");
          }
      }  // end switch

    // ---- 当前正在执行子节点 1（恢复行为）----
    } else if (current_child_idx_ == 1) {
      switch (child_status) {
        // 恢复行为成功（已脱困）：halt 恢复分支，重试计数 +1，
        // 切回主任务子节点重新尝试。
        case BT::NodeStatus::SUCCESS:
          {
            ControlNode::haltChild(1);
            retry_count_++;
            current_child_idx_--;
          }
          break;

        // 恢复行为也失败：无计可施，halt 自身，整节点失败。
        case BT::NodeStatus::FAILURE:
          {
            halt();
            return BT::NodeStatus::FAILURE;
          }

        // 恢复行为仍在执行（如 Spin 旋转中）：本节点继续 RUNNING。
        case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }

        default:
          {
            throw BT::LogicError("A child node must never return IDLE");
          }
      }  // end switch
    }
  }  // end while loop

  // 循环因重试额度耗尽而退出：halt 自身并返回失败。
  halt();
  return BT::NodeStatus::FAILURE;
}

// halt 钩子：复位基类（halt 所有子节点）并清零重试状态，
// 下次执行从主任务子节点重新开始。
void RecoveryNode::halt()
{
  ControlNode::halt();
  retry_count_ = 0;
  current_child_idx_ = 0;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "RecoveryNode" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RecoveryNode>("RecoveryNode");
}
