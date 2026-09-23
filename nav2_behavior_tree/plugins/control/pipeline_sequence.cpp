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
// 【文件说明】pipeline_sequence.cpp
// 【所属模块】nav2_behavior_tree/plugins/control —— Nav2 行为树的控制节点插件
//
// 实现 PipelineSequence（流水线序列）控制节点。它是 Nav2 默认导航树的
// 骨架，解决了标准 Sequence 的一个关键缺陷：
//
// 【标准 Sequence 的问题】
// 标准 Sequence 严格"从头到尾"执行：若末尾的 FollowPath 返回 RUNNING，
// 下个周期会从头重新 tick——前面的 ComputePathToPose 每个周期都被重新
// 规划，浪费计算且会打断跟踪。
//
// 【PipelineSequence 的语义（记忆型序列）】
// 从头开始逐个 tick 子节点，但会记住最远推进到的子节点下标
// （last_child_ticked_）：
//   - 子节点 SUCCESS：继续推进到下一个子节点；
//   - 子节点 RUNNING 且比上次更靠后：记录新位置并立即返回 RUNNING，
//     下个周期从已推进的位置继续（前面的节点不再被 tick）；
//   - 子节点 RUNNING 但不比上次靠后：跳过它继续后面的节点；
//   - 任一子节点 FAILURE：halt 全部子节点，复位，整节点 FAILURE。
// 效果：ComputePathToPose（一次成功）+ FollowPath（持续 RUNNING）组成
// "规划一次、持续跟踪"的流水线。
//
// 【XML 使用示例】（Nav2 默认导航树片段）
//   <PipelineSequence>
//     <RateController hz="1.0"><ComputePathToPose .../></RateController>
//     <FollowPath .../>
//   </PipelineSequence>
// =============================================================================

#include <stdexcept>
#include <sstream>
#include <string>

#include "nav2_behavior_tree/plugins/control/pipeline_sequence.hpp"

namespace nav2_behavior_tree
{

// 两个构造函数：委托 BT::ControlNode（控制节点基类，管理子节点列表）。
PipelineSequence::PipelineSequence(const std::string & name)
: BT::ControlNode(name, {})
{
}

PipelineSequence::PipelineSequence(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

// tick 主函数：实现"记忆型序列"语义（见文件头说明）。
BT::NodeStatus PipelineSequence::tick()
{
  // 从第一个子节点开始逐个 tick。
  for (std::size_t i = 0; i < children_nodes_.size(); ++i) {
    auto status = children_nodes_[i]->executeTick();
    switch (status) {
      // 子节点失败：整条流水线失败——halt 所有子节点并复位推进位置。
      case BT::NodeStatus::FAILURE:
        ControlNode::haltChildren();
        last_child_ticked_ = 0;  // reset
        return status;
      // 子节点成功：继续 tick 下一个子节点。若是最后一个子节点，
      // 则跳出循环，走方法末尾的整体成功逻辑。
      case BT::NodeStatus::SUCCESS:
        // do nothing and continue on to the next child. If it is the last child
        // we'll exit the loop and hit the wrap-up code at the end of the method.
        break;
      // 子节点仍在运行：分两种情况。
      case BT::NodeStatus::RUNNING:
        if (i >= last_child_ticked_) {
          // 本次推进到了更靠后的位置（或相同）：记录新位置并返回
          // RUNNING——下个周期从这里继续，前面的子节点不再 tick。
          last_child_ticked_ = i;
          return status;
        }
        // else do nothing and continue on to the next child
        // 位置比已记录的更靠前（前面的旧节点仍在 RUNNING）：
        // 跳过它，继续 tick 后面的子节点。
        break;
      // 非法状态（如 IDLE）：抛异常交由树引擎处理。
      default:
        std::stringstream error_msg;
        error_msg << "Invalid node status. Received status " << status <<
          "from child " << children_nodes_[i]->name();
        throw std::runtime_error(error_msg.str());
    }
  }
  // 所有子节点都成功：halt 全部子节点（复位它们的状态）、
  // 复位推进位置，整节点返回 SUCCESS。
  ControlNode::haltChildren();
  last_child_ticked_ = 0;  // reset
  return BT::NodeStatus::SUCCESS;
}

// halt 钩子：被中断时复位基类状态和推进位置。
void PipelineSequence::halt()
{
  BT::ControlNode::halt();
  last_child_ticked_ = 0;
}

}  // namespace nav2_behavior_tree

// 节点注册：动态库加载时把 "PipelineSequence" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PipelineSequence>("PipelineSequence");
}
