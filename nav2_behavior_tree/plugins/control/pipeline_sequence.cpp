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

#include <stdexcept>
#include <sstream>
#include <string>

#include "nav2_behavior_tree/plugins/control/pipeline_sequence.hpp"

namespace nav2_behavior_tree
{

// 初始化 PipelineSequence 对象，读取参数并准备运行时依赖。
PipelineSequence::PipelineSequence(const std::string & name)
// 实现 ControlNode 的控制或装饰逻辑，管理子节点执行顺序和状态转换。
: BT::ControlNode(name, {})
{
}

PipelineSequence::PipelineSequence(
  const std::string & name,
  const BT::NodeConfiguration & config)
// 实现 ControlNode 的控制或装饰逻辑，管理子节点执行顺序和状态转换。
: BT::ControlNode(name, config)
{
}

// 初始化 NodeStatus PipelineSequence::tick 对象，读取参数并准备运行时依赖。
BT::NodeStatus PipelineSequence::tick()
{
  for (std::size_t i = 0; i < children_nodes_.size(); ++i) {
    auto status = children_nodes_[i]->executeTick();
    switch (status) {
      case BT::NodeStatus::FAILURE:
// 实现该逻辑，并处理相关状态转换。
        ControlNode::haltChildren();
        last_child_ticked_ = 0;  // reset
        return status;
      case BT::NodeStatus::SUCCESS:
        // do nothing and continue on to the next child. If it is the last child
        // we'll exit the loop and hit the wrap-up code at the end of the method.
        break;
      case BT::NodeStatus::RUNNING:
        if (i >= last_child_ticked_) {
          last_child_ticked_ = i;
          return status;
        }
        // else do nothing and continue on to the next child
        break;
      default:
        std::stringstream error_msg;
        error_msg << "Invalid node status. Received status " << status <<
          "from child " << children_nodes_[i]->name();
// 实现该逻辑，并处理相关状态转换。
        throw std::runtime_error(error_msg.str());
    }
  }
// 实现该逻辑，并处理相关状态转换。
  ControlNode::haltChildren();
  last_child_ticked_ = 0;  // reset
  return BT::NodeStatus::SUCCESS;
}

// 立即中断当前节点执行，并让上游状态按树结构继续传播。
void PipelineSequence::halt()
{
// 初始化 ControlNode::halt 对象，读取参数并准备运行时依赖。
  BT::ControlNode::halt();
  last_child_ticked_ = 0;
}

}  // namespace nav2_behavior_tree

// 将当前节点类型注册到 BehaviorTreeFactory，使 XML 能直接实例化该节点。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PipelineSequence>("PipelineSequence");
}
