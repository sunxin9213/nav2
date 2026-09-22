// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#ifndef TEST_DUMMY_TREE_NODE_HPP_
#define TEST_DUMMY_TREE_NODE_HPP_

#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/action_node.h>

namespace nav2_behavior_tree
{

/**
 * @brief A Dummy TreeNode to be used as a child for testing nodes
 * Returns the current status on tick without any execution logic
 */
class DummyNode : public BT::ActionNodeBase
{
public:
// 实现 DummyNode 的控制或装饰逻辑，管理子节点执行顺序和状态转换。
  DummyNode()
// 初始化 ActionNodeBase 对象，读取参数并准备运行时依赖。
  : BT::ActionNodeBase("dummy", {})
  {
  }

// 初始化 NodeStatus status) 对象，读取参数并准备运行时依赖。
  void changeStatus(BT::NodeStatus status)
  {
// 实现该逻辑，并处理相关状态转换。
    setStatus(status);
  }

// 初始化 NodeStatus executeTick 对象，读取参数并准备运行时依赖。
  BT::NodeStatus executeTick() override
  {
    return tick();
  }

// 初始化 NodeStatus tick 对象，读取参数并准备运行时依赖。
  BT::NodeStatus tick() override
  {
    return status();
  }

// 立即中断当前节点执行，并让上游状态按树结构继续传播。
  void halt() override
  {
  }
};

}  // namespace nav2_behavior_tree

#endif  // TEST_DUMMY_TREE_NODE_HPP_
