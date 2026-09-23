// Copyright (c) 2021 Samsung Research America
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
// 【文件说明】single_trigger_node.cpp
// 【所属模块】nav2_behavior_tree/plugins/decorator —— Nav2 行为树的装饰节点插件
//
// 实现 SingleTrigger（单次触发）装饰节点：只允许子节点被执行一次。
// 第一次 tick 正常透传给子节点；此后无论树怎么重试，
// 本节点都直接返回 FAILURE，子节点不再被 tick。
//
// 【在 Nav2 导航流程中的位置】
// 用于"只需执行一次"的动作，例如导航开始时的一次性初始化、
// 只触发一次的日志/通知，防止 RecoveryNode 重试导致重复执行。
//
// 【BehaviorTree.CPP 核心概念】
// - 装饰节点（BT::DecoratorNode）只有一个子节点，本类改变其执行次数语义。
// - 状态从 IDLE 重新启动时（树被复位后再次运行），first_time_ 重新置
//   true，允许子节点再执行一次。
// - XML 使用示例：
//   <SingleTrigger><ClearEntireCostmap .../></SingleTrigger>
// =============================================================================

#include <chrono>
#include <string>

#include "nav2_behavior_tree/plugins/decorator/single_trigger_node.hpp"

namespace nav2_behavior_tree
{

// 构造函数：first_time_ 初始为 true（首次运行允许触发子节点）。
SingleTrigger::SingleTrigger(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_time_(true)
{
}

// tick 主函数：首次透传子节点，之后一律返回 FAILURE。
BT::NodeStatus SingleTrigger::tick()
{
  // 节点被复位回 IDLE（新一轮执行开始）时，重新允许触发一次。
  if (status() == BT::NodeStatus::IDLE) {
    first_time_ = true;
  }

  // 本节点进入 RUNNING 状态。
  setStatus(BT::NodeStatus::RUNNING);

  // 首次：tick 子节点并透传其状态。
  if (first_time_) {
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      // 子节点仍在运行：本节点 RUNNING（子节点还在第一次执行中）。
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      // 子节点完成（无论成败）：标记"已触发"，
      // 之后本节点不再执行子节点。
      case BT::NodeStatus::SUCCESS:
        first_time_ = false;
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
        first_time_ = false;
        return BT::NodeStatus::FAILURE;

      default:
        first_time_ = false;
        return BT::NodeStatus::FAILURE;
    }
  }

  // 非首次：直接返回 FAILURE（子节点已被执行过，不再触发）。
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "SingleTrigger" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SingleTrigger>("SingleTrigger");
}
