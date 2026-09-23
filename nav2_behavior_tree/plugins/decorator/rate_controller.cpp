// Copyright (c) 2018 Intel Corporation
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
// 【文件说明】rate_controller.cpp
// 【所属模块】nav2_behavior_tree/plugins/decorator —— Nav2 行为树的装饰节点插件
//
// 实现 RateController（频率控制器）装饰节点：限制子节点的执行频率，
// 周期之外的 tick 直接跳过子节点（保持上一状态），周期到达时才 tick 一次。
//
// 【在 Nav2 导航流程中的位置】
// 默认导航树中包裹 ComputePathToPose——重规划不必每个 BT 周期都做，
// 1Hz 重规划已经足够（目标不变时反复规划是浪费）。
//
// 【BehaviorTree.CPP 核心概念：装饰节点（Decorator）】
// - 装饰节点只有一个子节点，用于修改/限制子节点的行为
//  （频率限制、取反、重试等），本类继承 BT::DecoratorNode。
// - tick 决策：
//   a) 首次 tick（first_time_）或子节点处于 RUNNING：透传 tick；
//   b) 距上次执行超过周期 period_ = 1/hz：重置计时器并 tick 子节点；
//   c) 否则：不 tick 子节点，保持本节点当前状态（RUNNING）。
// - 子节点 SUCCESS 时重置计时器，重新开始下一个周期。
// - XML 使用示例：
//   <RateController hz="1.0"><ComputePathToPose .../></RateController>
// =============================================================================

#include <chrono>
#include <string>

#include "nav2_behavior_tree/plugins/decorator/rate_controller.hpp"

namespace nav2_behavior_tree
{

// 构造函数：读取频率端口并换算周期。
RateController::RateController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_time_(false)
{
  // hz：子节点的最大执行频率（赫兹），默认 1.0；周期 = 1/hz 秒。
  double hz = 1.0;
  getInput("hz", hz);
  period_ = 1.0 / hz;
}

// tick 主函数：按周期决定是否透传 tick 给子节点。
BT::NodeStatus RateController::tick()
{
  // 节点从 IDLE 启动（新一轮执行开始）：
  // 重置计时起点，标记"首次执行"。
  if (status() == BT::NodeStatus::IDLE) {
    // Reset the starting point since we're starting a new iteration of
    // the rate controller (moving from IDLE to RUNNING)
    start_ = std::chrono::high_resolution_clock::now();
    first_time_ = true;
  }

  // 本节点进入 RUNNING 状态（周期内保持）。
  setStatus(BT::NodeStatus::RUNNING);

  // Determine how long its been since we've started this iteration
  // 计算距本轮开始（或上次成功）以来的流逝时间。
  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = now - start_;

  typedef std::chrono::duration<float> float_seconds;
  auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

  // The child gets ticked the first time through and any time the period has
  // expired. In addition, once the child begins to run, it is ticked each time
  // 'til completion
  // （三种情况透传 tick：首次、子节点仍在 RUNNING、周期已到。）
  if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    seconds.count() >= period_)
  {
    first_time_ = false;
    // tick 子节点并透传其状态。
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      // 子节点仍在运行：本节点 RUNNING（下个周期继续跟踪它）。
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      // 子节点成功：重置计时器（重新开始下一个周期），透传 SUCCESS。
      case BT::NodeStatus::SUCCESS:
        start_ = std::chrono::high_resolution_clock::now();  // Reset the timer
        return BT::NodeStatus::SUCCESS;

      // 子节点失败：透传 FAILURE。
      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  // 周期未到且子节点未在运行：不 tick 子节点，保持当前状态（RUNNING）。
  return status();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "RateController" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RateController>("RateController");
}
