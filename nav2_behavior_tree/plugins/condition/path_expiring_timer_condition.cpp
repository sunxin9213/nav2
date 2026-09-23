// Copyright (c) 2022 Joshua Wallace
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
// 【文件说明】path_expiring_timer_condition.cpp
// 【所属模块】nav2_behavior_tree/plugins/condition —— Nav2 行为树的条件节点插件
//
// 实现 PathExpiringTimer 行为树节点："路径过期定时器"——路径更新时
// 重置定时器；若路径超过 seconds 秒没有被更新（过期），返回 SUCCESS。
// 与 TimeExpired 的区别：计时会被"路径变化"重置。
//
// 【在 Nav2 导航流程中的位置】
// 监控路径新鲜度：若重规划长时间没有产出新路径（上游堵死），
// 该条件触发以走异常处理分支。
//
// 【XML 使用示例】
//   <PathExpiringTimer path="{path}" seconds="10"/>
// =============================================================================

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"

#include "nav2_behavior_tree/plugins/condition/path_expiring_timer_condition.hpp"

namespace nav2_behavior_tree
{

// 构造函数：读取周期端口。
PathExpiringTimerCondition::PathExpiringTimerCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  period_(1.0),
  first_time_(true)
{
  // 过期时长（秒），默认 1.0。
  getInput("seconds", period_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

// tick 主函数：路径变化则重置计时；超时未更新则 SUCCESS。
BT::NodeStatus PathExpiringTimerCondition::tick()
{
  // 首次 tick：快照当前路径、启动计时，返回 FAILURE（先开始监控）。
  if (first_time_) {
    getInput("path", prev_path_);
    first_time_ = false;
    start_ = node_->now();
    return BT::NodeStatus::FAILURE;
  }

  // 读取当前路径。
  nav_msgs::msg::Path path;
  getInput("path", path);

  // Reset timer if the path has been updated
  // 路径发生了变化（重规划产出了新路径）：
  // 刷新快照并重置计时起点（路径还是"新鲜"的）。
  if (prev_path_ != path) {
    prev_path_ = path;
    start_ = node_->now();
  }

  // Determine how long its been since we've started this iteration
  // 计算自最近一次路径更新以来的流逝时间。
  auto elapsed = node_->now() - start_;

  // Now, get that in seconds
  auto seconds = elapsed.seconds();

  // 路径尚未过期：FAILURE。
  if (seconds < period_) {
    return BT::NodeStatus::FAILURE;
  }

  // 路径过期：重置计时器并返回 SUCCESS。
  start_ = node_->now();  // Reset the timer
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "PathExpiringTimer" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathExpiringTimerCondition>("PathExpiringTimer");
}
