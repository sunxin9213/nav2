// Copyright (c) 2021 Joshua Wallace
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
// 【文件说明】is_path_valid_condition.cpp
// 【所属模块】nav2_behavior_tree/plugins/condition —— Nav2 行为树的条件节点插件
//
// 实现 IsPathValid 行为树节点：调用规划器（nav2_planner）的
// "is_path_valid" ROS2 Service（nav2_msgs::srv::IsPathValid），
// 检查给定路径在当前代价地图上是否仍然无碰撞（路径上有没有新出现的
// 障碍物）。
//
// 【在 Nav2 导航流程中的位置】
// 用于"边走边检查"：若当前路径已被新障碍物阻断，可提前触发重规划，
// 而不是等控制器跟踪失败后再恢复。
//
// 【BehaviorTree.CPP 与 ROS2 API 要点】
// - 条件节点：有效 -> SUCCESS；无效或服务调用失败/超时 -> FAILURE。
// - 输入端口：path（待检查的路径，来自黑板）、server_timeout（毫秒）。
// - 服务的"同步调用"范式：async_send_request 拿到 future，
//   再用 rclcpp::spin_until_future_complete 阻塞等待（受超时限制）——
//   这是 BT 条件节点里调用服务的典型写法（区别于 BtServiceNode 的
//   多周期等待方式）。
// - XML 使用示例：
//   <IsPathValid path="{path}" server_timeout="50"/>
// =============================================================================

#include "nav2_behavior_tree/plugins/condition/is_path_valid_condition.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace nav2_behavior_tree
{

// 构造函数：创建 "is_path_valid" 服务客户端并读取超时。
IsPathValidCondition::IsPathValidCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  // 从黑板取共享 ROS2 节点。
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  // 创建服务客户端（服务名固定为 "is_path_valid"，
  // 由 nav2_planner / nav2_controller 提供）。
  client_ = node_->create_client<nav2_msgs::srv::IsPathValid>("is_path_valid");

  // 从黑板读取默认服务器超时；XML 端口值优先。
  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
}

// tick 主函数：同步调用服务检查路径有效性。
BT::NodeStatus IsPathValidCondition::tick()
{
  // 读取待检查的路径。
  nav_msgs::msg::Path path;
  getInput("path", path);

  // 组装服务请求：填充 path 字段。
  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();

  request->path = path;
  // 异步发送请求，立即返回 future。
  auto result = client_->async_send_request(request);

  // 在超时上限内阻塞等待服务响应
  // （spin_until_future_complete 会泵取回调直到 future 完成或超时）。
  if (rclcpp::spin_until_future_complete(node_, result, server_timeout_) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    // 服务返回：is_valid 为真表示路径仍可通行 -> SUCCESS。
    if (result.get()->is_valid) {
      return BT::NodeStatus::SUCCESS;
    }
  }
  // 路径无效、服务超时或调用失败：FAILURE。
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "IsPathValid" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsPathValidCondition>("IsPathValid");
}
