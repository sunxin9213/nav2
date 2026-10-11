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

// ============================================================================
// 【文件概况】生命周期节点服务客户端
// ----------------------------------------------------------------------------
// 概述: 封装生命周期节点的两个标准服务 <node>/change_state 与 <node>/get_state，
//       用于以编程方式驱动其他生命周期节点的状态迁移（configure/activate/…）。
// 主要内容:
//   1. 两个构造函数: 自建内部节点 / 复用外部节点承载服务客户端
//   2. change_state(transition, timeout): 触发状态迁移（失败抛异常）
//   3. change_state(transition):          简化版（固定 5s 服务等待）
//   4. get_state(timeout):                查询目标节点当前状态 id
// 使用场景: lifecycle_utils 的批量启停函数、lifecycle_manager 的核心依赖。
// ============================================================================

#ifndef NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_
#define NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_util/service_client.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_util
{

/// Helper functions to interact with a lifecycle node.
// 生命周期节点服务客户端：封装 change_state / get_state 两个标准服务，
// 用于以编程方式驱动其他生命周期节点的状态迁移
class LifecycleServiceClient
{
public:
  // 自建内部节点版本（会生成临时客户端节点）
  explicit LifecycleServiceClient(const std::string & lifecycle_node_name);
  // 复用外部节点版本
  LifecycleServiceClient(
    const std::string & lifecycle_node_name,
    rclcpp::Node::SharedPtr parent_node);

  /// Trigger a state change
  /**
   * Throws std::runtime_error on failure
   */
  // 触发状态迁移：transition 为 lifecycle_msgs::msg::Transition 中的迁移 id
  bool change_state(
    const uint8_t transition,  // takes a lifecycle_msgs::msg::Transition id
    const std::chrono::seconds timeout);

  /// Trigger a state change, returning result
  // 触发状态迁移（无限等待响应、固定 5s 等服务版本）
  bool change_state(std::uint8_t transition);

  /// Get the current state as a lifecycle_msgs::msg::State id value
  /**
   * Throws std::runtime_error on failure
   */
  // 查询目标节点当前状态 id（UNCONFIGURED/INACTIVE/ACTIVE 等）
  uint8_t get_state(const std::chrono::seconds timeout = std::chrono::seconds(2));

protected:
  // 承载服务客户端的节点（内部生成或外部传入）
  rclcpp::Node::SharedPtr node_;
  // 两个服务客户端：状态迁移服务与状态查询服务
  ServiceClient<lifecycle_msgs::srv::ChangeState> change_state_;
  ServiceClient<lifecycle_msgs::srv::GetState> get_state_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_
