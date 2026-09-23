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

/*
 * =============================================================================
 * 文件：lifecycle_manager_client.cpp（nav2_lifecycle_manager 模块）
 * =============================================================================
 * 功能概述：
 *   实现 LifecycleManagerClient——生命周期管理器的"客户端"辅助类。
 *   供其它节点（如行为树插件、RViz 插件、测试脚本等）以服务调用的方式
 *   远程操控 lifecycle_manager：
 *     - startup()/shutdown()/pause()/resume()/reset()
 *         → 调用 <manager>/manage_nodes（nav2_msgs/srv/ManageLifecycleNodes）；
 *     - is_active()
 *         → 调用 <manager>/is_active（std_srvs/srv/Trigger），
 *           返回 ACTIVE / INACTIVE / TIMEOUT 三种系统状态。
 *   所有服务调用基于父节点的 context/executor（nav2_util::ServiceClient 封装），
 *   在给定的超时内等待服务可用并同步等待结果。
 *
 * 在整个导航流程中的位置：
 *   属于管理平面（control plane）的客户端工具，不参与数据链路
 *   （bt_navigator → planner_server → controller_server → velocity_smoother），
 *   但行为树中的 lifecycle 相关节点（如管家插件）会借助它启停导航系统。
 * =============================================================================
 */

#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_lifecycle_manager
{
using nav2_util::geometry_utils::orientationAroundZAxis;

// 构造函数：name 为目标生命周期管理器（如 "lifecycle_manager"）的节点名。
// 基于该名称拼出两个服务名，并在传入的父节点上创建服务客户端；
// 这样可以复用父节点的 ROS 2 context、日志与 executor，无需新建节点。
LifecycleManagerClient::LifecycleManagerClient(
  const std::string & name,
  std::shared_ptr<rclcpp::Node> parent_node)
{
  // 根据管理器名称构造两个服务名，并复用父节点的 ROS 2 context、日志和 executor。
  manage_service_name_ = name + std::string("/manage_nodes");
  active_service_name_ = name + std::string("/is_active");

  // Use parent node for service call and logging
  node_ = parent_node;

  // Create the service clients
  manager_client_ = std::make_shared<nav2_util::ServiceClient<ManageLifecycleNodes>>(
    manage_service_name_, node_);
  is_active_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::Trigger>>(
    active_service_name_, node_);
}

bool
// 发送 STARTUP 命令，请求管理器配置并激活全部受管节点。
LifecycleManagerClient::startup(const std::chrono::nanoseconds timeout)
{
  return callService(ManageLifecycleNodes::Request::STARTUP, timeout);
}

bool
// 发送 SHUTDOWN 命令，请求管理器逆序停用、清理并关闭节点。
LifecycleManagerClient::shutdown(const std::chrono::nanoseconds timeout)
{
  return callService(ManageLifecycleNodes::Request::SHUTDOWN, timeout);
}

// 发送 PAUSE 命令，请求管理器停用（deactivate）全部受管节点，系统可随后 resume。
bool
LifecycleManagerClient::pause(const std::chrono::nanoseconds timeout)
{
  return callService(ManageLifecycleNodes::Request::PAUSE, timeout);
}

// 发送 RESUME 命令，请求管理器重新激活（activate）此前被暂停的节点。
bool
LifecycleManagerClient::resume(const std::chrono::nanoseconds timeout)
{
  return callService(ManageLifecycleNodes::Request::RESUME, timeout);
}

// 发送 RESET 命令，请求管理器将全部受管节点重置回 unconfigured 状态。
bool
LifecycleManagerClient::reset(const std::chrono::nanoseconds timeout)
{
  return callService(ManageLifecycleNodes::Request::RESET, timeout);
}

SystemStatus
// 调用 /is_active 服务并把 Trigger 响应映射为 ACTIVE、INACTIVE 或 TIMEOUT。
LifecycleManagerClient::is_active(const std::chrono::nanoseconds timeout)
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();

  RCLCPP_DEBUG(
    node_->get_logger(), "Waiting for the %s service...",
    active_service_name_.c_str());

  if (!is_active_client_->wait_for_service(std::chrono::seconds(1))) {
    return SystemStatus::TIMEOUT;
  }

  RCLCPP_DEBUG(
    node_->get_logger(), "Sending %s request",
    active_service_name_.c_str());

  try {
    response = is_active_client_->invoke(request, timeout);
  } catch (std::runtime_error &) {
    return SystemStatus::TIMEOUT;
  }

  if (response->success) {
    return SystemStatus::ACTIVE;
  } else {
    return SystemStatus::INACTIVE;
  }
}

bool
// 等待 /manage_nodes 服务可用，在给定 timeout 内发送生命周期命令并返回 success。
LifecycleManagerClient::callService(uint8_t command, const std::chrono::nanoseconds timeout)
{
  auto request = std::make_shared<ManageLifecycleNodes::Request>();
  request->command = command;

  RCLCPP_DEBUG(
    node_->get_logger(), "Waiting for the %s service...",
    manage_service_name_.c_str());

  while (!manager_client_->wait_for_service(timeout)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "Client interrupted while waiting for service to appear");
      return false;
    }
    RCLCPP_DEBUG(node_->get_logger(), "Waiting for service to appear...");
  }

  RCLCPP_DEBUG(
    node_->get_logger(), "Sending %s request",
    manage_service_name_.c_str());
  try {
    auto future_result = manager_client_->invoke(request, timeout);
    return future_result->success;
  } catch (std::runtime_error &) {
    return false;
  }
}

}  // namespace nav2_lifecycle_manager
