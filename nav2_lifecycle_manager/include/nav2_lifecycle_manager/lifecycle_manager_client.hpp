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
 * 文件：lifecycle_manager_client.hpp（nav2_lifecycle_manager 模块）
 * =============================================================================
 * 功能概述：
 *   声明 LifecycleManagerClient 类与 SystemStatus 枚举——
 *   生命周期管理器的服务客户端辅助类。
 *   供行为树插件、RViz 插件、测试等外部代码远程控制 lifecycle_manager：
 *     startup()/shutdown()/pause()/resume()/reset()
 *       → 调用 <name>/manage_nodes 服务；
 *     is_active() → 调用 <name>/is_active 服务，返回
 *       SystemStatus::ACTIVE / INACTIVE / TIMEOUT。
 *   内部所有服务调用由 nav2_util::ServiceClient 封装，
 *   基于调用方传入的父节点（parent_node）执行，超时时间可配置（默认无限等待）。
 *   对应实现见 lifecycle_manager_client.cpp。
 * =============================================================================
 */

#ifndef NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_
#define NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav2_util/service_client.hpp"

namespace nav2_lifecycle_manager
{
/**
 * @enum nav2_lifecycle_manager::SystemStatus
 * @brief Enum class representing the status of the system.
 */
// 系统状态枚举：ACTIVE（受管系统整体激活）/ INACTIVE（未激活）/ TIMEOUT（服务调用超时）
enum class SystemStatus {ACTIVE, INACTIVE, TIMEOUT};
/**
 * @class nav2_lifecycle_manager::LifeCycleMangerClient
 * @brief The LifecycleManagerClient sends requests to the LifecycleManager to
 * control the lifecycle state of the navigation modules.
 */
class LifecycleManagerClient
{
public:
  /**
   * @brief A constructor for LifeCycleMangerClient
   * @param name Managed node name
   * @param parent_node Node that execute the service calls
   */
  explicit LifecycleManagerClient(
    const std::string & name,
    std::shared_ptr<rclcpp::Node> parent_node);

  // Client-side interface to the Nav2 lifecycle manager
  /**
   * @brief Make start up service call
   * @return true or false
   */
  bool startup(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  /**
   * @brief Make shutdown service call
   * @return true or false
   */
  bool shutdown(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  /**
   * @brief Make pause service call
   * @return true or false
   */
  bool pause(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  /**
   * @brief Make resume service call
   * @return true or false
   */
  bool resume(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  /**
   * @brief Make reset service call
   * @return true or false
   */
  bool reset(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  /**
   * @brief Check if lifecycle node manager server is active
   * @return ACTIVE or INACTIVE or TIMEOUT
   */
  SystemStatus is_active(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

protected:
  using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;

  /**
   * @brief A generic method used to call startup, shutdown, etc.
   * @param command
   */
  bool callService(
    uint8_t command,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  // ---- 成员变量 ----
  // node_：执行服务调用所借助的父节点（复用其 context 与 executor）；
  // manager_client_：manage_nodes 服务客户端（生命周期命令）；
  // is_active_client_：is_active 服务客户端（状态查询）；
  // manage_service_name_ / active_service_name_：两个服务的全限定名称。
  // The node to use for the service call
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<nav2_util::ServiceClient<ManageLifecycleNodes>> manager_client_;
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::Trigger>> is_active_client_;
  std::string manage_service_name_;
  std::string active_service_name_;
};

}  // namespace nav2_lifecycle_manager

#endif  // NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_
