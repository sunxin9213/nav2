// Copyright (c) 2020 Samsung Research Russia
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

#ifndef NAV2_MAP_SERVER__COSTMAP_FILTER_INFO_SERVER_HPP_
#define NAV2_MAP_SERVER__COSTMAP_FILTER_INFO_SERVER_HPP_

// ============================================================================
// 文件说明（中文）：
// CostmapFilterInfoServer 节点的头文件声明。
// 模块定位：代价地图过滤器（keepout 禁行区 / 限速区等）的元信息发布节点。
// 激活时以 transient_local QoS 发布一帧 nav2_msgs::msg::CostmapFilterInfo
// （过滤器类型、掩码话题名、base/multiplier 线性变换参数），
// 供 nav2_costmap_2d 的各 costmap filter 插件订阅使用。
// 实现（.cpp，含详细中文注释）见
// src/costmap_filter_info/costmap_filter_info_server.cpp。
// ============================================================================

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"

namespace nav2_map_server
{

class CostmapFilterInfoServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for the nav2_map_server::CostmapFilterInfoServer
   * @param options Additional options to control creation of the node.
   */
  explicit CostmapFilterInfoServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for the nav2_map_server::CostmapFilterInfoServer
   */
  ~CostmapFilterInfoServer();

protected:
  /**
   * @brief Creates CostmapFilterInfo publisher and forms published message from ROS parameters
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Publishes a CostmapFilterInfo message
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates publisher
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets publisher
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // CostmapFilterInfo 消息发布器（transient_local，latched 语义）
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr publisher_;

  // 待发布的过滤器信息消息（configure 阶段填充，activate 阶段发布）
  nav2_msgs::msg::CostmapFilterInfo msg_;
};  // CostmapFilterInfoServer

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__COSTMAP_FILTER_INFO_SERVER_HPP_
