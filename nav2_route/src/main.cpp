// Copyright (c) 2025 Open Navigation LLC
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
// 文件说明（中文）：
// route_server 独立可执行入口：
// 创建 RouteServer（路线服务器）生命周期节点并阻塞自旋。
// 提供基于路图的 compute_route / compute_and_track_route 动作，
// 适用于结构化环境（车道/走廊）的拓扑导航。
// ============================================================================

#include <memory>

#include "nav2_route/route_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // 初始化 ROS2 -> 创建路线服务器节点 -> 阻塞自旋 -> 关闭退出
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_route::RouteServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
