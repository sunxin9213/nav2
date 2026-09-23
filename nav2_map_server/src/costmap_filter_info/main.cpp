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

// ============================================================================
// 文件说明（中文）：
// costmap_filter_info_server 独立可执行入口：
// 创建 CostmapFilterInfoServer 生命周期节点并阻塞自旋，
// 由生命周期管理器调度，向代价地图过滤器发布元信息。
// ============================================================================

#include <memory>

#include "nav2_map_server/costmap_filter_info_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  auto logger = rclcpp::get_logger("costmap_filter_info_server");

  RCLCPP_INFO(logger, "This is costmap filter info publisher");

  // 初始化 ROS2 -> 创建节点 -> 自旋处理回调 -> 关闭退出
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_map_server::CostmapFilterInfoServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
