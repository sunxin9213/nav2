// Copyright (c) 2019 Samsung Research America
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
// waypoint_follower 独立可执行入口：
// 创建 WaypointFollower 生命周期节点并阻塞自旋。
// 该节点提供 follow_waypoints 动作：依次导航停靠每个航点，
// 并可在每个航点执行任务插件（如拍照、等待）。
// ============================================================================

#include <memory>

#include "nav2_waypoint_follower/waypoint_follower.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // 初始化 ROS2 -> 创建航点跟随节点 -> 阻塞自旋 -> 关闭退出
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_waypoint_follower::WaypointFollower>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
