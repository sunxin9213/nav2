// Copyright (c) 2022 Samsung R&D Institute Russia
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
// collision_monitor 独立可执行入口：
// 创建 CollisionMonitor 节点并阻塞自旋。
// 该节点订阅传感器数据与速度指令，按安全区检测结果输出
// 减速/停车的速度命令，为导航栈提供独立于规划器的安全防护层。
// ============================================================================

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "nav2_collision_monitor/collision_monitor_node.hpp"

int main(int argc, char * argv[])
{
  // 初始化 ROS2 -> 创建碰撞监视节点 -> 阻塞自旋 -> 关闭退出
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_collision_monitor::CollisionMonitor>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}

