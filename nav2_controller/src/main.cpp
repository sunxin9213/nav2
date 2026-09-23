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
 * 文件：main.cpp（nav2_controller 模块）
 * =============================================================================
 * 功能概述：
 *   控制服务器（ControllerServer）的可执行入口。
 *   以独立进程方式启动 ROS 2 组件：初始化运行时 → 创建 ControllerServer
 *   （生命周期节点，初始为 unconfigured 状态，等待 lifecycle_manager 通过
 *   /change_state 服务触发 configure/activate）→ 自旋处理回调 → 退出前清理。
 *
 * 在整个导航流程中的位置：
 *   bt_navigator → planner_server（全局路径）→ controller_server【本进程】
 *   → velocity_smoother（速度平滑后发布 /cmd_vel）。
 *   本进程接收 FollowPath 动作目标，沿全局路径闭环输出速度指令。
 * =============================================================================
 */

#include <memory>

#include "nav2_controller/controller_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // 初始化 ROS 2 上下文（参数、日志、信号处理）
  rclcpp::init(argc, argv);
  // 创建控制服务器节点（构造时声明参数并创建局部 Costmap，尚未配置/激活）
  auto node = std::make_shared<nav2_controller::ControllerServer>();
  // 阻塞自旋：处理 FollowPath 动作、odom 订阅、速度限制订阅、Bond 等回调
  rclcpp::spin(node->get_node_base_interface());
  // 退出前关闭 ROS 2 上下文
  rclcpp::shutdown();

  return 0;
}
