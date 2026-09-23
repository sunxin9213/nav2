// Copyright (c) 2018 Intel Corporation
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

/*
 * =============================================================================
 * 文件：main.cpp（nav2_planner 模块）
 * =============================================================================
 * 功能概述：
 *   规划服务器（PlannerServer）的可执行入口。
 *   Nav2 中每个核心服务器都以 ROS 2 组件（component）形式实现，
 *   main.cpp 仅负责以"独立进程"方式将该组件跑起来：
 *     1) rclcpp::init            —— 初始化 ROS 2 运行时（参数、日志、信号）；
 *     2) 创建 PlannerServer 节点  —— 生命周期节点，默认处于 unconfigured 状态，
 *                                     由 lifecycle_manager 通过服务触发配置/激活；
 *     3) rclcpp::spin            —— 阻塞在主线程，处理回调（Action、服务、
 *                                   TF、Bond 心跳、参数事件等）直到 Ctrl+C；
 *     4) rclcpp::shutdown        —— 退出前清理运行时。
 *
 * 在整个导航流程中的位置：
 *   bt_navigator → planner_server（本进程）→ controller_server → velocity_smoother。
 *   注意：本进程并不主动 configure/activate 自身，节点状态的推进
 *   完全由 lifecycle_manager 通过 /change_state 生命周期服务驱动。
 * =============================================================================
 */

#include <memory>

#include "nav2_planner/planner_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // 初始化 ROS 2 上下文：解析命令行参数、注册信号处理器
  rclcpp::init(argc, argv);
  // 创建规划服务器节点（构造时声明参数并创建全局 Costmap，但尚未配置/激活）
  auto node = std::make_shared<nav2_planner::PlannerServer>();
  // 阻塞自旋：处理该节点的所有回调（Action 目标、服务、参数、Bond 等）
  rclcpp::spin(node->get_node_base_interface());
  // 关闭 ROS 2 上下文并安全退出进程
  rclcpp::shutdown();

  return 0;
}
