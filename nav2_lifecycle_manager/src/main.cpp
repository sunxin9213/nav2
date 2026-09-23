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
 * 文件：main.cpp（nav2_lifecycle_manager 模块）
 * =============================================================================
 * 功能概述：
 *   生命周期管理器（LifecycleManager）的可执行入口。
 *   与各核心服务器不同，LifecycleManager 是普通 rclcpp::Node（非生命周期节点），
 *   进程启动后即开始工作：构造时创建 manage_nodes / is_active 服务并注册
 *   延迟初始化定时器；若 autostart=true 则自动驱动全部受管节点进入 active。
 *
 * 在整个导航流程中的位置：
 *   位于导航数据链路（bt_navigator → planner_server → controller_server
 *   → velocity_smoother）之外的管理平面，负责整条链的启动、关闭与守护。
 * =============================================================================
 */

#include <memory>

#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // 初始化 ROS 2，创建生命周期管理器并进入 executor 处理服务、定时器和 bond 回调。
  // 初始化 ROS 2 运行时（参数解析、日志、信号处理）
  rclcpp::init(argc, argv);
  // 创建生命周期管理器节点：声明参数、创建管理服务与延迟初始化定时器；
  // 若 autostart=true，初始化定时器会自动执行 startup() 启动全部受管节点
  auto node = std::make_shared<nav2_lifecycle_manager::LifecycleManager>();
  // 阻塞自旋：处理 manage_nodes / is_active 服务、bond 检查定时器等回调
  rclcpp::spin(node);
  // 进程退出前清理 ROS 2 上下文（此前 preshutdown 回调已先行清理 bond）
  rclcpp::shutdown();

  return 0;
}
