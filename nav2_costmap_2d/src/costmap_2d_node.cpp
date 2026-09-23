/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
// ============================================================================
// 中文注释：独立代价地图节点（costmap_2d_node）可执行入口
//
// 【文件功能】
// 本文件是 nav2_costmap_2d 包提供的独立可执行程序 main() 入口，
// 用于单独运行一张代价地图（不依赖 planner/controller 服务器）。
// 常用于调试：单独启动 costmap 节点，配合 RViz 查看地图效果。
//
// 【在 Nav2 中的位置】
// Costmap2DROS 是一个 rclcpp_lifecycle 生命周期节点；Nav2 通常由
// nav2_lifecycle_manager 统一管理各节点状态迁移。本入口直接 spin 该
// 生命周期节点——注意生命周期节点需要被外部触发 configure/activate
// （例如通过 lifecycle 管理器或 rviz 的生命周期控制界面）才会工作。
//
// 【ROS2 知识点】
// - rclcpp::init：初始化 ROS2 运行时（解析参数、连接等）；
// - rclcpp::spin：进入事件循环，处理回调直到节点关闭（Ctrl-C）；
// - spin(node->get_node_base_interface())：以节点基接口启动，
//   兼容生命周期节点的标准写法。
// ============================================================================
#include <memory>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);   // 初始化 ROS2 客户端库
  // 创建名为 "costmap" 的代价地图 ROS 节点（生命周期节点）
  auto node = std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap");
  // 阻塞式事件循环：处理订阅/服务/定时器回调，直到关闭
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();         // 清理 ROS2 运行时

  return 0;
}
