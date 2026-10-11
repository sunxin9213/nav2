// Copyright (c) 2021 RoboTech Vision
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
// 【文件概况】路径平滑器插件抽象接口 Smoother
// ----------------------------------------------------------------------------
// 概述: 定义路径平滑器插件（simple_smoother、constrained_smoother 等）
//       与 Smoother Server 之间的纯虚接口，经 pluginlib 动态加载。
//       平滑器职责是优化全局路径的几何形状（去毛刺、平滑转弯），
//       同时保持与障碍物的安全距离。
// 主要内容:
//   1. Smoother: 抽象基类
//      - configure/cleanup/activate/deactivate: 生命周期四件套
//        （configure 注入 CostmapSubscriber / FootprintSubscriber
//         —— 平滑器以话题订阅方式获取代价地图与机器人外形，而非直接指针）
//      - smooth: 核心接口，原地平滑给定路径，受最大时间限制
// 使用场景: nav2_smoother 的 smoother_server 在 SmoothPath Action 中
//           调用；行为树的 SmoothPath 节点触发（可配置规划后/控制前执行）。
// ============================================================================

#ifndef NAV2_CORE__SMOOTHER_HPP_
#define NAV2_CORE__SMOOTHER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav_msgs/msg/path.hpp"


namespace nav2_core
{

/**
 * @class Smoother
 * @brief smoother interface that acts as a virtual base class for all smoother plugins
 */
// 平滑器插件基类: 所有优化路径几何形状的插件必须实现此接口
class Smoother
{
public:
  // 插件实例的共享指针类型别名，供 smoother_server 统一管理
  using Ptr = std::shared_ptr<nav2_core::Smoother>;

  /**
   * @brief Virtual destructor
   */
  // 虚析构保证通过基类指针释放派生插件时正确调用派生类析构
  virtual ~Smoother() {}

  // 注入生命周期节点、TF 和 Costmap 订阅，准备路径平滑所需的运行时依赖。
  // 参数含义（中文补充）:
  //   parent: 承载插件的生命周期父节点（WeakPtr）
  //   name:   插件实例名（参数前缀）
  //   tf:     TF 缓冲区指针
  //   costmap_subscriber:   代价地图话题订阅（获取最新地图数据）
  //   footprint_subscriber: 机器人外形话题订阅（碰撞校验用）
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) = 0;

  /**
   * @brief Method to cleanup resources.
   */
  // 释放插件持有的订阅与内部缓冲资源
  virtual void cleanup() = 0;

  /**
   * @brief Method to activate smoother and any threads involved in execution.
   */
  // 激活平滑器: 启用其内部的话题订阅/发布
  virtual void activate() = 0;

  /**
   * @brief Method to deactivate smoother and any threads involved in execution.
   */
  // 停用平滑器: 暂停处理新的平滑请求
  virtual void deactivate() = 0;

  /**
   * @brief Method to smooth given path
   *
   * @param path In-out path to be smoothed
   * @param max_time Maximum duration smoothing should take
   * @return If smoothing was completed (true) or interrupted by time limit (false)
   */
  // 核心接口: 原地平滑给定路径
  // 参数含义（中文补充）:
  //   path:     输入输出参数 —— 原始路径进、平滑后路径出
  //   max_time: 允许的最大平滑时长（软实时约束，超时应中断返回）
  // 返回: true 平滑完整完成；false 因超时被中断（部分平滑的结果仍保留在 path 中）
  virtual bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__SMOOTHER_HPP_
