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
// 【文件概况】全局规划器插件抽象接口 GlobalPlanner
// ----------------------------------------------------------------------------
// 概述: 定义全局规划器插件（NavFn、Smac Planner、Theta* 等）与
//       Planner Server 之间的纯虚接口，经 pluginlib 动态加载。
//       全局规划器职责是在全局代价地图上计算起点到终点的一条完整路径。
// 主要内容:
//   1. GlobalPlanner: 抽象基类
//      - configure/cleanup/activate/deactivate: 生命周期四件套
//      - createPlan: 核心接口，给定起终点计算全局路径
// 使用场景: nav2_planner 的 planner_server 在 ComputePathToPose /
//           ComputePathThroughPoses Action 中调用插件的 createPlan；
//           行为树的 ComputePathToPose 节点经该 server 触发规划。
// ============================================================================

#ifndef NAV2_CORE__GLOBAL_PLANNER_HPP_
#define NAV2_CORE__GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_core
{

/**
 * @class GlobalPlanner
 * @brief Abstract interface for global planners to adhere to with pluginlib
 */
// 全局规划器插件基类：所有在全局代价地图上计算路径的插件必须实现此接口
class GlobalPlanner
{
public:
  // 插件实例的共享指针类型别名，供 planner_server 统一管理
  using Ptr = std::shared_ptr<GlobalPlanner>;

  /**
   * @brief Virtual destructor
   */
  // 虚析构保证通过基类指针释放派生插件时正确调用派生类析构
  virtual ~GlobalPlanner() {}

  // 注入生命周期节点、TF 和 Costmap，准备规划器参数与内部地图访问接口。
  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   * @param  costmap_ros A pointer to the costmap
   */
  // 参数含义（中文补充）:
  //   parent:      承载插件的生命周期父节点（WeakPtr）
  //   name:        插件实例名（参数前缀）
  //   tf:          TF 缓冲区指针
  //   costmap_ros: 全局代价地图（Costmap2DROS）指针
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  // 清理规划器内部缓存和通信资源。
  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  // 激活规划器，使其可以响应规划请求。
  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  virtual void activate() = 0;

  // 停用规划器，停止处理新的规划工作。
  /**
   * @brief Method to deactive planner and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief Method create the plan from a starting and ending goal.
   * @param start The starting pose of the robot
   * @param goal  The goal pose of the robot
   * @return      The sequence of poses to get from start to goal, if any
   */
  // 核心接口: 计算从 start 到 goal 的全局路径（位姿序列）
  // 参数含义（中文补充）:
  //   start: 起点位姿（通常为机器人当前位姿）
  //   goal:  目标位姿（同一坐标系下）
  // 返回: 全局路径；无解时应抛出 PlannerException 而非返回空路径
  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__GLOBAL_PLANNER_HPP_
