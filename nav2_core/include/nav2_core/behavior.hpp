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
// 【文件概况】行为（恢复行为）插件抽象接口 Behavior
// ----------------------------------------------------------------------------
// 概述: 定义 nav2 行为插件（如 Spin、BackUp、DriveOnHeading、Wait 等
//       恢复/脱困动作）与 Behavior Server 之间的纯虚生命周期接口，
//       经 pluginlib 动态加载。
// 主要内容:
//   1. Behavior: 抽象基类，声明 configure/cleanup/activate/deactivate
//      四个生命周期纯虚函数；configure 注入父节点、插件名、TF Buffer
//      与基于话题的碰撞检查器（CostmapTopicCollisionChecker）。
// 使用场景: nav2_behaviors 的 behavior_server 加载实现该接口的插件；
//           行为树中的恢复节点（RecoveryNode 分支）通过对应 Action
//           触发这些行为。机器人卡死、需要倒车/原地旋转脱困时使用。
// ============================================================================

#ifndef NAV2_CORE__BEHAVIOR_HPP_
#define NAV2_CORE__BEHAVIOR_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"

namespace nav2_core
{

/**
 * @class Behavior
 * @brief Abstract interface for behaviors to adhere to with pluginlib
 */
// 行为插件基类：所有恢复行为插件（spin/backup/wait 等）必须实现此接口
class Behavior
{
public:
  // 插件实例的共享指针类型别名，供 server 统一管理插件生命周期
  using Ptr = std::shared_ptr<Behavior>;

  /**
   * @brief Virtual destructor
   */
  // 虚析构保证通过基类指针释放派生插件时正确调用派生类析构
  virtual ~Behavior() {}

  // 生命周期配置阶段注入父节点、TF Buffer 和碰撞检查器；插件应在此读取参数并创建通信实体。
  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   * @param  costmap_ros A pointer to the costmap
   */
  // 参数含义（中文补充）:
  //   parent: 承载插件的父生命周期节点（WeakPtr，防止延长节点寿命）
  //   name:   插件实例名（对应参数 YAML 中的插件名，用于参数前缀）
  //   tf:     TF 缓冲区指针，用于坐标变换查询
  //   collision_checker: 基于话题的碰撞检查器，行为执行中用于碰撞校验
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker) = 0;

  // 清理配置阶段创建的资源，使同一插件实例可以安全地重新配置。
  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  // 生命周期进入 active 时启用 Action、发布器或内部线程。
  /**
   * @brief Method to active Behavior and any threads involved in execution.
   */
  virtual void activate() = 0;

  // 生命周期离开 active 时停止处理请求和发布控制命令。
  /**
   * @brief Method to deactive Behavior and any threads involved in execution.
   */
  virtual void deactivate() = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__BEHAVIOR_HPP_
