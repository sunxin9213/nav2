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
class Behavior
{
public:
  using Ptr = std::shared_ptr<Behavior>;

  /**
   * @brief Virtual destructor
   */
  virtual ~Behavior() {}

  // 生命周期配置阶段注入父节点、TF Buffer 和碰撞检查器；插件应在此读取参数并创建通信实体。
  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   * @param  costmap_ros A pointer to the costmap
   */
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
