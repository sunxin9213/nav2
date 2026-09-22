// Copyright (c) 2018 Samsung Research America
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
// limitations under the License. Reserved.


#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/behavior.hpp"

#ifndef NAV2_BEHAVIORS__BEHAVIOR_SERVER_HPP_
#define NAV2_BEHAVIORS__BEHAVIOR_SERVER_HPP_

namespace behavior_server
{

/**
 * @class behavior_server::BehaviorServer
 * @brief An server hosting a map of behavior plugins
 */
class BehaviorServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for behavior_server::BehaviorServer
   * @param options Additional options to control creation of the node.
   */
  // 构造生命周期节点；NodeOptions 可传入命名空间、参数覆盖和组件组合配置。
  explicit BehaviorServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  // 释放插件容器；插件生命周期资源由 on_cleanup 在正常生命周期转换中处理。
  ~BehaviorServer();

  /**
   * @brief Loads behavior plugins from parameter file
   * @return bool if successfully loaded the plugins
   */
  // 读取 behavior_plugins 列表和每个插件的类型字符串，动态创建行为实例。
  bool loadBehaviorPlugins();

protected:
  /**
   * @brief Configure lifecycle server
   */
  // 配置阶段创建 TF/Costmap 依赖并加载插件；返回 FAILURE 会阻止进入 active 状态。
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate lifecycle server
   */
  // 激活插件的 Action Server 和发布器，允许行为真正控制机器人。
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate lifecycle server
   */
  // 停用插件通信实体，防止 inactive 节点继续处理行为请求或发布控制命令。
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup lifecycle server
   */
  // 清理动态插件、TF、Costmap 订阅和碰撞检查器，为下一次 configure 做准备。
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Shutdown lifecycle server
   */
  // 处理进程关闭事件；此阶段不再接受新的行为请求。
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  // Plugins
  pluginlib::ClassLoader<nav2_core::Behavior> plugin_loader_;
  std::vector<pluginlib::UniquePtr<nav2_core::Behavior>> behaviors_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> behavior_ids_;
  std::vector<std::string> behavior_types_;

  // Utilities
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
};

}  // namespace behavior_server

#endif  // NAV2_BEHAVIORS__BEHAVIOR_SERVER_HPP_
