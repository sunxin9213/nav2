// Copyright (c) 2020 Fetullah Atas
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
// 【文件概况】航点到点任务插件基类 WaypointTaskExecutor
// ----------------------------------------------------------------------------
// 概述: 定义"机器人到达某个航点后执行附加任务"的插件接口。
//       航点间的导航调度由 WaypointFollower 负责，插件只负责到点后
//       的动作（拍照、等待、开门、抓取等），两者职责完全解耦。
// 主要内容:
//   1. WaypointTaskExecutor: 抽象基类
//      - initialize:         创建插件所需的 pub/sub/service client
//      - processAtWaypoint:  核心接口，机器人到达航点后被同步调用
// 使用场景: nav2_waypoint_follower 的 waypoint_follower 加载实现
//           （如 nav2_bt_waypoint_follower 的 BT 任务执行器、
//            wait_at_waypoint、photo_at_waypoint 等示例插件），
//           在每个航点 navigate_to_pose 成功后调用 processAtWaypoint。
// ============================================================================


#ifndef NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
#define NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_core
{
/**
 * @brief Base class for creating a plugin in order to perform a specific task at waypoint arrivals.
 *
 */
// 航点到点任务插件基类: 定义机器人到达航点后执行的附加动作接口
class WaypointTaskExecutor
{
public:
  // 这是 waypoint 到点任务插件的最小接口：WaypointFollower 负责导航调度，
  // 插件只负责机器人到达某个点之后的附加动作，因此两者职责彼此独立。
  /**
   * @brief Construct a new Simple Task Execution At Waypoint Base object
   *
   */
  // 默认构造（基类无状态，无需特殊初始化）
  WaypointTaskExecutor() {}

  /**
   * @brief Destroy the Simple Task Execution At Waypoint Base object
   *
   */
  // 虚析构保证通过基类指针释放派生插件时正确调用派生类析构
  virtual ~WaypointTaskExecutor() {}

  // 初始化到达 waypoint 后任务所需的 publisher、subscriber 或 service client。
  /**
   * @brief Override this to setup your pub, sub or any ros services that you will use in the plugin.
   *
   * @param parent parent node that plugin will be created within(for an example see nav_waypoint_follower)
   * @param plugin_name plugin name comes from parameters in yaml file
   */
  // 参数含义（中文补充）:
  //   parent:      承载插件的生命周期父节点（WeakPtr）
  //   plugin_name: 插件实例名（参数前缀，来自 YAML 配置）
  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) = 0;

  // WaypointFollower 在 navigate_to_pose 成功后同步调用该函数。
  // 返回值会参与 stop_on_failure 判断：false 可以让整个 waypoint Action 失败，
  // 也可以在允许继续时被记录为 missed waypoint。
  /**
   * @brief Override this to define the body of your task that you would like to execute once the robot arrived to waypoint
   *
   * @param curr_pose current pose of the robot
   * @param curr_waypoint_index current waypoint, that robot just arrived
   * @return true if task execution was successful
   * @return false if task execution failed
   */
  // 核心接口: 机器人刚到达某航点时被调用，执行该航点的附加任务
  // 参数含义（中文补充）:
  //   curr_pose:          机器人当前（到点）位姿
  //   curr_waypoint_index: 刚到达的航点在任务中的序号（从 0 起）
  // 返回: true 任务成功；false 任务失败
  virtual bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index) = 0;
};
}  // namespace nav2_core
#endif  // NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
