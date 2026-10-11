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

// ============================================================================
// 【文件概况】进度检查插件接口 ProgressChecker
// ----------------------------------------------------------------------------
// 概述: 定义"机器人在朝目标行进中是否真的有进展"的判定插件接口。
//       用于检测卡死/原地打转等异常——若长时间无有效位移，
//       controller server 会触发恢复行为。
// 主要内容:
//   1. ProgressChecker: 抽象基类
//      - initialize: 读取进展半径、允许时间等参数
//      - check:      核心接口，比较当前位姿与基准位姿判断是否有进展
//      - reset:      重置基准位姿等内部状态
// 使用场景: nav2_controller 的 controller_server 加载实现（默认
//           pose_progress_checker / distance_cartesian_progress_checker），
//           在 FollowPath 控制循环中每周期调用 check；
//           check 返回 false 视为"卡住"，抛出异常进入恢复行为。
// ============================================================================

#ifndef NAV2_CORE__PROGRESS_CHECKER_HPP_
#define NAV2_CORE__PROGRESS_CHECKER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace nav2_core
{
/**
 * @class nav2_core::ProgressChecker
 * @brief This class defines the plugin interface used to check the
 * position of the robot to make sure that it is actually progressing
 * towards a goal.
 */
// 进度检查器插件基类: 检测机器人相对上次基准位置是否取得有效进展
class ProgressChecker
{
public:
  // 插件实例的共享指针类型别名，供 controller_server 统一管理
  typedef std::shared_ptr<nav2_core::ProgressChecker> Ptr;

  // 虚析构保证通过基类指针释放派生插件时正确调用派生类析构
  virtual ~ProgressChecker() {}

  // 初始化进展半径、允许时间等参数；这些参数通常由 Controller Server 的插件配置块提供。
  /**
   * @brief Initialize parameters for ProgressChecker
   * @param parent Node pointer
   */
  // 参数含义（中文补充）:
  //   parent:      承载插件的生命周期父节点（WeakPtr）
  //   plugin_name: 插件实例名（参数前缀）
  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) = 0;
  /**
   * @brief Checks if the robot has moved compare to previous
   * pose
   * @param current_pose Current pose of the robot
   * @return True if progress is made
   */
  // 核心接口: 判断机器人相比之前的基准位姿是否有进展
  // 参数: current_pose 机器人当前位姿
  // 返回: true 表示有进展；false 表示停滞（上层据此触发恢复行为）
  virtual bool check(geometry_msgs::msg::PoseStamped & current_pose) = 0;
  /**
   * @brief Reset class state upon calling
   */
  // 重置内部状态: 新任务/恢复行为执行后重新建立基准位姿
  virtual void reset() = 0;
};
}  // namespace nav2_core

#endif  // NAV2_CORE__PROGRESS_CHECKER_HPP_
