/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

// ============================================================================
// 【文件概况】目标到达判定插件接口 GoalChecker
// ----------------------------------------------------------------------------
// 概述: 定义"机器人是否到达目标"的判定插件接口。主要比较两个位姿的
//       相对位置（假定同一坐标系），可选校验速度（部分应用要求机器人
//       完全停止才算到达）。
// 主要内容:
//   1. GoalChecker: 抽象基类
//      - initialize: 读取容差参数（xy_dist_tol、yaw_tol 等）
//      - reset:      新目标/新任务时重置内部状态
//      - isGoalReached: 核心接口，判定是否到达
//      - getTolerances: 向外提供当前使用的容差（供 controller 收敛速度）
// 使用场景: nav2_controller 的 controller_server 加载实现（默认
//           simple_goal_checker / pose_goal_checker / exact_goal_checker），
//           在 FollowPath 控制循环中每周期调用 isGoalReached，
//           返回 true 时任务成功结束。
// ============================================================================

#ifndef NAV2_CORE__GOAL_CHECKER_HPP_
#define NAV2_CORE__GOAL_CHECKER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"


namespace nav2_core
{

/**
 * @class GoalChecker
 * @brief Function-object for checking whether a goal has been reached
 *
 * This class defines the plugin interface for determining whether you have reached
 * the goal state. This primarily consists of checking the relative positions of two poses
 * (which are presumed to be in the same frame). It can also check the velocity, as some
 * applications require that robot be stopped to be considered as having reached the goal.
 */
// 目标检查器插件基类: 判定机器人是否到达目标（位置/姿态容差 + 可选速度校验）
class GoalChecker
{
public:
  // 插件实例的共享指针类型别名，供 controller_server 统一管理
  typedef std::shared_ptr<nav2_core::GoalChecker> Ptr;

  // 虚析构保证通过基类指针释放派生插件时正确调用派生类析构
  virtual ~GoalChecker() {}

  // 在 Controller Server 配置阶段读取容差参数；plugin_name 用于区分同类插件的参数前缀。
  /**
   * @brief Initialize any parameters from the NodeHandle
   * @param parent Node pointer for grabbing parameters
   */
  // 参数含义（中文补充）:
  //   parent:      承载插件的生命周期父节点（WeakPtr）
  //   plugin_name: 插件实例名（参数前缀）
  //   costmap_ros: 代价地图指针（部分实现可能用其获取机器人外形等）
  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  // 重置内部状态: 新任务开始时调用，清除已记录的历史状态
  // （如 stateful 检查器记录的速度均值等）
  virtual void reset() = 0;

  /**
   * @brief Check whether the goal should be considered reached
   * @param query_pose The pose to check
   * @param goal_pose The pose to check against
   * @param velocity The robot's current velocity
   * @return True if goal is reached
   */
  // 核心接口: 判定 query_pose 是否满足到达 goal_pose 的条件
  // 参数含义（中文补充）:
  //   query_pose: 机器人当前位姿（与 goal_pose 同一坐标系）
  //   goal_pose:  目标位姿
  //   velocity:   机器人当前速度（启用速度检查的实现用其判断是否已停止）
  // 返回: true 表示到达（任务成功）
  virtual bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) = 0;

  /**
   * @brief Get the maximum possible tolerances used for goal checking in the major types.
   * Any field without a valid entry is replaced with std::numeric_limits<double>::lowest()
   * to indicate that it is not measured. For tolerance across multiple entries
   * (e.x. XY tolerances), both fields will contain this value since it is the maximum tolerance
   * that each independent field could be assuming the other has no error (e.x. X and Y).
   * @param pose_tolerance The tolerance used for checking in Pose fields
   * @param vel_tolerance The tolerance used for checking velocity fields
   * @return True if the tolerances are valid to use
   */
  // 输出本检查器使用的最大容差（位姿容差与速度容差）
  // 未参与判定的字段填 std::numeric_limits<double>::lowest() 表示"不检测"；
  // XY 这类联合容差会把该值同时填入两个字段（假设另一维零误差时的最大值）
  // 用途: controller 插件据此在接近目标时收敛速度（如 RPP 的降速逻辑）
  // 返回: 容差是否有效可用
  virtual bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__GOAL_CHECKER_HPP_
