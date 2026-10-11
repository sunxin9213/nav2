/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  Copyright (c) 2019, Intel Corporation
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
// 【文件概况】控制器插件抽象接口 Controller
// ----------------------------------------------------------------------------
// 概述: 定义局部控制器插件（DWB、Regulated Pure Pursuit、MPPI、
//       Graceful、Rotation Shim 等）与 Controller Server 之间的纯虚
//       接口，经 pluginlib 动态加载。控制器职责是沿全局路径输出
//       速度命令（TwistStamped），实现局部避障与轨迹跟踪。
// 主要内容:
//   1. Controller: 抽象基类
//      - configure/cleanup/activate/deactivate: 生命周期四件套
//      - setPlan: 注入待跟踪的全局路径
//      - computeVelocityCommands: 核心接口，按当前位姿/速度计算速度命令
//      - setSpeedLimit: 接收外部（如速度限制 costmap filter）限速指令
// 使用场景: nav2_controller 的 controller_server 在 FollowPath Action
//           中周期调用（默认 20Hz）：setPlan 一次、computeVelocityCommands
//           循环调用直到 goal checker 判定到达。
// ============================================================================

#ifndef NAV2_CORE__CONTROLLER_HPP_
#define NAV2_CORE__CONTROLLER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/goal_checker.hpp"


namespace nav2_core
{

/**
 * @class Controller
 * @brief controller interface that acts as a virtual base class for all controller plugins
 */
// 局部控制器插件基类：所有跟踪全局路径并输出速度命令的插件必须实现此接口
class Controller
{
public:
  // 插件实例的共享指针类型别名，供 controller_server 统一管理
  using Ptr = std::shared_ptr<nav2_core::Controller>;


  /**
   * @brief Virtual destructor
   */
  // 虚析构保证通过基类指针释放派生插件时正确调用派生类析构
  virtual ~Controller() {}

  // 配置控制器插件的父节点、TF 和 Costmap；插件通常在这里声明参数并创建 publisher。
  /**
   * @param  parent pointer to user's node
   * @param  costmap_ros A pointer to the costmap
   */
  // 参数含义（中文补充）:
  //   parent:      承载插件的生命周期父节点（WeakPtr）
  //   name:        插件实例名（参数前缀）
  //   tf:          TF 缓冲区指针
  //   costmap_ros: 局部代价地图（Costmap2DROS）指针，用于避障查询
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) = 0;

  // 释放控制器持有的参数、线程和通信资源。
  /**
   * @brief Method to cleanup resources.
   */
  virtual void cleanup() = 0;

  // 允许控制器开始响应路径并输出速度命令。
  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  virtual void activate() = 0;

  // 停止控制器输出，避免非 active 状态继续驱动底盘。
  /**
   * @brief Method to deactive planner and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief local setPlan - Sets the global plan
   * @param path The global plan
   */
  // 设置待跟踪的全局路径；通常一次任务中调用一次（或重规划后再次调用），
  // 插件应缓存路径并完成坐标系转换/裁剪等预处理
  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;

  /**
   * @brief Controller computeVelocityCommands - calculates the best command given the current pose and velocity
   *
   * It is presumed that the global plan is already set.
   *
   * This is mostly a wrapper for the protected computeVelocityCommands
   * function which has additional debugging info.
   *
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @param goal_checker Pointer to the current goal checker the task is utilizing
   * @return The best command for the robot to drive
   */
  // 核心接口：按当前位姿与速度计算速度命令（TwistStamped）
  // 前提: setPlan 已被调用（全局路径已就绪）
  // 参数含义（中文补充）:
  //   pose:         机器人当前位姿（全局坐标系）
  //   velocity:     机器人当前速度（用于加速度限制等）
  //   goal_checker: 目标检查器指针，插件可在接近目标时降低速度/提前停止
  // 返回: 期望的速度命令，经 velocity_smoother 平滑后发给底盘
  // 异常: 插件可抛 ControllerException/PlannerException 等使任务失败并触发恢复
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) = 0;

  // 设置线速度上限；percentage=true 时按机器人最大速度百分比解释，否则按 m/s 解释。
  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  // 外部限速入口: speed filter（costmap filter）经 controller_server
  // 转发限速请求时调用，插件应即时调整内部速度上限
  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__CONTROLLER_HPP_
