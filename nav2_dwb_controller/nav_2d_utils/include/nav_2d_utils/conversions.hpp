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
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NAV_2D_UTILS__CONVERSIONS_HPP_
#define NAV_2D_UTILS__CONVERSIONS_HPP_

#include <vector>
#include <string>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/convert.h"

namespace nav_2d_utils
{
// ----------------------------------------------------------------------------
// 本头文件集中提供 Nav2 内部 2D 消息类型与标准 ROS 3D 消息类型之间的
// 转换函数族：
//   Twist2D <-> geometry_msgs::msg::Twist        速度（2D 平面 vs 3D 空间）
//   PoseStamped <-> Pose2DStamped                带时间戳的位姿
//   Pose <-> Pose2D                               位姿（四元数 vs 单一偏航角）
//   vector<PoseStamped> <-> nav_msgs::msg::Path  路径
//   Path <-> Path2D                               2D 路径 vs 标准路径
// Nav2 的 DWB 控制器族在内部使用轻量的 nav_2d_msgs 类型（只有 x/y/theta
// 三个分量），而与外部（TF、可视化、话题）交互必须用标准 3D 消息，
// 这些转换函数消除两者之间的样板代码。
// ----------------------------------------------------------------------------

// 2D 速度转 3D 速度：x/y 线速度与 theta 角速度分别映射到 linear.x/y 与 angular.z
geometry_msgs::msg::Twist twist2Dto3D(const nav_2d_msgs::msg::Twist2D & cmd_vel_2d);
// 3D 速度转 2D 速度：反向映射（丢弃 z 线速度与 x/y 角速度分量）
nav_2d_msgs::msg::Twist2D twist3Dto2D(const geometry_msgs::msg::Twist & cmd_vel);
// nav_2d_msgs::msg::Pose2DStamped stampedPoseToPose2D(const tf2::Stamped<tf2::Pose>& pose);
// 带戳位姿转 2D：位姿四元数通过 tf2::getYaw 提取为单一偏航角
nav_2d_msgs::msg::Pose2DStamped poseStampedToPose2D(const geometry_msgs::msg::PoseStamped & pose);
// 3D 位姿转 2D 位姿：位置取 x/y，姿态取 yaw（四元数 -> 单角度）
geometry_msgs::msg::Pose2D poseToPose2D(const geometry_msgs::msg::Pose & pose);
// 2D 位姿转 3D 位姿：yaw 通过 orientationAroundZAxis 构造绕 Z 轴四元数
geometry_msgs::msg::Pose pose2DToPose(const geometry_msgs::msg::Pose2D & pose2d);
// 2D 带戳位姿转 3D 带戳位姿：复用 pose2DToPose，header 原样拷贝
geometry_msgs::msg::PoseStamped pose2DToPoseStamped(
  const nav_2d_msgs::msg::Pose2DStamped & pose2d);
// 2D 位姿 + 指定坐标系/时间戳 转 3D 带戳位姿（header 由参数显式给出）
geometry_msgs::msg::PoseStamped pose2DToPoseStamped(
  const geometry_msgs::msg::Pose2D & pose2d,
  const std::string & frame, const rclcpp::Time & stamp);
// 位姿数组转标准路径消息：header 取第一个位姿的 header（空数组返回空路径）
nav_msgs::msg::Path posesToPath(const std::vector<geometry_msgs::msg::PoseStamped> & poses);
// 标准路径转 2D 路径：逐点位姿调用 poseToPose2D 压平为 (x,y,theta)
nav_2d_msgs::msg::Path2D pathToPath2D(const nav_msgs::msg::Path & path);
// 2D 位姿数组转标准路径：所有位姿共享同一个 frame 与时间戳
nav_msgs::msg::Path poses2DToPath(
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  const std::string & frame, const rclcpp::Time & stamp);
// 2D 路径转标准路径：逐点调用 pose2DToPose，所有点共享路径 header
nav_msgs::msg::Path pathToPath(const nav_2d_msgs::msg::Path2D & path2d);

}  // namespace nav_2d_utils

#endif  // NAV_2D_UTILS__CONVERSIONS_HPP_
