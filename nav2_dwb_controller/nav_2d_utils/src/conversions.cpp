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

#include "nav_2d_utils/conversions.hpp"

#include <vector>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

#include "nav2_util/geometry_utils.hpp"

namespace nav_2d_utils
{
using nav2_util::geometry_utils::orientationAroundZAxis;

// 2D 速度 -> 3D 速度消息：把平面 (x, y, theta) 映射到 Twist 的
// linear.x / linear.y / angular.z 三个字段（其余字段保持默认 0）
geometry_msgs::msg::Twist twist2Dto3D(const nav_2d_msgs::msg::Twist2D & cmd_vel_2d)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = cmd_vel_2d.x;      // 前向线速度
  cmd_vel.linear.y = cmd_vel_2d.y;      // 横向线速度（全向底盘用）
  cmd_vel.angular.z = cmd_vel_2d.theta; // 绕 z 轴角速度
  return cmd_vel;
}

// 3D 速度 -> 2D 速度消息：反向映射，仅保留平面运动三分量
nav_2d_msgs::msg::Twist2D twist3Dto2D(const geometry_msgs::msg::Twist & cmd_vel)
{
  nav_2d_msgs::msg::Twist2D cmd_vel_2d;
  cmd_vel_2d.x = cmd_vel.linear.x;
  cmd_vel_2d.y = cmd_vel.linear.y;
  cmd_vel_2d.theta = cmd_vel.angular.z;
  return cmd_vel_2d;
}

// （历史遗留的注释代码：tf2::Stamped 直接转 Pose2DStamped，已弃用）

// PoseStamped -> Pose2DStamped：header 原样拷贝；位置取 x/y；
// 四元数姿态用 tf2::getYaw 压平为单一偏航角（丢失俯仰/横滚）
nav_2d_msgs::msg::Pose2DStamped poseStampedToPose2D(const geometry_msgs::msg::PoseStamped & pose)
{
  nav_2d_msgs::msg::Pose2DStamped pose2d;
  pose2d.header = pose.header;
  pose2d.pose.x = pose.pose.position.x;
  pose2d.pose.y = pose.pose.position.y;
  pose2d.pose.theta = tf2::getYaw(pose.pose.orientation);  // 四元数 -> yaw
  return pose2d;
}

// Pose -> Pose2D：同上但不含 header
geometry_msgs::msg::Pose2D poseToPose2D(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = pose.position.x;
  pose2d.y = pose.position.y;
  pose2d.theta = tf2::getYaw(pose.orientation);
  return pose2d;
}

// Pose2D -> Pose：反向转换，yaw 角通过 orientationAroundZAxis 构造
// 等价的绕 Z 轴单位四元数（qx=qy=0, qz=sin(theta/2), qw=cos(theta/2)）
geometry_msgs::msg::Pose pose2DToPose(const geometry_msgs::msg::Pose2D & pose2d)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = pose2d.x;
  pose.position.y = pose2d.y;
  pose.orientation = orientationAroundZAxis(pose2d.theta);  // yaw -> 四元数
  return pose;
}

// Pose2DStamped -> PoseStamped：header 拷贝 + 复用 pose2DToPose
geometry_msgs::msg::PoseStamped pose2DToPoseStamped(
  const nav_2d_msgs::msg::Pose2DStamped & pose2d)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = pose2d.header;
  pose.pose = pose2DToPose(pose2d.pose);
  return pose;
}

// Pose2D + 显式 frame/stamp -> PoseStamped：header 由参数直接指定，
// 适用于源数据没有 header 的场合（如从数组恢复路径时统一打时间戳）
geometry_msgs::msg::PoseStamped pose2DToPoseStamped(
  const geometry_msgs::msg::Pose2D & pose2d,
  const std::string & frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp = stamp;
  pose.pose.position.x = pose2d.x;
  pose.pose.position.y = pose2d.y;
  pose.pose.orientation = orientationAroundZAxis(pose2d.theta);
  return pose;
}

// PoseStamped 数组 -> Path：header 沿用第一个位姿的 header；
// 空数组直接返回空路径（frame_id 为空）
nav_msgs::msg::Path posesToPath(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  nav_msgs::msg::Path path;
  if (poses.empty()) {
    return path;
  }
  path.poses.resize(poses.size());
  path.header.frame_id = poses[0].header.frame_id;
  path.header.stamp = poses[0].header.stamp;
  for (unsigned int i = 0; i < poses.size(); i++) {
    path.poses[i] = poses[i];   // 逐位姿拷贝
  }
  return path;
}

// Path -> Path2D：header 拷贝；逐点位姿用 poseToPose2D 压平为 (x,y,theta)
nav_2d_msgs::msg::Path2D pathToPath2D(const nav_msgs::msg::Path & path)
{
  nav_2d_msgs::msg::Path2D path2d;
  path2d.header = path.header;
  for (auto & pose : path.poses) {
    path2d.poses.push_back(poseToPose2D(pose.pose));
  }
  return path2d;
}


// Pose2D 数组 + 统一 frame/stamp -> Path：每个点都调用带显式
// header 的 pose2DToPoseStamped，保证整条路径时间戳一致
nav_msgs::msg::Path poses2DToPath(
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  const std::string & frame, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.poses.resize(poses.size());
  path.header.frame_id = frame;
  path.header.stamp = stamp;
  for (unsigned int i = 0; i < poses.size(); i++) {
    path.poses[i] = pose2DToPoseStamped(poses[i], frame, stamp);
  }
  return path;
}

// Path2D -> Path：header 拷贝到路径与每个位姿；逐点 pose2DToPose 恢复四元数
nav_msgs::msg::Path pathToPath(const nav_2d_msgs::msg::Path2D & path2d)
{
  nav_msgs::msg::Path path;
  path.header = path2d.header;
  path.poses.resize(path2d.poses.size());
  for (unsigned int i = 0; i < path.poses.size(); i++) {
    path.poses[i].header = path2d.header;
    path.poses[i].pose = pose2DToPose(path2d.poses[i]);
  }
  return path;
}

}  // namespace nav_2d_utils
