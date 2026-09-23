// Copyright (c) 2022 Samsung R&D Institute Russia
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
// 文件说明（中文）：
// 碰撞监视器的运动学工具函数（kinematics）实现。
// 模块定位：为碰撞预测提供两个基础数学操作：
//   1. transformPoints：把一组点从旧坐标系旋转变换到新坐标系
//      （用于把障碍物点变换到机器人当前/预测位姿坐标系下检测）；
//   2. projectState：按当前速度把机器人位姿前推 dt 时间，
//      得到预测位姿与预测速度（用于碰撞趋势预测）。
// ============================================================================

#include "nav2_collision_monitor/kinematics.hpp"

#include <cmath>

namespace nav2_collision_monitor
{

// ---------------------------------------------------------------------------
// 坐标系变换：把 points 从旧坐标系变换到以 pose 为原点的新坐标系。
// 数学：p' = Rt * (p - pose)，Rt 为旋转矩阵的转置（逆），
//   R = [cos -sin; sin cos]
// 即先平移（减去新原点），再反向旋转（角度 -theta）。
// ---------------------------------------------------------------------------
void transformPoints(const Pose & pose, std::vector<Point> & points)
{
  const double cos_theta = std::cos(pose.theta);
  const double sin_theta = std::sin(pose.theta);

  for (Point & point : points) {
    // p = R*p' + pose
    // p' = Rt * (p - pose)
    // where:
    //   p - point coordinates in initial frame
    //   p' - point coordinates in a new frame
    //   R - rotation matrix =
    //     [cos_theta -sin_theta]
    //     [sin_theta  cos_theta]
    //   Rt - transposed (inverted) rotation matrix
    const double mul_x = point.x - pose.x;
    const double mul_y = point.y - pose.y;
    // 应用逆旋转
    point.x = mul_x * cos_theta + mul_y * sin_theta;
    point.y = -mul_x * sin_theta + mul_y * cos_theta;
  }
}

// ---------------------------------------------------------------------------
// 状态前推（预测）：按当前速度把位姿推算 dt 时间后的状态。
//   位置：p' = p + vel*dt（平移）
//   朝向：theta' = theta + w*dt（角速度积分）
//   速度：vel' = R(theta')*vel（速度向量随新朝向旋转，线速度大小不变）
// ---------------------------------------------------------------------------
void projectState(const double & dt, Pose & pose, Velocity & velocity)
{
  const double theta = velocity.tw * dt;
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);

  // p' = p + vel*dt
  // where:
  //   p - initial pose
  //   p' - projected pose
  pose.x = pose.x + velocity.x * dt;
  pose.y = pose.y + velocity.y * dt;
  // Rotate the pose on theta
  pose.theta = pose.theta + theta;

  // vel' = R*vel
  // where:
  //   vel - initial velocity
  //   R - rotation matrix
  //   vel' - rotated velocity
  const double velocity_upd_x = velocity.x * cos_theta - velocity.y * sin_theta;
  const double velocity_upd_y = velocity.x * sin_theta + velocity.y * cos_theta;
  velocity.x = velocity_upd_x;
  velocity.y = velocity_upd_y;
}

}  // namespace nav2_collision_monitor