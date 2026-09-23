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

#ifndef DWB_CRITICS__ALIGNMENT_UTIL_HPP_
#define DWB_CRITICS__ALIGNMENT_UTIL_HPP_

#include "geometry_msgs/msg/pose2_d.hpp"

// ============================================================================
// 模块说明：对齐工具（Alignment Utility）
// ----------------------------------------------------------------------------
// 该头文件为 DWB 的多个"对齐类"评价器（path_align、goal_align 等）提供一个
// 公共的小工具函数 getForwardPose：
//   给定一个位姿 pose，沿其朝向（x 方向）向前（或向后，距离为负）投影
//   distance 米，得到一个新的位姿。
// 核心思想：评价器通过考察"机器人前方一点"（而非机器人中心）到目标/路径的
// 距离，可以隐式地约束机器人的朝向，使轨迹与全局路径或目标方向对齐。
// 数学公式（平面几何）：
//   x' = pose.x + distance * cos(pose.theta)
//   y' = pose.y + distance * sin(pose.theta)
//   theta' = pose.theta（朝向保持不变）
// ============================================================================

namespace dwb_critics
{
/**
 * @brief Projects the given pose forward the specified distance in the x direction.
 * @param pose Input pose
 * @param distance distance to move (in meters)
 * @return Pose distance meters in front of input pose.
 *
 * (used in both path_align and dist_align)
 */
// 将给定位姿沿自身 x 朝向投影指定距离：用于在机器人前方 forward_point_distance
// 米处取一个"虚拟点"，供 path_align / goal_align 评价器评估该点到路径/目标的距离，
// 从而间接引导机器人朝向。distance 为负时表示投影到机器人后方。
geometry_msgs::msg::Pose2D getForwardPose(const geometry_msgs::msg::Pose2D & pose, double distance);

}  // namespace dwb_critics

#endif  // DWB_CRITICS__ALIGNMENT_UTIL_HPP_
