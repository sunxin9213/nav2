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

#include "nav_2d_utils/path_ops.hpp"
#include <cmath>

using std::sqrt;

namespace nav_2d_utils
{
// 提升路径分辨率：详见头文件注释。核心是把间距超过 2 格的相邻路径点
// 之间按 resolution 线性内插补点，使路径密度与代价地图分辨率匹配。
nav_2d_msgs::msg::Path2D adjustPlanResolution(
  const nav_2d_msgs::msg::Path2D & global_plan_in,
  double resolution)
{
  nav_2d_msgs::msg::Path2D global_plan_out;
  // 空路径直接返回
  if (global_plan_in.poses.size() == 0) {
    return global_plan_out;
  }

  // last 记录上一个已输出的路径点（从第一点开始，首点直接放入输出）
  geometry_msgs::msg::Pose2D last = global_plan_in.poses[0];
  global_plan_out.poses.push_back(last);

  // we can take "holes" in the plan smaller than 2 grid cells (squared = 4)
  // 判断"空洞"的阈值：间距平方 > (2*resolution)^2 才需要补点。
  // 小于两个格子的间距视为可接受，不补
  double min_sq_resolution = resolution * resolution * 4.0;

  // 从第二个点开始逐点检查与上一点的间距
  for (unsigned int i = 1; i < global_plan_in.poses.size(); ++i) {
    geometry_msgs::msg::Pose2D loop = global_plan_in.poses[i];
    // 当前点与上一点的欧氏距离平方（比较用平方避免开方）
    double sq_dist = (loop.x - last.x) * (loop.x - last.x) + (loop.y - last.y) * (loop.y - last.y);
    if (sq_dist > min_sq_resolution) {
      // add points in-between
      // 需要补点：先算超出阈值部分的距离，再除以分辨率得到步数
      double diff = sqrt(sq_dist) - sqrt(min_sq_resolution);
      int steps = static_cast<int>(diff / resolution) - 1;
      double steps_double = static_cast<double>(steps);

      // 每一步的 x/y/theta 增量（线性插值，theta 也是线性内插）
      double delta_x = (loop.x - last.x) / steps_double;
      double delta_y = (loop.y - last.y) / steps_double;
      double delta_t = (loop.theta - last.theta) / steps_double;

      // 生成中间点（j 从 1 到 steps-1，两端点由 last 与 loop 本身承担）
      for (int j = 1; j < steps; ++j) {
        geometry_msgs::msg::Pose2D pose;
        pose.x = last.x + j * delta_x;
        pose.y = last.y + j * delta_y;
        pose.theta = last.theta + j * delta_t;
        global_plan_out.poses.push_back(pose);
      }
    }
    // 原始路径点本身无论如何都保留在输出中
    global_plan_out.poses.push_back(global_plan_in.poses[i]);
    last.x = loop.x;   // 更新 last 为当前点（只关心位置，用于距离比较）
    last.y = loop.y;
  }
  return global_plan_out;
}
}  // namespace nav_2d_utils
