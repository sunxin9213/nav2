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

#ifndef NAV_2D_UTILS__PATH_OPS_HPP_
#define NAV_2D_UTILS__PATH_OPS_HPP_

#include "nav_2d_msgs/msg/path2_d.hpp"

namespace nav_2d_utils
{
/**
 * @brief Increase plan resolution to match that of the costmap by adding points linearly between points
 *
 * @param global_plan_in input plan
 * @param resolution desired distance between waypoints
 * @return Higher resolution plan
 */
// ---------------------------------------------------------------------------
// 路径分辨率调整（adjustPlanResolution）：
// 全局规划器输出的路径点间距可能远大于局部代价地图的分辨率（例如全局
// 规划在粗粒度栅格上搜索，或路径被抽稀过）。BFW/DWB 的评价器（尤其是
// map_grid 系列）以路径点为源点做 BFS，点距过稀会导致距离场精度不足。
// 本函数在相邻路径点间距超过 2*resolution（即"空洞"超过两个格子）时，
// 按分辨率线性内插补齐中间点（x/y/theta 均匀插值），输出与代价地图
// 分辨率匹配的高密度路径。
// ---------------------------------------------------------------------------
// 提升路径分辨率：在相邻点间距过大处按 resolution 线性插入中间路径点
nav_2d_msgs::msg::Path2D adjustPlanResolution(
  const nav_2d_msgs::msg::Path2D & global_plan_in,
  double resolution);
}  // namespace nav_2d_utils

#endif  // NAV_2D_UTILS__PATH_OPS_HPP_
