/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// ============================================================================
// 中文注释：代价地图数学工具（costmap_math）实现文件
//
// 【文件功能】
// 提供代价地图相关的通用几何/数学工具函数。大部分函数以 inline 形式
// 定义在 costmap_math.hpp 中（如 distance、dist、sign、bresenham 等），
// 本 .cpp 文件只实现在头文件中不便内联的较复杂函数。
//
// 【在 Nav2 中的位置】
// 被 InflationLayer（膨胀层）、ObstacleLayer（障碍层）等广泛引用，
// 是代价地图体系的公共数学基础库。
// ============================================================================

#include <nav2_costmap_2d/costmap_math.hpp>

#include <vector>

// ============================================================================
// distanceToLine()：计算点 (pX,pY) 到线段 (x0,y0)-(x1,y1) 的最短距离
// 算法：
//   1) 求点在线段方向上的投影参数 param = dot/|len|^2 ∈ [0,1]；
//   2) param<0 时最近点为线段起点；param>1 时为线段终点；
//      否则为线段上的投影点（点到线段的垂足）；
//   3) 返回点到该最近点的欧氏距离。
// 用途：判断机器人足迹/路径与障碍物的距离关系。
// ============================================================================
double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1)
{
  // 向量：起点→目标点 (A,B)；起点→终点 (C,D)
  double A = pX - x0;
  double B = pY - y0;
  double C = x1 - x0;
  double D = y1 - y0;

  // 投影参数：dot/|CD|^2，表示垂足在线段上的相对位置
  double dot = A * C + B * D;
  double len_sq = C * C + D * D;
  double param = dot / len_sq;

  double xx, yy;

  if (param < 0) {
    // 垂足在线段起点之前 → 最近点为起点
    xx = x0;
    yy = y0;
  } else if (param > 1) {
    // 垂足在线段终点之后 → 最近点为终点
    xx = x1;
    yy = y1;
  } else {
    // 垂足在线段上 → 最近点为投影点
    xx = x0 + param * C;
    yy = y0 + param * D;
  }

  // 返回点到最近点的欧氏距离（distance 为头文件中的内联函数）
  return distance(pX, pY, xx, yy);
}
