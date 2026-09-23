/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef NAV2_COSTMAP_2D__COSTMAP_MATH_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_MATH_HPP_
// ============================================================================
// 中文注释：代价地图数学工具头文件（少量实现见 costmap_math.cpp）
//
// 【文件功能】
// 公共数学基础库，被膨胀层/障碍层/足迹工具等广泛引用：
//   - distance/dist：2D/3D 欧氏距离（inline）；
//   - distanceToLine：点到线段最短距离（见 .cpp）；
//   - sign/sign0：取符号（sign0(0)=0）；
//   - to_prob/to_cost：概率 [0,1] ↔ 代价编码 [0,255] 互转
//     （0.5 概率=无信息，RangeSensorLayer 使用）；
//   - orient2d：三角形有向面积二倍值（重心坐标/点在三角形内判断，
//     RangeSensorLayer 的锥形视场判定使用）。
// ============================================================================

#include <math.h>
#include <algorithm>
#include <vector>

#include "geometry_msgs/msg/point.hpp"

/** @brief Return -1 if x < 0, +1 otherwise. */
inline double sign(double x)
{
  return x < 0.0 ? -1.0 : 1.0;
}

/** @brief Same as sign(x) but returns 0 if x is 0. */
inline double sign0(double x)
{
  return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}

/** @brief Gets L2 norm distance */
inline double distance(double x0, double y0, double x1, double y1)
{
  return hypot(x1 - x0, y1 - y0);
}

/** @brief Gets point distance to a line */
double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1);

#endif  // NAV2_COSTMAP_2D__COSTMAP_MATH_HPP_
