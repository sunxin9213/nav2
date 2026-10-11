// Copyright (c) 2012, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// ============================================================================
// 【文件概况】Bresenham 直线光栅化迭代器（header-only）
// ----------------------------------------------------------------------------
// 概述: 以纯整数运算逐像素遍历栅格地图上 (x0,y0)→(x1,y1) 的直线，
//       无浮点运算与三角函数，天然适合栅格地图上的射线追踪。
// 主要内容:
//   1. 构造函数: 确定主/副轴、步进方向与误差累积初值（算法核心）
//   2. isValid(): 是否还有未遍历的像素
//   3. advance(): 前进一步（主轴每步必进 1，副轴按累积误差决定）
//   4. getX()/getY(): 当前像素坐标; getX0/Y0/X1/Y1: 两端点坐标
// 使用场景: costmap 的 obstacle/voxel layer 做传感器视线清除（raytrace）、
//           规划器做直线碰撞检查等一切"遍历栅格直线上所有格子"的场合。
// ============================================================================

#ifndef NAV2_UTIL__LINE_ITERATOR_HPP_
#define NAV2_UTIL__LINE_ITERATOR_HPP_

#include <stdlib.h>

namespace nav2_util
{


/**
 * @class nav2_util::LineIterator
 * @brief An iterator implementing Bresenham Ray-Tracing.
 * Bresenham 直线光栅化迭代器：以整数运算逐点遍历栅格地图上
 * (x0,y0) 到 (x1,y1) 之间的直线像素，广泛用于代价地图的
 * 射线追踪（如清除传感器视线上的栅格）。
 */
class LineIterator
{
public:
  /**
   * @brief A constructor for LineIterator
   * @param x0 Starting x
   * @param y0 Starting y
   * @param x1 Ending x
   * @param y1 Ending y
   */
  LineIterator(int x0, int y0, int x1, int y1)
  : x0_(x0),
    y0_(y0),
    x1_(x1),
    y1_(y1),
    x_(x0),    // X and Y start of at first endpoint.
    y_(y0),
    deltax_(abs(x1 - x0)),
    deltay_(abs(y1 - y0)),
    curpixel_(0)
  {
    // 根据终点相对起点的方向设置 x/y 步进符号（+1 或 -1）
    if (x1_ >= x0_) {                // The x-values are increasing
      xinc1_ = 1;
      xinc2_ = 1;
    } else {                      // The x-values are decreasing
      xinc1_ = -1;
      xinc2_ = -1;
    }

    if (y1_ >= y0_) {               // The y-values are increasing
      yinc1_ = 1;
      yinc2_ = 1;
    } else {                      // The y-values are decreasing
      yinc1_ = -1;
      yinc2_ = -1;
    }

    // 以增量较大（更陡/更平缓）的轴为主轴：
    // 主轴每步必进 1，副轴按累积误差决定是否进 1，这正是 Bresenham 算法核心
    if (deltax_ >= deltay_) {        // There is at least one x-value for every y-value
      xinc1_ = 0;                  // Don't change the x when numerator >= denominator
      yinc2_ = 0;                  // Don't change the y for every iteration
      den_ = deltax_;              // 分母：主轴总增量
      num_ = deltax_ / 2;          // 分子：误差累积器，初值取半避免偏向
      numadd_ = deltay_;           // 每步加入分子：副轴总增量
      numpixels_ = deltax_;         // There are more x-values than y-values
    } else {                      // There is at least one y-value for every x-value
      xinc2_ = 0;                  // Don't change the x for every iteration
      yinc1_ = 0;                  // Don't change the y when numerator >= denominator
      den_ = deltay_;
      num_ = deltay_ / 2;
      numadd_ = deltax_;
      numpixels_ = deltay_;         // There are more y-values than x-values
    }
  }

  /**
   * @brief If the iterator is valid
   * @return bool If valid
   */
  // 是否还有可迭代的像素点（当前序号未超过总像素数）
  bool isValid() const
  {
    return curpixel_ <= numpixels_;
  }

  /**
   * @brief Advance iteration along the line
   */
  // 前进到直线下一个像素点：误差累积 + 判断副轴是否步进
  void advance()
  {
    num_ += numadd_;              // Increase the numerator by the top of the fraction
    if (num_ >= den_) {           // Check if numerator >= denominator
      // 误差溢出：副轴额外步进一次，并把误差回绕
      num_ -= den_;               // Calculate the new numerator value
      x_ += xinc1_;               // Change the x as appropriate
      y_ += yinc1_;               // Change the y as appropriate
    }
    // 主轴每步固定步进一次
    x_ += xinc2_;                 // Change the x as appropriate
    y_ += yinc2_;                 // Change the y as appropriate

    curpixel_++;
  }

  /**
   * @brief Get current X value
   * @return X
   */
  int getX() const
  {
    return x_;
  }

  /**
   * @brief Get current Y value
   * @return Y
   */
  int getY() const
  {
    return y_;
  }

  /**
   * @brief Get initial X value
   * @return X
   */
  int getX0() const
  {
    return x0_;
  }

  /**
   * @brief Get initial Y value
   * @return Y
   */
  int getY0() const
  {
    return y0_;
  }

  /**
   * @brief Get terminal X value
   * @return X
   */
  int getX1() const
  {
    return x1_;
  }

  /**
   * @brief Get terminal Y value
   * @return Y
   */
  int getY1() const
  {
    return y1_;
  }

private:
  // 线段两端点坐标
  int x0_;  ///< X coordinate of first end point.
  int y0_;  ///< Y coordinate of first end point.
  int x1_;  ///< X coordinate of second end point.
  int y1_;  ///< Y coordinate of second end point.

  // 当前迭代点坐标
  int x_;  ///< X coordinate of current point.
  int y_;  ///< Y coordinate of current point.

  // 两轴的总增量（绝对值），用于确定主轴
  int deltax_;  ///< Difference between Xs of endpoints.
  int deltay_;  ///< Difference between Ys of endpoints.

  int curpixel_;  ///< index of current point in line loop.
  // 步进符号：inc1 用于误差溢出时的副轴步进，inc2 用于主轴的固定步进，
  // 其中被置 0 的那组表示该轴在对应时机不步进（见构造函数中的主副轴判定）
  int xinc1_, xinc2_, yinc1_, yinc2_;
  // Bresenham 误差累积：den 分母、num 分子(误差)、numadd 每步增量、numpixels 总像素数
  int den_, num_, numadd_, numpixels_;
};

}  // end namespace nav2_util

#endif  // NAV2_UTIL__LINE_ITERATOR_HPP_
