/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef DWB_CRITICS__LINE_ITERATOR_HPP_
#define DWB_CRITICS__LINE_ITERATOR_HPP_

#include <stdlib.h>

// ============================================================================
// 模块说明：Bresenham 直线光栅化迭代器（LineIterator）
// ----------------------------------------------------------------------------
// 经典的 Bresenham 直线算法（1975）：在整数栅格上用"整数增量 + 误差累积"
// 的方式逼近一条直线，只经过沿途格子而无需浮点乘除或 sqrt，效率极高。
// 在 DWB 中的用途：ObstacleFootprintCritic 检查机器人 footprint 边界时，
// 需要遍历 footprint 多边形每条边经过的所有格子（线栅格化），配合
// pointCost 检查碰撞。代价地图的膨胀层（inflation layer）也使用同样的
// 算法从障碍物向空地传播代价。
//
// 算法要点（以 x 为主方向为例，即 deltax >= deltay）：
//   - 维护误差分数 num_（初始为 deltax/2，实现四舍五入）；
//   - 每步先把 num_ 加上 deltay（短轴跨度）；
//   - 若 num_ >= den_（长轴跨度 deltax），说明短轴需要前进 1 格：
//     num_ -= den_，并同时步进 x、y（xinc1_/yinc1_ 分支）；
//   - 否则只步进 x（y 不变）；
//   - 循环直到走完 numpixels_（= 长轴跨度）个像素。
// 整数化处理避免了浮点误差累积，保证了算法的确定性与高效性。
// ============================================================================

namespace dwb_critics
{

/** An iterator implementing Bresenham Ray-Tracing. */
// Bresenham 直线迭代器：从 (x0,y0) 到 (x1,y1) 逐格遍历直线穿过的所有栅格
class LineIterator
{
public:
  // 构造函数：接收线段两端点（栅格坐标），初始化所有增量与误差变量
  LineIterator(int x0, int y0, int x1, int y1)
  : x0_(x0),
    y0_(y0),
    x1_(x1),
    y1_(y1),
    x_(x0),    // X and Y start of at first endpoint.
    y_(y0),    // 当前点从第一个端点出发
    deltax_(abs(x1 - x0)),   // x 方向总跨度
    deltay_(abs(y1 - y0)),   // y 方向总跨度
    curpixel_(0)              // 当前已遍历像素计数
  {
    // 确定 x 方向步进符号：终点 x 大于等于起点则 +1，否则 -1
    if (x1_ >= x0_) {                // The x-values are increasing
      xinc1_ = 1;
      xinc2_ = 1;
    } else {                      // The x-values are decreasing
      xinc1_ = -1;
      xinc2_ = -1;
    }

    // 确定 y 方向步进符号：同上
    if (y1_ >= y0_) {               // The y-values are increasing
      yinc1_ = 1;
      yinc2_ = 1;
    } else {                      // The y-values are decreasing
      yinc1_ = -1;
      yinc2_ = -1;
    }

    // 选择"主方向"（跨度更大的轴），决定每次迭代步进哪个坐标：
    if (deltax_ >= deltay_) {        // x 为主方向（每步必走 x，y 视误差而定）
      xinc1_ = 0;                  // Don't change the x when numerator >= denominator
      yinc2_ = 0;                  // Don't change the y for every iteration
      den_ = deltax_;              // 分母 = 主轴跨度
      num_ = deltax_ / 2;          // 分子初值 = 主轴跨度的一半（实现四舍五入起点）
      numadd_ = deltay_;            // 每步累加短轴跨度
      numpixels_ = deltax_;         // There are more x-values than y-values（总像素数 = 主轴跨度）
    } else {                      // y 为主方向（对称处理）
      xinc2_ = 0;                  // Don't change the x for every iteration
      yinc1_ = 0;                  // Don't change the y when numerator >= denominator
      den_ = deltay_;
      num_ = deltay_ / 2;
      numadd_ = deltax_;
      numpixels_ = deltay_;         // There are more y-values than x-values
    }
  }

  // 判断是否还有像素未遍历：当前索引 <= 总像素数
  bool isValid() const
  {
    return curpixel_ <= numpixels_;
  }

  // 前进到直线上的下一个像素：核心的误差累积整数运算
  void advance()
  {
    num_ += numadd_;              // Increase the numerator by the top of the fraction（误差累积）
    if (num_ >= den_) {           // Check if numerator >= denominator（误差溢出：短轴需要步进）
      num_ -= den_;               // Calculate the new numerator value（误差回卷）
      x_ += xinc1_;               // Change the x as appropriate（短轴步进时同时移动 x）
      y_ += yinc1_;               // Change the y as appropriate（短轴步进时同时移动 y）
    }
    x_ += xinc2_;                 // Change the x as appropriate（主轴每步固定步进）
    y_ += yinc2_;                 // Change the y as appropriate（另一轴不动时 inc2=0）

    curpixel_++;                  // 已遍历像素计数 +1
  }

  int getX() const
  {
    return x_;  // 返回当前像素的 x 坐标
  }
  int getY() const
  {
    return y_;  // 返回当前像素的 y 坐标
  }

  int getX0() const
  {
    return x0_;  // 起点 x 坐标
  }
  int getY0() const
  {
    return y0_;  // 起点 y 坐标
  }

  int getX1() const
  {
    return x1_;  // 终点 x 坐标
  }
  int getY1() const
  {
    return y1_;  // 终点 y 坐标
  }

private:
  int x0_;  ///< X coordinate of first end point.
  int y0_;  ///< Y coordinate of first end point.
  int x1_;  ///< X coordinate of second end point.
  int y1_;  ///< Y coordinate of second end point.

  int x_;  ///< X coordinate of current point.
  int y_;  ///< Y coordinate of current point.（当前遍历到的像素坐标）

  int deltax_;  ///< Difference between Xs of endpoints.
  int deltay_;  ///< Difference between Ys of endpoints.

  int curpixel_;  ///< index of current point in line loop.（已遍历像素的索引）

  // 步进增量：inc1 用于误差溢出时同时步进两轴；inc2 用于主轴每步固定步进。
  // 在构造函数中根据主方向的不同，其中各有一个被置零。
  int xinc1_, xinc2_, yinc1_, yinc2_;
  // Bresenham 误差累积变量：num_（分子/误差）、den_（分母/主轴跨度）、
  // numadd_（每步累加的短轴跨度）、numpixels_（总像素数 = 主轴跨度）
  int den_, num_, numadd_, numpixels_;
};

}  // end namespace dwb_critics

#endif  // DWB_CRITICS__LINE_ITERATOR_HPP_
