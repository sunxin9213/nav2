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
 *********************************************************************/
#ifndef NAV2_COSTMAP_2D__COST_VALUES_HPP_
#define NAV2_COSTMAP_2D__COST_VALUES_HPP_
// ============================================================================
// 中文注释：代价地图语义值定义头文件（最常被引用的常量）
//
// 【代价语义】（unsigned char，0~255，255 也用于未知）
//  FREE_SPACE(0)                  —— 自由空间，无任何代价
//  1~252（含 MAX_NON_OBSTACLE=252）—— 中间代价：膨胀层按离障碍的
//       距离指数衰减生成，值越大越"危险"（越接近障碍）
//  INSCRIBED_INFLATED_OBSTACLE(253) —— 内切障碍：机器人中心到障碍
//       的距离小于足迹内切半径，中心进入此区域必然碰撞（无论朝向）
//  LETHAL_OBSTACLE(254)           —— 致命障碍：栅格本身被占据，
//       机器人任何部分碰到都碰撞；也是膨胀的"种子"
//  NO_INFORMATION(255)            —— 未知区域：传感器未覆盖或地图外，
//       track_unknown_space 参数决定规划器是否允许穿越
// ============================================================================
/** Provides a mapping for often used cost values */
namespace nav2_costmap_2d
{
static constexpr unsigned char NO_INFORMATION = 255;
static constexpr unsigned char LETHAL_OBSTACLE = 254;
static constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static constexpr unsigned char MAX_NON_OBSTACLE = 252;
static constexpr unsigned char FREE_SPACE = 0;
}
#endif  // NAV2_COSTMAP_2D__COST_VALUES_HPP_
