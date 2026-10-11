// Copyright (c) 2020 Samsung Research Russia
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
//  * Neither the name of the <ORGANIZATION> nor the names of its
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
//
// Author: Alexey Merzlyakov

// ============================================================================
// 【文件概况】OccupancyGrid 占据栅格取值常量
// ----------------------------------------------------------------------------
// 概述: 定义 ROS 标准占据栅格 data 数组的三种取值常量（int8_t），
//       为 nav2 中所有涉及占据栅格数值解释的代码提供统一口径。
// 主要内容:
//   - OCC_GRID_UNKNOWN  = -1   未知区域（无观测数据）
//   - OCC_GRID_FREE     = 0    自由区域（无障碍）
//   - OCC_GRID_OCCUPIED = 100  完全占据（确定障碍）
// 使用场景: costmap filter（keepout/speed）翻译掩码值、
//           map 相关工具判断栅格状态等。
// ============================================================================

#ifndef NAV2_UTIL__OCC_GRID_VALUES_HPP_
#define NAV2_UTIL__OCC_GRID_VALUES_HPP_

namespace nav2_util
{

/**
  * @brief OccupancyGrid data constants
  */
// OccupancyGrid（占据栅格）数据的取值常量：
// ROS 标准约定 data 数组中每个 int8_t 单元的取值含义
static constexpr int8_t OCC_GRID_UNKNOWN = -1;   // 未知区域（无观测数据）
static constexpr int8_t OCC_GRID_FREE = 0;       // 完全自由（无障碍）
static constexpr int8_t OCC_GRID_OCCUPIED = 100; // 完全占据（确定障碍）

}  // namespace nav2_util

#endif  // NAV2_UTIL__OCC_GRID_VALUES_HPP_
