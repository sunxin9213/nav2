/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Samsung Research Russia
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
 *   * Neither the name of the <ORGANIZATION> nor the names of its
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
 * Author: Alexey Merzlyakov
 *********************************************************************/

#ifndef NAV2_COSTMAP_2D__COSTMAP_FILTERS__FILTER_VALUES_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_FILTERS__FILTER_VALUES_HPP_
// ============================================================================
// 中文注释：过滤器公共常量头文件
//
// 【文件功能】
// 定义代价地图过滤器体系的公共常量：
//   - BASE_DEFAULT / MULTIPLIER_DEFAULT：CostmapFilterInfo 中
//     base/multiplier 的默认值（Keepout 等不使用换算的过滤器必须保持默认）；
//   - SPEED_FILTER_PERCENT / SPEED_FILTER_ABSOLUTE：限速模式
//     （百分比 / 绝对 m/s）；
//   - BINARY_FILTER：二值过滤器类型标识；
//   - NO_SPEED_LIMIT：无限速标记值。
// ============================================================================

/** Provides constants used in costmap filters */

namespace nav2_costmap_2d
{

/** Types of costmap filter */
static constexpr uint8_t KEEPOUT_FILTER = 0;
static constexpr uint8_t SPEED_FILTER_PERCENT = 1;
static constexpr uint8_t SPEED_FILTER_ABSOLUTE = 2;
static constexpr uint8_t BINARY_FILTER = 3;

/** Default values for base and multiplier */
static constexpr double BASE_DEFAULT = 0.0;
static constexpr double MULTIPLIER_DEFAULT = 1.0;

/** Speed filter constants */
static constexpr int8_t SPEED_MASK_UNKNOWN = -1;
static constexpr int8_t SPEED_MASK_NO_LIMIT = 0;
static constexpr double NO_SPEED_LIMIT = 0.0;

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_FILTERS__FILTER_VALUES_HPP_
