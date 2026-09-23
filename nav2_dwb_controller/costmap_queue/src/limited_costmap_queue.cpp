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

// ============================================================================
// 模块说明：LimitedCostmapQueue 的实现（受限 BFS 扩散）
// ----------------------------------------------------------------------------
// 两个成员函数：
//   构造函数：把基类的 max_distance_ 直接设为外部传入的距离限制（格数），
//     再调用 reset()——reset 里的 computeCache 读到该值后，距离查找表
//     只需覆盖限制范围（而非整张地图），既省内存又省建表时间；
//   validCellToQueue：扩散截断的核心——距离超过限制的格子拒绝入队，
//     BFS 波前到限制边界自动停止。
// ============================================================================

#include "costmap_queue/limited_costmap_queue.hpp"

namespace costmap_queue
{

// 构造：cell_distance_limit 即扩散半径上限（格数）
LimitedCostmapQueue::LimitedCostmapQueue(
  nav2_costmap_2d::Costmap2D & costmap,
  const int distance_limit)
: CostmapQueue(costmap)
{
  // 覆盖基类的 max_distance_（否则基类默认取整张地图的跨度），
  // 使距离缓存表与扩散范围都限制在该值以内
  max_distance_ = distance_limit;
  reset();
}

// 扩散截断钩子：只有距离不超过限制的格子才允许入队
bool LimitedCostmapQueue::validCellToQueue(const CellData & cell)
{
  return cell.distance_ <= max_distance_;
}

}  // namespace costmap_queue
