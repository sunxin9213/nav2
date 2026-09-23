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

#ifndef COSTMAP_QUEUE__LIMITED_COSTMAP_QUEUE_HPP_
#define COSTMAP_QUEUE__LIMITED_COSTMAP_QUEUE_HPP_

#include "costmap_queue/costmap_queue.hpp"

// ============================================================================
// 模块说明：受限代价地图队列（LimitedCostmapQueue）
// ----------------------------------------------------------------------------
// CostmapQueue 的受限版本：BFS 扩散只延伸到距源点 cell_distance_limit
// 个格子以内，超出即不再入队。
// 使用场景：当只需要源点附近的距离场时（例如膨胀层只需要膨胀半径内的
// 距离、DWB 评价器只需局部范围），受限扩散可以大幅减少计算量——
// 从 O(整张地图) 降到 O(限制半径的平方)。
// 实现方式：重写 validCellToQueue 钩子，把 distance_ > max_distance_ 的
// 格子直接拒之门外；同时在构造时把 max_distance_ 设为限制值，距离查找表
// 也只需覆盖该范围。
// ============================================================================

namespace costmap_queue
{

/**
 * @class LimitedCostmapQueue
 * @brief Extension of Costmap Queue where distances are limited to a given distance from source cells.
 */
// 受限扩散队列：限制 BFS 波前只传播到指定格子距离以内
class LimitedCostmapQueue : public CostmapQueue
{
public:
  /**
   * @brief Constructor with limit as an integer number of cells.
   */
  // 构造：cell_distance_limit 为最大扩散距离（格数），内部设置 max_distance_ 后 reset
  LimitedCostmapQueue(nav2_costmap_2d::Costmap2D & costmap, const int cell_distance_limit);
  // 只允许距离 <= max_distance_ 的格子入队（超出限制即截断扩散）
  bool validCellToQueue(const CellData & cell) override;
};
}  // namespace costmap_queue

#endif  // COSTMAP_QUEUE__LIMITED_COSTMAP_QUEUE_HPP_
