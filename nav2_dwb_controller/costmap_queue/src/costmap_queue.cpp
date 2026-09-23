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
// 模块说明：CostmapQueue 的实现（BFS 波前扩散）
// ----------------------------------------------------------------------------
// 实现代价地图队列的核心逻辑：reset（重置已访问表与距离缓存表）、
// enqueueCell（带距离的入队）、getNextCell（出队并向四邻域扩散）、
// computeCache（预计算 dx-dy 距离查找表）。
// 算法特性：
//   - 每格最多入队一次（seen_ 数组去重）；
//   - 出队顺序按距离单调不减（MapBasedQueue 保证），
//     因此每格第一次出队时即获得到最近源点的正确距离（Dijkstra 性质）；
//   - 距离通过查表获得，不做任何实时 sqrt。
// ============================================================================

#include "costmap_queue/costmap_queue.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

using std::hypot;

namespace costmap_queue
{

// 构造函数：初始化基类（优先队列）、max_distance_=-1 表示"尚未确定
// 距离表跨度"（首次 computeCache 时取地图长宽的最大值）
CostmapQueue::CostmapQueue(nav2_costmap_2d::Costmap2D & costmap, bool manhattan)
: MapBasedQueue(), costmap_(costmap), max_distance_(-1), manhattan_(manhattan),
  cached_max_distance_(-1)
{
  reset();
}

// 重置队列：准备一次全新的扩散过程
void CostmapQueue::reset()
{
  unsigned int size_x = costmap_.getSizeInCellsX(), size_y = costmap_.getSizeInCellsY();
  // seen_ 数组与地图同尺寸（地图尺寸变化时才重新分配，避免频繁分配）
  if (seen_.size() != size_x * size_y) {
    seen_.resize(size_x * size_y);
  }
  // 全部标记为"未访问"
  std::fill(seen_.begin(), seen_.end(), false);
  // 重建距离查找表（若跨度没变则直接复用旧表）
  computeCache();
  // 清空底层优先队列
  MapBasedQueue::reset();
}

// 源格入队：源点就是它自己，距离自然为 0（查表 [0][0]）
void CostmapQueue::enqueueCell(unsigned int x, unsigned int y)
{
  unsigned int index = costmap_.getIndex(x, y);
  enqueueCell(index, x, y, x, y);
}

// 内部入队：把 (cur_x, cur_y) 格以 (src_x, src_y) 为源点加入队列
void CostmapQueue::enqueueCell(
  unsigned int index, unsigned int cur_x, unsigned int cur_y,
  unsigned int src_x, unsigned int src_y)
{
  // 已入过队的格子直接跳过（BFS 的关键去重，保证每格只处理一次）
  if (seen_[index]) {return;}

  // we compute our distance table one cell further than the inflation radius
  // dictates so we can make the check below
  // 查预计算表得到当前格到源点的距离（注意：距离表比所需范围多算一格，
  // 以便 validCellToQueue 能对边界外一格的格子做出判断）
  double distance = distanceLookup(cur_x, cur_y, src_x, src_y);
  CellData data(distance, index, cur_x, cur_y, src_x, src_y);
  // 钩子：询问派生类该格是否允许入队（LimitedCostmapQueue 在此截断扩散）
  if (validCellToQueue(data)) {
    seen_[index] = true;   // 标记已入队
    enqueue(distance, data);  // 以距离为优先级压入底层 MapBasedQueue
  }
}

// 取出下一个待处理格子，并把它的四邻域入队（BFS 的"扩散"步骤）
CellData CostmapQueue::getNextCell()
{
  // get the highest priority cell and pop it off the priority queue
  // 取出距离最小（最高优先级）的格子并弹出
  CellData current_cell = front();
  pop();

  // 当前格子与源点的坐标
  unsigned int index = current_cell.index_;
  unsigned int mx = current_cell.x_;
  unsigned int my = current_cell.y_;
  unsigned int sx = current_cell.src_x_;
  unsigned int sy = current_cell.src_y_;

  // attempt to put the neighbors of the current cell onto the queue
  // 把当前格的四个邻居入队（邻居继承同一个源点），注意边界检查。
  // 一维索引的算术等价于二维坐标的移动：
  //   x-1 => index-1；y-1 => index-size_x；x+1 => index+1；y+1 => index+size_x
  unsigned int size_x = costmap_.getSizeInCellsX();
  if (mx > 0) {                    // 左邻（不在地图左边界时）
    enqueueCell(index - 1, mx - 1, my, sx, sy);
  }
  if (my > 0) {                    // 下邻（不在地图下边界时）
    enqueueCell(index - size_x, mx, my - 1, sx, sy);
  }
  if (mx < size_x - 1) {           // 右邻（不在地图右边界时）
    enqueueCell(index + 1, mx + 1, my, sx, sy);
  }
  if (my < costmap_.getSizeInCellsY() - 1) {  // 上邻（不在地图上边界时）
    enqueueCell(index + size_x, mx, my + 1, sx, sy);
  }

  return current_cell;             // 返回本格数据（含距离与源点信息）
}

// 预计算距离查找表：cached_distances_[dx][dy] = f(dx, dy)
void CostmapQueue::computeCache()
{
  // 距离表跨度未指定时，取地图长宽的最大值（保证覆盖最远格子对）
  if (max_distance_ == -1) {
    max_distance_ = std::max(costmap_.getSizeInCellsX(), costmap_.getSizeInCellsY());
  }
  // 跨度与上次相同则直接复用旧表，避免重复计算
  if (max_distance_ == cached_max_distance_) {return;}
  cached_distances_.clear();

  // +2：故意比 max_distance_ 多算两格，供边界判断（见 enqueueCell 注释）
  cached_distances_.resize(max_distance_ + 2);

  for (unsigned int i = 0; i < cached_distances_.size(); ++i) {
    cached_distances_[i].resize(max_distance_ + 2);
    for (unsigned int j = 0; j < cached_distances_[i].size(); ++j) {
      if (manhattan_) {
        cached_distances_[i][j] = i + j;       // 曼哈顿距离：|dx|+|dy|
      } else {
        cached_distances_[i][j] = hypot(i, j); // 欧氏距离：sqrt(dx^2+dy^2)
      }
    }
  }
  cached_max_distance_ = max_distance_;  // 记录本次表对应的跨度
}

}  // namespace costmap_queue
