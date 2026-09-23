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

#ifndef COSTMAP_QUEUE__COSTMAP_QUEUE_HPP_
#define COSTMAP_QUEUE__COSTMAP_QUEUE_HPP_

#include <cmath>
#include <vector>
#include <limits>
#include <memory>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "costmap_queue/map_based_queue.hpp"

// ============================================================================
// 模块说明：代价地图 BFS 扩散队列（CostmapQueue）
// ----------------------------------------------------------------------------
// costmap_queue 包的核心类。它解决的问题：给定代价地图上一组"源格"
// （source cells，例如障碍物格或路径点格），计算其他所有格子各自离哪个
// 源格最近、距离多少。这是导航领域的经典"波前扩散"（wavefront
// propagation）问题：
//   - 膨胀层（InflationLayer）用它计算每格到最近障碍物的距离，
//     从而按指数衰减公式赋予代价值；
//   - DWB 的 map_grid 系列评价器用它计算每格到全局路径/目标的距离场。
//
// 算法本质（Dijkstra 的整数化特例 / BFS）：
//   1. 所有源格入队（enqueueCell，源格自身即源点，距离 0）；
//   2. 循环取队首（getNextCell，距离最小的格），将其四邻域格入队，
//      邻居的"源点"继承自当前格，距离 = 预计算的距离表中查表值；
//   3. seen_ 数组保证每格只入队一次（贪心最优：距离小的先出队）。
//
// 距离度量：默认欧氏距离（hypot）；manhattan=true 时用曼哈顿距离
// （DWB 的 MapGridCritic 用曼哈顿距离，膨胀层用欧氏距离）。
// 距离不实时计算，而是预计算成二维查表 cached_distances_[dx][dy]，
// 每格只需一次 O(1) 查表。
//
// 扩展点：虚函数 validCellToQueue 允许派生类限制扩散范围——
// LimitedCostmapQueue 用它实现"只扩散到距离上限以内"。
// ============================================================================

namespace costmap_queue
{
/**
 * @class CellData
 * @brief Storage for cell information used during queue expansion
 */
// 队列元素：记录扩散过程中一个格子的全部信息
class CellData
{
public:
  /**
   * @brief Real Constructor
   * @param d The distance to the nearest obstacle
   * @param i The index of the cell in the costmap. Redundant with the following two parameters.
   * @param x The x coordinate of the cell in the cost map
   * @param y The y coordinate of the cell in the cost map
   * @param sx The x coordinate of the closest source cell in the costmap
   * @param sy The y coordinate of the closest source cell in the costmap
   */
  // 完整构造：距离 d、格子一维索引 i、格子坐标 (x,y)、最近源点坐标 (sx,sy)
  CellData(
    const double d, const unsigned int i, const unsigned int x, const unsigned int y,
    const unsigned int sx, const unsigned int sy)
  : distance_(d), index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }

  /**
   * @brief Default Constructor - Should be used sparingly
   */
  // 默认构造：距离设为 double 最大值（表示"尚未确定/无穷远"）
  CellData()
  : distance_(std::numeric_limits<double>::max()), index_(0), x_(0), y_(0), src_x_(0), src_y_(0)
  {
  }

  // 无符号整数的安全求差：避免 unsigned 相减出现负数回绕
  static unsigned absolute_difference(const unsigned x, const unsigned y)
  {
    return (x > y) ? (x - y) : (y - x);
  }

  double distance_;         // 本格到其最近源点的距离（队列的优先级键）
  unsigned int index_;      // 本格在代价地图中的一维索引（与 x_,y_ 冗余）
  unsigned int x_, y_;      // 本格的栅格坐标
  unsigned int src_x_, src_y_;  // 最近源点的栅格坐标（膨胀层据此查源格代价）
};

/**
 * @class CostmaQueue
 * @brief A tool for finding the cells closest to some set of originating cells.
 *
 * A common operation with costmaps is to define a set of cells in the costmap, and then
 * perform some operation on all the other cells based on which cell in the original set
 * the other cells are closest to. This operation is done in the inflation layer to figure out
 * how far each cell is from an obstacle, and is also used in a number of Trajectory cost functions.
 *
 * It is implemented with a queue. The standard operation is to enqueueCell the original set, and then
 * retreive the other cells with the isEmpty/getNextCell iterator-like functionality. getNextCell
 * returns an object that contains the coordinates of this cell and the origin cell, as well as
 * the distance between them. By default, the Euclidean distance is used for ordering, but passing in
 * manhattan=true to the constructor will use the Manhattan distance.
 *
 * The validCellToQueue overridable-function allows for deriving classes to limit the queue traversal
 * to a subset of all costmap cells. LimitedCostmapQueue does this by ignoring distances above a limit.
 *
 */
class CostmapQueue : public MapBasedQueue<CellData>
{
public:
  /**
   * @brief constructor
   * @param costmap Costmap which defines the size/number of cells
   * @param manhattan If true, sort cells by Manhattan distance, otherwise use Euclidean distance
   */
  // 构造：绑定代价地图；manhattan 选择距离度量（默认欧氏）
  explicit CostmapQueue(nav2_costmap_2d::Costmap2D & costmap, bool manhattan = false);

  /**
   * @brief Clear the queue
   */
  // 重置：清空 seen_ 数组、重建距离查找表、调用基类清空队列
  void reset() override;

  /**
   * @brief Add a cell the queue
   * @param x X coordinate of the cell
   * @param y Y coordinate of the cell
   */
  // 源格入队：以自身为源点（src = 自身坐标），距离为 0
  void enqueueCell(unsigned int x, unsigned int y);

  /**
   * @brief Get the next cell to examine, and enqueue its neighbors as needed
   * @return The next cell
   *
   * NB: Assumes that isEmpty has been called before this call and returned false
   */
  // 取出下一个要处理的格子（距离最小者），并把它的四邻域入队
  //（调用前必须先确认 isEmpty() == false）
  CellData getNextCell();

  /**
   * @brief Check to see if we should add this cell to the queue. Always true unless overridden.
   * @param cell The cell to check
   * @return True, unless overriden
   */
  // 钩子函数：派生类重写以限制扩散范围（如 LimitedCostmapQueue 的距离上限）
  virtual bool validCellToQueue(const CellData & /*cell*/) {return true;}
  /**
   * @brief convenience typedef for a pointer
   */
  typedef std::shared_ptr<CostmapQueue> Ptr;  // 智能指针别名，便于外部持有

protected:
  /**
   * @brief Enqueue a cell with the given coordinates and the given source cell
   */
  // 内部入队：给定格子坐标与源点坐标，计算距离后按优先级入队
  void enqueueCell(
    unsigned int index, unsigned int cur_x, unsigned int cur_y, unsigned int src_x,
    unsigned int src_y);

  /**
   * @brief Compute the cached distances
   */
  // 预计算距离查找表：cached_distances_[dx][dy] = hypot(dx,dy) 或 dx+dy
  void computeCache();

  nav2_costmap_2d::Costmap2D & costmap_;  // 宿主代价地图的引用
  std::vector<bool> seen_;                 // 每格是否已入过队（保证每格只处理一次）
  int max_distance_;                       // 距离表覆盖的最大跨度（格数）
  bool manhattan_;                         // true=曼哈顿距离，false=欧氏距离

protected:
  /**
   * @brief  Lookup pre-computed distances
   * @param cur_x The x coordinate of the current cell
   * @param cur_y The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  // 距离查表：以 |dx|、|dy| 为下标在预计算表中取值，O(1)
  inline double distanceLookup(
    const unsigned int cur_x, const unsigned int cur_y,
    const unsigned int src_x, const unsigned int src_y)
  {
    unsigned int dx = CellData::absolute_difference(cur_x, src_x);
    unsigned int dy = CellData::absolute_difference(cur_y, src_y);
    return cached_distances_[dx][dy];
  }
  std::vector<std::vector<double>> cached_distances_;  // 预计算距离表 [dx][dy]
  int cached_max_distance_;                            // 距离表当前对应的最大跨度
};
}  // namespace costmap_queue

#endif  // COSTMAP_QUEUE__COSTMAP_QUEUE_HPP_
