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
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
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
// ============================================================================
// 中文注释：CostmapLayer 中间层基类实现文件
//
// 【文件功能】
// 本文件实现 CostmapLayer——Layer 与具体地图层（Obstacle/Static/Voxel）
// 之间的中间基类。它同时继承 Layer（插件接口）与 Costmap2D（数据存储），
// 即"本层自带一张与主地图同尺寸的缓存地图 costmap_"。
// 本文件主要提供：
//   1. 边界工具：touch（把点扩进更新边界）、addExtraBounds/useExtraBounds
//      （外部请求的额外重绘区域）；
//   2. matchSize()：本层缓存与主地图尺寸对齐；
//   3. clearArea()：把指定矩形区域置为 NO_INFORMATION；
//   4. 四种"本层缓存 → 主地图"的合并策略：
//      updateWithMax（取最大值，最常用/最安全）、
//      updateWithTrueOverwrite（无条件覆盖，静态层用）、
//      updateWithOverwrite（非未知值覆盖）、
//      updateWithAddition（相加并饱和，调试用）。
//
// 【在 Nav2 中的位置】
// 继承链：Layer → CostmapLayer → StaticLayer / ObstacleLayer / VoxelLayer
// （InflationLayer 与 CostmapFilter 直接继承 Layer，因为它们没有独立缓存）。
//
// 【内存布局提示】
// 主地图与本层缓存都是一维 unsigned char 数组，行优先，
// index = i + j * span（span = size_x），两图同几何参数时下标一一对应。
// ============================================================================

#include <nav2_costmap_2d/costmap_layer.hpp>
#include <stdexcept>
#include <algorithm>

namespace nav2_costmap_2d
{

// ============================================================================
// touch()：把一个世界坐标点扩展进更新边界（min/max 取极值）
// 每当本层修改了某个格子，就 touch 该点，保证 updateCosts 覆盖到它
// ============================================================================

void CostmapLayer::touch(
  double x, double y, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  *min_x = std::min(x, *min_x);
  *min_y = std::min(y, *min_y);
  *max_x = std::max(x, *max_x);
  *max_y = std::max(y, *max_y);
}

// ============================================================================
// matchSize()：把本层缓存地图调整为与主地图相同的尺寸/分辨率/原点
// （注意：会丢弃本层缓存中已有的数据）
// ============================================================================

void CostmapLayer::matchSize()
{
  Costmap2D * master = layered_costmap_->getCostmap();
  resizeMap(
    master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
    master->getOriginX(), master->getOriginY());
}

// ============================================================================
// clearArea()：把指定矩形区域内（或区域外，invert=true 时）的格子
// 全部置为 NO_INFORMATION——clear costmap 服务"恢复未知"的实现基础
// invert=false：清除矩形内部；invert=true：清除矩形外部（保留内部）
// ============================================================================

void CostmapLayer::clearArea(int start_x, int start_y, int end_x, int end_y, bool invert)
{
  current_ = false;   // 数据被改动，标记不最新
  unsigned char * grid = getCharMap();
  // 遍历整张地图
  for (int x = 0; x < static_cast<int>(getSizeInCellsX()); x++) {
    bool xrange = x > start_x && x < end_x;   // x 是否在目标矩形内

    for (int y = 0; y < static_cast<int>(getSizeInCellsY()); y++) {
      // (在矩形内) == invert 时跳过：
      //   invert=false：矩形内的格子要清除，跳过矩形外
      //   invert=true ：矩形外要清除，跳过矩形内
      if ((xrange && y > start_y && y < end_y) == invert) {
        continue;
      }
      // 一维下标 index = x + y * size_x
      int index = getIndex(x, y);
      if (grid[index] != NO_INFORMATION) {
        grid[index] = NO_INFORMATION;   // 置为未知
      }
    }
  }
}

// ============================================================================
// addExtraBounds()：外部（如 clear costmap 服务）请求本层额外重绘的区域
// 记录到 extra_* 成员并置 has_extra_bounds_ 标志
// ============================================================================

void CostmapLayer::addExtraBounds(double mx0, double my0, double mx1, double my1)
{
  extra_min_x_ = std::min(mx0, extra_min_x_);
  extra_max_x_ = std::max(mx1, extra_max_x_);
  extra_min_y_ = std::min(my0, extra_min_y_);
  extra_max_y_ = std::max(my1, extra_max_y_);
  has_extra_bounds_ = true;
}

// ============================================================================
// useExtraBounds()：把额外边界合并进本轮更新边界（一次性消费）
// 合并后复位 extra_* 为极值并清除标志
// ============================================================================

void CostmapLayer::useExtraBounds(double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!has_extra_bounds_) {
    return;
  }

  // 额外区域与当前边界取并集
  *min_x = std::min(extra_min_x_, *min_x);
  *min_y = std::min(extra_min_y_, *min_y);
  *max_x = std::max(extra_max_x_, *max_x);
  *max_y = std::max(extra_max_y_, *max_y);
  // 复位为极值，等待下次 addExtraBounds
  extra_min_x_ = 1e6;
  extra_min_y_ = 1e6;
  extra_max_x_ = -1e6;
  extra_max_y_ = -1e6;
  has_extra_bounds_ = false;
}

// ============================================================================
// updateWithMax()：合并策略一（最常用）——逐格取本层与主地图的较大值
// 语义："只增不减"：保留更危险的信息，多个感知层叠加时更安全。
// 本层为 NO_INFORMATION 的格子跳过（未知不覆盖任何信息）。
// ============================================================================

void CostmapLayer::updateWithMax(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  // 主地图一维数组与行跨度
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  // 逐行扫描更新窗口：it = i + j*span 的一维游标
  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++) {
      // 本层未知：不改动主地图
      if (costmap_[it] == NO_INFORMATION) {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      // 主地图未知 或 本层值更大 → 写入本层值
      if (old_cost == NO_INFORMATION || old_cost < costmap_[it]) {
        master_array[it] = costmap_[it];
      }
      it++;
    }
  }
}

// ============================================================================
// updateWithTrueOverwrite()：合并策略二——无条件整块覆盖
// 语义："本层是权威数据源"：静态层使用（静态地图优先于一切）。
// 注意与 updateWithOverwrite 的区别：本函数连 NO_INFORMATION 也覆盖。
// ============================================================================

void CostmapLayer::updateWithTrueOverwrite(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  // 本层缓存未初始化：抛异常（调用方逻辑错误）
  if (costmap_ == nullptr) {
    throw std::runtime_error("Can't update costmap layer: It has't been initialized yet!");
  }

  unsigned char * master = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  // 两图几何参数一致，一维下标一一对应，直接逐格赋值
  for (int j = min_j; j < max_j; j++) {
    unsigned int it = span * j + min_i;
    for (int i = min_i; i < max_i; i++) {
      master[it] = costmap_[it];
      it++;
    }
  }
}

// ============================================================================
// updateWithOverwrite()：合并策略三——仅覆盖非未知值
// 与 TrueOverwrite 的区别：本层 NO_INFORMATION 的格子不覆盖主地图
// ============================================================================

void CostmapLayer::updateWithOverwrite(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  unsigned char * master = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = span * j + min_i;
    for (int i = min_i; i < max_i; i++) {
      // 本层非未知才覆盖（保留主地图未知区域）
      if (costmap_[it] != NO_INFORMATION) {
        master[it] = costmap_[it];
      }
      it++;
    }
  }
}

// ============================================================================
// updateWithAddition()：合并策略四——逐格相加（饱和处理）
// 两层置信度叠加的语义；总和封顶在 INSCRIBED_INFLATED_OBSTACLE-1(252)，
// 避免溢出到 253+ 的特殊语义值（内切/致命/未知）
// ============================================================================

void CostmapLayer::updateWithAddition(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++) {
      // 本层未知：跳过
      if (costmap_[it] == NO_INFORMATION) {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost == NO_INFORMATION) {
        // 主地图未知：直接用本层值
        master_array[it] = costmap_[it];
      } else {
        // 两值相加，饱和到 252（不超过特殊代价语义区间）
        int sum = old_cost + costmap_[it];
        if (sum >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          master_array[it] = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1;
        } else {
          master_array[it] = sum;
        }
      }
      it++;
    }
  }
}
}  // namespace nav2_costmap_2d
