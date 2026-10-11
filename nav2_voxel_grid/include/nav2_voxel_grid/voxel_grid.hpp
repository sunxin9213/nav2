/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
// ============================================================================
// 【文件概况】3D 体素栅格数据结构 VoxelGrid
// ----------------------------------------------------------------------------
// 概述: nav2 中唯一的三维占据表示。X/Y 索引一个 uint32_t 数组（每元素
//       代表一"列"），Z 用位选择——低 16 位表示"未知"、高 16 位表示
//       "已标记障碍"，因此垂直方向最多 16 层，每个体素仅占 2 bit。
// 主要内容:
//   1. VoxelGrid 类:
//      - markVoxel / clearVoxel 系列: 体素级标记与清除（含联动 2D 地图版本）
//      - getVoxel / getVoxelColumn:   体素与整列状态查询（投影 2D 用）
//      - raytraceLine:                3D Bresenham 直线光栅化（模板方法），
//                                     配合 Mark/Clear 仿函数模拟激光射线
//      - bresenham3D:                 整数误差累积的 3D 直线光栅化核心
//   2. 内部仿函数: MarkVoxel / ClearVoxel / ClearVoxelInMap（访问动作），
//      GridOffset / ZOffset（轴步进偏移器）
// 使用场景: nav2_costmap_2d 的 voxel_layer（3D obstacle layer）用它把
//           点云/激光观测以射线形式写入 3D 栅格，再投影为 2D 代价地图，
//           实现按高度过滤障碍（如桌面激光不把地面标成障碍）。
// ============================================================================
#ifndef NAV2_VOXEL_GRID__VOXEL_GRID_HPP_
#define NAV2_VOXEL_GRID__VOXEL_GRID_HPP_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <limits.h>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"

/**
 * @class VoxelGrid
 * @brief A 3D grid structure that stores points as an integer array.
 *        X and Y index the array and Z selects which bit of the integer
 *        is used giving a limit of 16 vertical cells.
 */
namespace nav2_voxel_grid
{

// 体素状态枚举：
// FREE 已知无障碍 | UNKNOWN 未知（未观测/观测被清除）| MARKED 已知有障碍
enum VoxelStatus
{
  FREE = 0,
  UNKNOWN = 1,
  MARKED = 2,
};

// ==================== VoxelGrid 体素栅格 ====================
// 数据结构核心思想（bit 压缩存储）：
// 栅格是一个 size_x * size_y 的 uint32_t 数组，每个 uint32_t 代表一"列"（column），
// 即同一 (x,y) 位置上垂直方向的最多 16 层体素：
//   - 低 16 位（bit 0~15）：第 k 位为 1 表示第 k 层体素"未知"
//   - 高 16 位（bit 16~31）：第 k 位为 1 表示第 k 层体素"已标记障碍"
// 因此每个体素只占 2 个 bit，一对 (unknown,marked) 位组合出三种状态：
//   00 = 已知自由, 01 = 未知, 11 = 已知占据
// markVoxel/clearVoxel 中的 full_mask = (1<<z<<16)|(1<<z) 正是同时操作这对 bit。
class VoxelGrid
{
public:
  /**
   * @brief  Constructor for a voxel grid
   * @param size_x The x size of the grid
   * @param size_y The y size of the grid
   * @param size_z The z size of the grid, only sizes <= 16 are supported
   */
  VoxelGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z);

  ~VoxelGrid();

  /**
   * @brief  Resizes a voxel grid to the desired size
   * @param size_x The x size of the grid
   * @param size_y The y size of the grid
   * @param size_z The z size of the grid, only sizes <= 16 are supported
   */
  void resize(unsigned int size_x, unsigned int size_y, unsigned int size_z);

  void reset();
  // 获取底层数据指针（外部只读访问用）
  uint32_t * getData() {return data_;}

  // 标记单个体素为"已知占据"：full_mask 在高、低 16 位中同时置位第 z 位
  inline void markVoxel(unsigned int x, unsigned int y, unsigned int z)
  {
    // 越界检查：任何一维超出网格范围则直接返回
    if (x >= size_x_ || y >= size_y_ || z >= size_z_) {
      RCLCPP_DEBUG(logger, "Error, voxel out of bounds.\n");
      return;
    }
    // 位掩码：(1<<z<<16) 选中高 16 位的第 z 位，(1<<z) 选中低 16 位的第 z 位
    uint32_t full_mask = ((uint32_t)1 << z << 16) | (1 << z);
    data_[y * size_x_ + x] |= full_mask;  // clear unknown and mark cell
  }

  // 与 markVoxel 类似，但额外统计该列 marked 位数是否达到阈值：
  // 返回 true 表示标记后该列的障碍位数已达到/超过 marked_threshold
  inline bool markVoxelInMap(
    unsigned int x, unsigned int y, unsigned int z,
    unsigned int marked_threshold)
  {
    if (x >= size_x_ || y >= size_y_ || z >= size_z_) {
      RCLCPP_DEBUG(logger, "Error, voxel out of bounds.\n");
      return false;
    }

    int index = y * size_x_ + x;
    uint32_t * col = &data_[index];
    uint32_t full_mask = ((uint32_t)1 << z << 16) | (1 << z);
    *col |= full_mask;  // clear unknown and mark cell

    // 高 16 位即 marked 位图
    unsigned int marked_bits = *col >> 16;

    // make sure the number of bits in each is below our thresholds
    return !bitsBelowThreshold(marked_bits, marked_threshold);
  }

  // 清除单个体素：把该体素的 unknown/marked 两个 bit 都清零 -> 已知自由
  inline void clearVoxel(unsigned int x, unsigned int y, unsigned int z)
  {
    if (x >= size_x_ || y >= size_y_ || z >= size_z_) {
      RCLCPP_DEBUG(logger, "Error, voxel out of bounds.\n");
      return;
    }
    uint32_t full_mask = ((uint32_t)1 << z << 16) | (1 << z);
    // 掩码按位取反后做与运算：同时清掉 unknown 位与 marked 位
    data_[y * size_x_ + x] &= ~(full_mask);  // clear unknown and clear cell
  }

  // 清空整列（按一维索引）：32 位全部清零，该 (x,y) 上所有层变自由
  inline void clearVoxelColumn(unsigned int index)
  {
    assert(index < size_x_ * size_y_);
    data_[index] = 0;
  }

  // 清除体素的同时联动更新 2D 代价地图：
  // 若清除后该列既无未知位也无标记位，则把 costmap 对应格子置 0（自由）
  inline void clearVoxelInMap(unsigned int x, unsigned int y, unsigned int z)
  {
    if (x >= size_x_ || y >= size_y_ || z >= size_z_) {
      RCLCPP_DEBUG(logger, "Error, voxel out of bounds.\n");
      return;
    }
    int index = y * size_x_ + x;
    uint32_t * col = &data_[index];
    uint32_t full_mask = ((uint32_t)1 << z << 16) | (1 << z);
    *col &= ~(full_mask);  // clear unknown and clear cell

    // 异或求出"未知位"图：只有低 16 位置 1 而高 16 位为 0 的位才表示未知
    unsigned int unknown_bits = uint16_t(*col >> 16) ^ uint16_t(*col);
    unsigned int marked_bits = *col >> 16;

    // make sure the number of bits in each is below our thresholds
    // 阈值取 1：即完全无未知、无标记位时才把 2D 格子判为自由
    if (bitsBelowThreshold(unknown_bits, 1) && bitsBelowThreshold(marked_bits, 1)) {
      costmap[index] = 0;
    }
  }

  // 统计 n 中置位个数是否不超过 bit_threshold（提前退出的优化版位计数）
  inline bool bitsBelowThreshold(unsigned int n, unsigned int bit_threshold)
  {
    unsigned int bit_count;
    for (bit_count = 0; n; ) {
      ++bit_count;
      // 超过阈值即可立即返回 false，不必数完所有位
      if (bit_count > bit_threshold) {
        return false;
      }
      n &= n - 1;  // clear the least significant bit set
      // 技巧：n &= n-1 每次消去最低位的 1（Brian Kernighan 算法）
    }
    return true;
  }

  // 经典位计数：返回 n 的置位个数（循环次数恰好等于 1 的个数）
  static inline unsigned int numBits(unsigned int n)
  {
    unsigned int bit_count;
    for (bit_count = 0; n; ++bit_count) {
      n &= n - 1;  // clear the least significant bit set
    }
    return bit_count;
  }

  // 静态版 getVoxel：允许直接对一段外部数据做查询，无需构造对象
  static VoxelStatus getVoxel(
    unsigned int x, unsigned int y, unsigned int z,
    unsigned int size_x, unsigned int size_y, unsigned int size_z, const uint32_t * data)
  {
    if (x >= size_x || y >= size_y || z >= size_z) {
      return UNKNOWN;
    }
    uint32_t full_mask = ((uint32_t)1 << z << 16) | (1 << z);
    // 取出该体素的 unknown/marked 两个 bit
    uint32_t result = data[y * size_x + x] & full_mask;
    unsigned int bits = numBits(result);

    // known marked: 11 = 2 bits, unknown: 01 = 1 bit, known free: 00 = 0 bits
    // 两位都置位 -> MARKED；只有一位 -> UNKNOWN；零位 -> FREE
    if (bits < 2) {
      if (bits < 1) {
        return FREE;
      }
      return UNKNOWN;
    }
    return MARKED;
  }

  // 沿 3D 直线做标记（模拟激光束打到障碍：从传感器原点到命中点全部标记）
  void markVoxelLine(
    double x0, double y0, double z0, double x1, double y1, double z1,
    unsigned int max_length = UINT_MAX);
  // 沿 3D 直线做清除（模拟激光束扫过的自由空间：清除视线上的体素）
  void clearVoxelLine(
    double x0, double y0, double z0, double x1, double y1, double z1,
    unsigned int max_length = UINT_MAX, unsigned int min_length = 0);
  // 清除视线的同时把结果投影到 2D 代价地图（见 ClearVoxelInMap 仿函数）
  void clearVoxelLineInMap(
    double x0, double y0, double z0, double x1, double y1, double z1, unsigned char * map_2d,
    unsigned int unknown_threshold, unsigned int mark_threshold,
    unsigned char free_cost = 0, unsigned char unknown_cost = 255,
    unsigned int max_length = UINT_MAX, unsigned int min_length = 0);

  VoxelStatus getVoxel(unsigned int x, unsigned int y, unsigned int z);

  // Are there any obstacles at that (x, y) location in the grid?
  // 查询某 (x,y) 列的整体状态（用于 3D 体素向 2D 代价地图投影时的判定）：
  // 标记位超阈值 -> MARKED；未知位超阈值 -> UNKNOWN；否则 FREE
  VoxelStatus getVoxelColumn(
    unsigned int x, unsigned int y,
    unsigned int unknown_threshold = 0, unsigned int marked_threshold = 0);

  void printVoxelGrid();
  void printColumnGrid();
  unsigned int sizeX();
  unsigned int sizeY();
  unsigned int sizeZ();

  // ==================== 3D Bresenham 射线追踪 ====================
  // 模板方法：ActionType 是访问动作（MarkVoxel/ClearVoxel/ClearVoxelInMap 仿函数），
  // 沿 (x0,y0,z0)->(x1,y1,z1) 直线逐体素调用 at(offset, z_mask)。
  // max_length 限制射线最大长度（模拟传感器量程），
  // min_length 把射线起点前移一段距离（模拟最小探测距离/机器人自身体积）。
  template<class ActionType>
  inline void raytraceLine(
    ActionType at, double x0, double y0, double z0,
    double x1, double y1, double z1, unsigned int max_length = UINT_MAX,
    unsigned int min_length = 0)
  {
    // we need to chose how much to scale our dominant dimension, based on the
    // maximum length of the line
    // 先计算线段全长
    double dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1) * (z0 - z1));
    // 线段本身比 min_length 还短：整段跳过，不做任何标记/清除
    if ((unsigned int)(dist) < min_length) {
      return;
    }
    double scale, min_x0, min_y0, min_z0;
    if (dist > 0.0) {
      // 射线实际追踪长度不超过 max_length：按比例缩放（最大为 1 即不缩放）
      scale = std::min(1.0, max_length / dist);

      // Updating starting point to the point at distance min_length from the initial point
      // 把起点沿方向前移 min_length 得到截断后的新起点
      min_x0 = x0 + (x1 - x0) / dist * min_length;
      min_y0 = y0 + (y1 - y0) / dist * min_length;
      min_z0 = z0 + (z1 - z0) / dist * min_length;
    } else {
      // dist can be 0 if [x0, y0, z0]==[x1, y1, z1].
      // In this case only this voxel should be processed.
      // 起终点重合的退化情况：只处理这一个体素
      scale = 1.0;
      min_x0 = x0;
      min_y0 = y0;
      min_z0 = z0;
    }

    // 三个轴的整数增量（从截断起点指向终点）
    int dx = int(x1) - int(min_x0);  // NOLINT
    int dy = int(y1) - int(min_y0);  // NOLINT
    int dz = int(z1) - int(min_z0);  // NOLINT

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);
    unsigned int abs_dz = abs(dz);

    // 预计算每步的一维数组偏移量：
    // x 方向 ±1（相邻列），y 方向 ±size_x_（跳一整行），z 方向 ±1（层间）
    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * size_x_;
    int offset_dz = sign(dz);

    // z_mask：当前层对应的一对 bit（高/低 16 位各一位），z 前进时整体移位
    unsigned int z_mask = ((1 << 16) | 1) << (unsigned int)min_z0;
    // 一维数组起始偏移 = y * 行宽 + x
    unsigned int offset = (unsigned int)min_y0 * size_x_ + (unsigned int)min_x0;

    // 用仿函数包装 offset 与 z_mask 的更新方式（见下方 GridOffset/ZOffset）
    GridOffset grid_off(offset);
    ZOffset z_off(z_mask);

    // is x dominant
    // 以下三分支按"主轴"（增量最大的轴）调用 3D Bresenham：
    // x 为主轴：x 每步必走，y/z 按误差累积决定是否走
    if (abs_dx >= max(abs_dy, abs_dz)) {
      int error_y = abs_dx / 2;
      int error_z = abs_dx / 2;

      bresenham3D(
        at, grid_off, grid_off, z_off, abs_dx, abs_dy, abs_dz, error_y, error_z,
        offset_dx, offset_dy, offset_dz, offset, z_mask, (unsigned int)(scale * abs_dx));
      return;
    }

    // y is dominant
    // y 为主轴
    if (abs_dy >= abs_dz) {
      int error_x = abs_dy / 2;
      int error_z = abs_dy / 2;

      bresenham3D(
        at, grid_off, grid_off, z_off, abs_dy, abs_dx, abs_dz, error_x, error_z,
        offset_dy, offset_dx, offset_dz, offset, z_mask, (unsigned int)(scale * abs_dy));
      return;
    }

    // otherwise, z is dominant
    // z 为主轴：注意第一个偏移参数换成 z_off（主轴每步移位 z_mask）
    int error_x = abs_dz / 2;
    int error_y = abs_dz / 2;

    bresenham3D(
      at, z_off, grid_off, grid_off, abs_dz, abs_dx, abs_dy, error_x, error_y, offset_dz,
      offset_dx, offset_dy, offset, z_mask, (unsigned int)(scale * abs_dz));
  }

private:
  // the real work is done here... 3D bresenham implementation
  // 3D Bresenham 核心实现：off_a 是主轴偏移器，off_b/off_c 是两个副轴偏移器。
  // 每步：对当前体素执行动作 at -> 主轴步进 -> 两个副轴误差累积，
  // 误差达到主轴增量时副轴步进一次并回绕误差。经典整数直线光栅化算法。
  template<class ActionType, class OffA, class OffB, class OffC>
  inline void bresenham3D(
    ActionType at, OffA off_a, OffB off_b, OffC off_c,
    unsigned int abs_da, unsigned int abs_db, unsigned int abs_dc,
    int error_b, int error_c, int offset_a, int offset_b, int offset_c, unsigned int & offset,
    unsigned int & z_mask, unsigned int max_length = UINT_MAX)
  {
    // 实际迭代步数取"主轴步数"与"最大长度"的较小者
    unsigned int end = std::min(max_length, abs_da);
    for (unsigned int i = 0; i < end; ++i) {
      at(offset, z_mask);      // 对当前体素执行标记/清除动作
      off_a(offset_a);         // 主轴前进一步
      error_b += abs_db;       // 副轴 b 误差累积
      error_c += abs_dc;       // 副轴 c 误差累积
      if ((unsigned int)error_b >= abs_da) {
        off_b(offset_b);       // b 轴误差溢出：b 轴步进并回绕误差
        error_b -= abs_da;
      }
      if ((unsigned int)error_c >= abs_da) {
        off_c(offset_c);       // c 轴误差溢出：c 轴步进并回绕误差
        error_c -= abs_da;
      }
    }
    at(offset, z_mask);        // 循环结束后处理最后一个体素
  }

  // 符号函数：正数返回 1，非正返回 -1
  inline int sign(int i)
  {
    return i > 0 ? 1 : -1;
  }

  inline unsigned int max(unsigned int x, unsigned int y)
  {
    return x > y ? x : y;
  }

  unsigned int size_x_, size_y_, size_z_;
  uint32_t * data_;                 // 核心数据：size_x*size_y 个 uint32_t，每个元素一列
  unsigned char * costmap;          // 联动的 2D 代价地图指针（可为空）
  rclcpp::Logger logger;

  // Aren't functors so much fun... used to recreate the Bresenham macro Eric
  // wrote in the original version, but in "proper" c++
  // ===== 以下仿函数（functor）用于参数化 Bresenham 遍历中的"访问"动作 =====

  // 标记动作：把 z_mask 对应的 bit 置 1（标记障碍）
  class MarkVoxel
  {
public:
    explicit MarkVoxel(uint32_t * data)
    : data_(data) {}
    inline void operator()(unsigned int offset, unsigned int z_mask)
    {
      data_[offset] |= z_mask;  // clear unknown and mark cell
    }

private:
    uint32_t * data_;
  };

  // 清除动作：把 z_mask 对应的 bit 清零（清除为已知自由）
  class ClearVoxel
  {
public:
    explicit ClearVoxel(uint32_t * data)
    : data_(data) {}
    inline void operator()(unsigned int offset, unsigned int z_mask)
    {
      data_[offset] &= ~(z_mask);  // clear unknown and clear cell
    }

private:
    uint32_t * data_;
  };

  // 清除 + 同步更新 2D 代价地图的动作（3D -> 2D 投影）：
  // 清除体素后根据该列剩余的 unknown/marked 位数决定 2D 格子的代价值
  class ClearVoxelInMap
  {
public:
    ClearVoxelInMap(
      uint32_t * data, unsigned char * costmap,
      unsigned int unknown_clear_threshold, unsigned int marked_clear_threshold,
      unsigned char free_cost = 0, unsigned char unknown_cost = 255)
    : data_(data), costmap_(costmap),
      unknown_clear_threshold_(unknown_clear_threshold), marked_clear_threshold_(
        marked_clear_threshold),
      free_cost_(free_cost), unknown_cost_(unknown_cost)
    {
    }

    inline void operator()(unsigned int offset, unsigned int z_mask)
    {
      uint32_t * col = &data_[offset];
      *col &= ~(z_mask);  // clear unknown and clear cell

      // 统计该列剩余的未知位数与标记位数
      unsigned int unknown_bits = uint16_t(*col >> 16) ^ uint16_t(*col);
      unsigned int marked_bits = *col >> 16;

      // make sure the number of bits in each is below our thresholds
      // 标记位数足够少（无障碍证据）时才更新 2D 地图：
      // 未知位也少 -> 置为自由代价；否则置为未知代价
      if (bitsBelowThreshold(marked_bits, marked_clear_threshold_)) {
        if (bitsBelowThreshold(unknown_bits, unknown_clear_threshold_)) {
          costmap_[offset] = free_cost_;
        } else {
          costmap_[offset] = unknown_cost_;
        }
      }
    }

private:
    inline bool bitsBelowThreshold(unsigned int n, unsigned int bit_threshold)
    {
      unsigned int bit_count;
      for (bit_count = 0; n; ) {
        ++bit_count;
        if (bit_count > bit_threshold) {
          return false;
        }
        n &= n - 1;  // clear the least significant bit set
      }
      return true;
    }

    uint32_t * data_;
    unsigned char * costmap_;
    unsigned int unknown_clear_threshold_, marked_clear_threshold_;
    unsigned char free_cost_, unknown_cost_;
  };

  // 平面内偏移器：让一维数组索引 offset 增/减（x 或 y 方向步进）
  class GridOffset
  {
public:
    explicit GridOffset(unsigned int & offset)
    : offset_(offset) {}
    inline void operator()(int offset_val)
    {
      offset_ += offset_val;
    }

private:
    unsigned int & offset_;
  };

  // z 轴偏移器：z 层前进时把 z_mask 整体左移/右移一位（切换到相邻层的 bit 对）
  class ZOffset
  {
public:
    explicit ZOffset(unsigned int & z_mask)
    : z_mask_(z_mask) {}
    inline void operator()(int offset_val)
    {
      offset_val > 0 ? z_mask_ <<= 1 : z_mask_ >>= 1;
    }

private:
    unsigned int & z_mask_;
  };
};

}  // namespace nav2_voxel_grid

#endif  // NAV2_VOXEL_GRID__VOXEL_GRID_HPP_
