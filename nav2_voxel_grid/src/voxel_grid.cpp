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
// 【文件概况】VoxelGrid 类的实现文件
// ----------------------------------------------------------------------------
// 概述: 实现 3D 体素栅格的构造/析构/重置/尺寸管理、三条射线操作
//       （markVoxelLine / clearVoxelLine / clearVoxelLineInMap，内部
//       均委托给头文件中的 raytraceLine 3D Bresenham 模板），
//       以及体素/整列状态查询与调试打印。射线操作是本文件核心：
//       一次调用即完成"传感器原点 → 命中点"整条视线的数据更新。
// 主要内容:
//   1. 构造/resize/reset: 内存分配与"全未知"初始化（z 上限 16 层）
//   2. markVoxelLine:      用 MarkVoxel 仿函数沿直线标记障碍
//   3. clearVoxelLine:     用 ClearVoxel 仿函数沿直线清除自由空间
//   4. clearVoxelLineInMap: 清除 + 按阈值把整列状态投影写入 2D 代价地图
//   5. getVoxel / getVoxelColumn: 位解码查询单个体素/整列聚合状态
// 使用场景: 由 nav2_costmap_2d 的 voxel_layer 调用——每收到一帧
//           点云，对每个观测点执行 clear（视线）+ mark（命中点），
//           并把结果投影到 2D 代价地图供规划/控制使用。
// ============================================================================
#include <nav2_voxel_grid/voxel_grid.hpp>

namespace nav2_voxel_grid
{
// 本文件实现 VoxelGrid 的构造/重置/尺寸管理与查询接口。
// 数据布局见头文件注释：每个 uint32_t 存一列（最多 16 层），
// 低 16 位为 unknown 标志，高 16 位为 marked 标志。

VoxelGrid::VoxelGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z)
: logger(rclcpp::get_logger("voxel_grid"))
{
  size_x_ = size_x;
  size_y_ = size_y;
  size_z_ = size_z;

  // 位压缩存储只支持 16 层：超过则截断为 16 并提示
  if (size_z_ > 16) {
    RCLCPP_INFO(
      logger, "Error, this implementation can only support up to 16 z values (%d)",
      size_z_);
    size_z_ = 16;
  }

  // 分配 size_x*size_y 个 uint32_t（每格一列），并全部初始化为"全未知"
  data_ = new uint32_t[size_x_ * size_y_];
  // unknown_col = 0x0000FFFF：低 16 位全 1（每层都未知），高 16 位全 0（无障碍）
  uint32_t unknown_col = ~((uint32_t)0) >> 16;
  uint32_t * col = data_;
  for (unsigned int i = 0; i < size_x_ * size_y_; ++i) {
    *col = unknown_col;
    ++col;
  }
}

void VoxelGrid::resize(unsigned int size_x, unsigned int size_y, unsigned int size_z)
{
  // if we're not actually changing the size, we can just reset things
  // 尺寸没变时无需重新分配内存，直接复位即可
  if (size_x == size_x_ && size_y == size_y_ && size_z == size_z_) {
    reset();
    return;
  }

  // 尺寸变化：释放旧内存，按新尺寸重新分配并初始化为全未知
  delete[] data_;
  size_x_ = size_x;
  size_y_ = size_y;
  size_z_ = size_z;

  if (size_z_ > 16) {
    RCLCPP_INFO(
      logger, "Error, this implementation can only support up to 16 z values (%d)",
      size_z);
    size_z_ = 16;
  }

  data_ = new uint32_t[size_x_ * size_y_];
  uint32_t unknown_col = ~((uint32_t)0) >> 16;
  uint32_t * col = data_;
  for (unsigned int i = 0; i < size_x_ * size_y_; ++i) {
    *col = unknown_col;
    ++col;
  }
}

VoxelGrid::~VoxelGrid()
{
  // 裸数组用 delete[] 释放（与 new[] 配对）
  delete[] data_;
}

void VoxelGrid::reset()
{
  // 复位：所有列恢复为"全未知"（低 16 位全 1），不重新分配内存
  uint32_t unknown_col = ~((uint32_t)0) >> 16;
  uint32_t * col = data_;
  for (unsigned int i = 0; i < size_x_ * size_y_; ++i) {
    *col = unknown_col;
    ++col;
  }
}

// 沿 3D 直线标记体素（模拟传感器射线命中障碍点）
void VoxelGrid::markVoxelLine(
  double x0, double y0, double z0, double x1, double y1, double z1,
  unsigned int max_length)
{
  // 起点或终点任一越界则放弃本次射线（Bresenham 无法处理）
  if (x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1 >= size_x_ || y1 >= size_y_ ||
    z1 >= size_z_)
  {
    RCLCPP_DEBUG(
      logger,
      "Error, line endpoint out of bounds. "
      "(%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)",
      x0, y0, z0, x1, y1, z1, size_x_, size_y_, size_z_);
    return;
  }

  // MarkVoxel 仿函数 + 射线追踪：途经体素全部标记
  MarkVoxel mv(data_);
  raytraceLine(mv, x0, y0, z0, x1, y1, z1, max_length);
}

// 沿 3D 直线清除体素（模拟传感器视线扫过的自由空间）
void VoxelGrid::clearVoxelLine(
  double x0, double y0, double z0, double x1, double y1, double z1,
  unsigned int max_length, unsigned int min_length)
{
  if (x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1 >= size_x_ || y1 >= size_y_ ||
    z1 >= size_z_)
  {
    RCLCPP_DEBUG(
      logger,
      "Error, line endpoint out of bounds. "
      "(%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)",
      x0, y0, z0, x1, y1, z1, size_x_, size_y_, size_z_);
    return;
  }

  // ClearVoxel 仿函数 + 射线追踪：途经体素全部清除为自由
  ClearVoxel cv(data_);
  raytraceLine(cv, x0, y0, z0, x1, y1, z1, max_length, min_length);
}

// 清除视线的同时把 3D 结果投影写入 2D 代价地图
void VoxelGrid::clearVoxelLineInMap(
  double x0, double y0, double z0, double x1, double y1, double z1, unsigned char * map_2d,
  unsigned int unknown_threshold, unsigned int mark_threshold, unsigned char free_cost,
  unsigned char unknown_cost, unsigned int max_length, unsigned int min_length)
{
  // 记录 2D 地图指针供 ClearVoxelInMap 仿函数使用
  costmap = map_2d;
  // 无 2D 地图时退化为普通清除
  if (map_2d == NULL) {
    clearVoxelLine(x0, y0, z0, x1, y1, z1, max_length, min_length);
    return;
  }

  if (x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1 >= size_x_ || y1 >= size_y_ ||
    z1 >= size_z_)
  {
    RCLCPP_DEBUG(
      logger,
      "Error, line endpoint out of bounds. "
      "(%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)",
      x0, y0, z0, x1, y1, z1, size_x_, size_y_, size_z_);
    return;
  }

  // ClearVoxelInMap：清除体素并按阈值把该列状态写为 2D 自由/未知代价
  ClearVoxelInMap cvm(data_, costmap, unknown_threshold, mark_threshold, free_cost, unknown_cost);
  raytraceLine(cvm, x0, y0, z0, x1, y1, z1, max_length, min_length);
}

// 查询单个体素状态（详见头文件中的位编码说明）
VoxelStatus VoxelGrid::getVoxel(unsigned int x, unsigned int y, unsigned int z)
{
  if (x >= size_x_ || y >= size_y_ || z >= size_z_) {
    RCLCPP_DEBUG(logger, "Error, voxel out of bounds. (%d, %d, %d)\n", x, y, z);
    return UNKNOWN;
  }
  // 同时取出该体素的 unknown 位（低 16 位）与 marked 位（高 16 位）
  uint32_t full_mask = ((uint32_t)1 << z << 16) | (1 << z);
  uint32_t result = data_[y * size_x_ + x] & full_mask;
  unsigned int bits = numBits(result);

  // known marked: 11 = 2 bits, unknown: 01 = 1 bit, known free: 00 = 0 bits
  // 两个位都为 1 -> MARKED；仅一个为 1 -> UNKNOWN；全 0 -> FREE
  if (bits < 2) {
    if (bits < 1) {
      return FREE;
    }

    return UNKNOWN;
  }

  return MARKED;
}

// 查询 (x,y) 位置整列的聚合状态（用于 2D 投影判定）
VoxelStatus VoxelGrid::getVoxelColumn(
  unsigned int x, unsigned int y,
  unsigned int unknown_threshold, unsigned int marked_threshold)
{
  if (x >= size_x_ || y >= size_y_) {
    RCLCPP_DEBUG(logger, "Error, voxel out of bounds. (%d, %d)\n", x, y);
    return UNKNOWN;
  }

  uint32_t * col = &data_[y * size_x_ + x];

  // 未知位图：高低 16 位异或，得到"仅低位置 1"的位（即未知层）
  unsigned int unknown_bits = uint16_t(*col >> 16) ^ uint16_t(*col);
  // 标记位图：直接取高 16 位
  unsigned int marked_bits = *col >> 16;

  // check if the number of marked bits qualifies the col as marked
  // 标记位数超过阈值 -> 整列判为 MARKED（该 2D 格子视为障碍）
  if (!bitsBelowThreshold(marked_bits, marked_threshold)) {
    return MARKED;
  }

  // check if the number of unkown bits qualifies the col as unknown
  // 未知位数超过阈值 -> 整列判为 UNKNOWN
  if (!bitsBelowThreshold(unknown_bits, unknown_threshold)) {
    return UNKNOWN;
  }

  // 其余情况：已知自由
  return FREE;
}

unsigned int VoxelGrid::sizeX()
{
  return size_x_;
}

unsigned int VoxelGrid::sizeY()
{
  return size_y_;
}

unsigned int VoxelGrid::sizeZ()
{
  return size_z_;
}

// 调试打印：逐层输出整个体素栅格（'#' 表示 MARKED）
void VoxelGrid::printVoxelGrid()
{
  for (unsigned int z = 0; z < size_z_; z++) {
    printf("Layer z = %u:\n", z);
    for (unsigned int y = 0; y < size_y_; y++) {
      for (unsigned int x = 0; x < size_x_; x++) {
        printf((getVoxel(x, y, z)) == nav2_voxel_grid::MARKED ? "#" : " ");
      }
      printf("|\n");
    }
  }
}

// 调试打印：输出列视图（每 (x,y) 一格，按 getVoxelColumn 聚合判定）
void VoxelGrid::printColumnGrid()
{
  printf("Column view:\n");
  for (unsigned int y = 0; y < size_y_; y++) {
    for (unsigned int x = 0; x < size_x_; x++) {
      printf((getVoxelColumn(x, y, 16, 0) == nav2_voxel_grid::MARKED) ? "#" : " ");
    }
    printf("|\n");
  }
}
}  // namespace nav2_voxel_grid
