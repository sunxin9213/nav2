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
// 中文注释：Costmap2D 核心类实现文件（代价地图的"底层数据结构"）
//
// 【文件功能】
// 本文件实现 nav2_costmap_2d 包最核心的类 Costmap2D。它是一张纯数据
// 结构型的 2D 栅格代价地图，不包含任何 ROS 话题/传感器逻辑——
// 所有 Layer（静态/障碍/膨胀等）与 LayeredCostmap 都基于它存储数据。
//
// 【核心概念：代价地图的内存与坐标模型】
// 1. 内存布局：一维 unsigned char 数组 costmap_，长度 size_x_*size_y_，
//    行优先（row-major）存储。栅格 (mx,my) 的一维下标：
//        index = mx + my * size_x_
// 2. 几何参数：
//    - resolution_：分辨率（米/格），每格边长；
//    - (origin_x_, origin_y_)：地图左下角格的世界坐标（米）；
//    - size_x_/size_y_：x/y 方向格数。
// 3. 世界坐标 ↔ 地图坐标（栅格中心约定）：
//    - mapToWorld：wx = origin_x + (mx + 0.5) * resolution
//                  （+0.5 取格中心，避免点落在格边界上）
//    - worldToMap：mx = (wx - origin_x) / resolution（截断取整），
//      超出地图范围返回 false。
// 4. 代价定义（见 cost_values.hpp）：
//    FREE_SPACE=0（自由）、LETHAL_OBSTACLE=254（致命障碍）、
//    NO_INFORMATION=255（未知）、INSCRIBED_INFLATED_OBSTACLE=253。
// 5. 线程安全：access_ 互斥锁保护 costmap_ 的分配/释放/复位等
//    结构性修改（读写热点路径通常依赖上层锁）。
// 6. 滚动窗口（rolling window）：updateOrigin() 平移地图原点，
//    保留新旧窗口重叠区的数据，实现"跟随机器人滑动的局部地图"。
// 7. 栅格几何算法：raytraceLine（Bresenham 直线）、
//    convexFillCells（凸多边形填充）等用于射线清除与足迹清除。
// ============================================================================
#include "nav2_costmap_2d/costmap_2d.hpp"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/occ_grid_values.hpp"

namespace nav2_costmap_2d
{
// ============================================================================
// 构造函数（按尺寸创建）：分配指定尺寸的代价数组并复位为默认值
// 参数：cells_size_x/y=格数、resolution=分辨率、origin=左下角世界坐标、
//       default_value=默认代价（如 NO_INFORMATION 或 FREE_SPACE）
// ============================================================================
Costmap2D::Costmap2D(
  unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
  double origin_x, double origin_y, unsigned char default_value)
: size_x_(cells_size_x), size_y_(cells_size_y), resolution_(resolution), origin_x_(origin_x),
  origin_y_(origin_y), costmap_(NULL), default_value_(default_value)
{
  // 创建读写锁，保护地图数据的结构性修改
  access_ = new mutex_t();

  // create the costmap
  initMaps(size_x_, size_y_);   // 分配 unsigned char 数组
  resetMaps();                  // 全部置为默认值
}

// ============================================================================
// 构造函数（从 OccupancyGrid 消息转换）：
// 把 SLAM/地图服务器发布的占用栅格 [-1,100] 转换为代价 [0,255]
// ============================================================================
Costmap2D::Costmap2D(const nav_msgs::msg::OccupancyGrid & map)
: default_value_(FREE_SPACE)
{
  access_ = new mutex_t();

  // fill local variables
  // 从消息头提取几何信息：尺寸、分辨率、原点
  size_x_ = map.info.width;
  size_y_ = map.info.height;
  resolution_ = map.info.resolution;
  origin_x_ = map.info.origin.position.x;
  origin_y_ = map.info.origin.position.y;

  // create the costmap
  // 分配代价数组（一维，长度 size_x*size_y）
  costmap_ = new unsigned char[size_x_ * size_y_];

  // fill the costmap with a data
  // 逐格转换：int8 [-1,100] → unsigned char [0,255]
  int8_t data;
  for (unsigned int it = 0; it < size_x_ * size_y_; it++) {
    data = map.data[it];
    if (data == nav2_util::OCC_GRID_UNKNOWN) {
      // -1（未知）→ NO_INFORMATION(255)
      costmap_[it] = NO_INFORMATION;
    } else {
      // Linear conversion from OccupancyGrid data range [OCC_GRID_FREE..OCC_GRID_OCCUPIED]
      // to costmap data range [FREE_SPACE..LETHAL_OBSTACLE]
      // [0,100] 线性映射到 [0,254]：cost = data * 254/100
      costmap_[it] = std::round(
        static_cast<double>(data) * (LETHAL_OBSTACLE - FREE_SPACE) /
        (nav2_util::OCC_GRID_OCCUPIED - nav2_util::OCC_GRID_FREE));
    }
  }
}

// deleteMaps()：释放代价数组（加锁，置空指针）
void Costmap2D::deleteMaps()
{
  // clean up data
  std::unique_lock<mutex_t> lock(*access_);
  delete[] costmap_;
  costmap_ = NULL;
}

// initMaps()：按新尺寸重新分配代价数组（旧数据被丢弃）
void Costmap2D::initMaps(unsigned int size_x, unsigned int size_y)
{
  std::unique_lock<mutex_t> lock(*access_);
  delete[] costmap_;
  costmap_ = new unsigned char[size_x * size_y];
}

// ============================================================================
// resizeMap()：调整地图尺寸/分辨率/原点（旧数据不保留，全部复位）
// ============================================================================
void Costmap2D::resizeMap(
  unsigned int size_x, unsigned int size_y, double resolution,
  double origin_x, double origin_y)
{
  size_x_ = size_x;
  size_y_ = size_y;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;

  initMaps(size_x, size_y);

  // reset our maps to have no information
  // 全图复位为默认值（视为无信息）
  resetMaps();
}

// resetMaps()：整张地图复位为默认值（memset 快速清零/填充）
void Costmap2D::resetMaps()
{
  std::unique_lock<mutex_t> lock(*access_);
  memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

// resetMap()：复位一个矩形区域（x0,y0)-(xn,yn) 为默认值
void Costmap2D::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
  resetMapToValue(x0, y0, xn, yn, default_value_);
}

// ============================================================================
// resetMapToValue()：把矩形区域 [x0,xn)×[y0,yn) 置为指定值
// 按"行"memset：每行首元素的一维下标为 y*size_x + x0
// ============================================================================
void Costmap2D::resetMapToValue(
  unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn, unsigned char value)
{
  std::unique_lock<mutex_t> lock(*(access_));
  unsigned int len = xn - x0;   // 每行长度（格数）
  // y 从 y0 行到 yn 行，步进 size_x_（跳到下一行的同列）
  for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_) {
    memset(costmap_ + y, value, len * sizeof(unsigned char));
  }
}

// ============================================================================
// copyCostmapWindow()：把另一张地图的一个世界坐标窗口复制为本地图
// （本地图被重建为该窗口大小；用于从全局地图裁剪局部窗口）
// ============================================================================
bool Costmap2D::copyCostmapWindow(
  const Costmap2D & map, double win_origin_x, double win_origin_y,
  double win_size_x,
  double win_size_y)
{
  // check for self windowing
  // 不允许把自身窗口化为自身的窗口
  if (this == &map) {
    // ROS_ERROR("Cannot convert this costmap into a window of itself");
    return false;
  }

  // clean up old data
  // 释放旧数组
  deleteMaps();

  // compute the bounds of our new map
  // 窗口两角（左下/右上）的世界坐标 → 源地图栅格坐标，越界则失败
  unsigned int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  if (!map.worldToMap(win_origin_x, win_origin_y, lower_left_x, lower_left_y) ||
    !map.worldToMap(
      win_origin_x + win_size_x, win_origin_y + win_size_y, upper_right_x,
      upper_right_y))
  {
    // ROS_ERROR("Cannot window a map that the window bounds don't fit inside of");
    return false;
  }

  // 新地图几何参数：窗口尺寸/源分辨率/窗口原点
  size_x_ = upper_right_x - lower_left_x;
  size_y_ = upper_right_y - lower_left_y;
  resolution_ = map.resolution_;
  origin_x_ = win_origin_x;
  origin_y_ = win_origin_y;

  // initialize our various maps and reset markers for inflation
  initMaps(size_x_, size_y_);

  // copy the window of the static map and the costmap that we're taking
  // 逐行拷贝源地图窗口数据到新数组
  copyMapRegion(
    map.costmap_, lower_left_x, lower_left_y, map.size_x_, costmap_, 0, 0, size_x_,
    size_x_,
    size_y_);
  return true;
}

// ============================================================================
// copyWindow()：栅格坐标级的区域拷贝（源矩形 → 本地图指定偏移处）
// 用于地图原点平移等需要搬移数据的场合（越界检查后逐行复制）
// ============================================================================
bool Costmap2D::copyWindow(
  const Costmap2D & source,
  unsigned int sx0, unsigned int sy0, unsigned int sxn, unsigned int syn,
  unsigned int dx0, unsigned int dy0)
{
  // 源窗口尺寸
  const unsigned int sz_x = sxn - sx0;
  const unsigned int sz_y = syn - sy0;

  // 源窗口越界检查
  if (sxn > source.getSizeInCellsX() || syn > source.getSizeInCellsY()) {
    return false;
  }

  // 目标区域越界检查
  if (dx0 + sz_x > size_x_ || dy0 + sz_y > size_y_) {
    return false;
  }

  // 执行逐行拷贝（copyMapRegion 处理行步长差异）
  copyMapRegion(
    source.costmap_, sx0, sy0, source.size_x_,
    costmap_, dx0, dy0, size_x_,
    sz_x, sz_y);
  return true;
}

// ============================================================================
// 赋值运算符：深拷贝另一张地图（先释放旧数组，再分配并 memcpy）
// ============================================================================
Costmap2D & Costmap2D::operator=(const Costmap2D & map)
{
  // check for self assignement
  // 自赋值保护
  if (this == &map) {
    return *this;
  }

  // clean up old data
  deleteMaps();

  // 复制几何参数
  size_x_ = map.size_x_;
  size_y_ = map.size_y_;
  resolution_ = map.resolution_;
  origin_x_ = map.origin_x_;
  origin_y_ = map.origin_y_;

  // initialize our various maps
  initMaps(size_x_, size_y_);

  // copy the cost map
  // 两个数组布局相同，直接整块内存拷贝
  memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));

  return *this;
}

// 拷贝构造：创建新锁后复用赋值运算符（注意锁不共享，各自独立）
Costmap2D::Costmap2D(const Costmap2D & map)
: costmap_(NULL)
{
  access_ = new mutex_t();
  *this = map;
}

// just initialize everything to NULL by default
// 默认构造：全零尺寸、空指针（等待后续 resizeMap/initMaps）
Costmap2D::Costmap2D()
: size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), costmap_(NULL)
{
  access_ = new mutex_t();
}

// 析构：释放数组与互斥锁
Costmap2D::~Costmap2D()
{
  deleteMaps();
  delete access_;
}

// ============================================================================
// cellDistance()：世界距离（米）→ 栅格距离（格数），向上取整且非负
// 例如膨胀半径 0.55m / 分辨率 0.05m = 11 格
// ============================================================================
unsigned int Costmap2D::cellDistance(double world_dist)
{
  double cells_dist = std::max(0.0, ceil(world_dist / resolution_));
  return (unsigned int)cells_dist;
}

// getCharMap()：获取底层一维数组指针（unsigned char*，长度 size_x*size_y）
unsigned char * Costmap2D::getCharMap() const
{
  return costmap_;
}

// getCost(mx,my)：读取栅格代价——一维下标换算 index = mx + my*size_x
unsigned char Costmap2D::getCost(unsigned int mx, unsigned int my) const
{
  return costmap_[getIndex(mx, my)];
}

// getCost(undex)：直接按一维下标读取
unsigned char Costmap2D::getCost(unsigned int undex) const
{
  return costmap_[undex];
}

// setCost(mx,my,cost)：写入栅格代价
void Costmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
  costmap_[getIndex(mx, my)] = cost;
}

// ============================================================================
// mapToWorld()：栅格坐标 → 世界坐标（格中心约定）
// wx = origin_x + (mx + 0.5) * resolution：+0.5 表示取格子中心，
// 避免坐标恰好落在格边界产生歧义
// ============================================================================
void Costmap2D::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

// ============================================================================
// worldToMap()：世界坐标 → 栅格坐标（带边界检查，越界返回 false）
// mx = (wx - origin_x) / resolution（C++ 截断取整等价于向下取整）
// ============================================================================
bool Costmap2D::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  // 在地图左/下方：越界
  if (wx < origin_x_ || wy < origin_y_) {
    return false;
  }

  // 换算为栅格坐标（隐式向下取整）
  mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
  my = static_cast<unsigned int>((wy - origin_y_) / resolution_);

  // 检查是否超出右/上边界
  if (mx < size_x_ && my < size_y_) {
    return true;
  }
  return false;
}

// worldToMapNoBounds()：世界坐标 → 栅格坐标（不检查边界，可能为负）
// 供调用方自行裁剪的场合（如过滤器计算重叠窗口）
void Costmap2D::worldToMapNoBounds(double wx, double wy, int & mx, int & my) const
{
  mx = static_cast<int>((wx - origin_x_) / resolution_);
  my = static_cast<int>((wy - origin_y_) / resolution_);
}

// ============================================================================
// worldToMapEnforceBounds()：世界坐标 → 栅格坐标（越界钳位到边界格）
// 先比较边界再做除法，避免极大/极小 double 值在换算中溢出
// ============================================================================
void Costmap2D::worldToMapEnforceBounds(double wx, double wy, int & mx, int & my) const
{
  // Here we avoid doing any math to wx,wy before comparing them to
  // the bounds, so their values can go out to the max and min values
  // of double floating point.
  // x 方向：小于左边界 → 0；大于右边界 → size_x-1；否则正常换算
  if (wx < origin_x_) {
    mx = 0;
  } else if (wx > resolution_ * size_x_ + origin_x_) {
    mx = size_x_ - 1;
  } else {
    mx = static_cast<int>((wx - origin_x_) / resolution_);
  }

  // y 方向同理
  if (wy < origin_y_) {
    my = 0;
  } else if (wy > resolution_ * size_y_ + origin_y_) {
    my = size_y_ - 1;
  } else {
    my = static_cast<int>((wy - origin_y_) / resolution_);
  }
}

// ============================================================================
// updateOrigin()：滚动窗口原点平移——本文件最重要的算法之一
// 思路：新旧两个同尺寸窗口有一部分重叠，平移时只需保留重叠区数据：
//   1) 新原点换算为栅格偏移并网格对齐；
//   2) 计算重叠区在旧窗口中的位置，暂存到 local_map；
//   3) 复位整张地图（移出区域变为默认/未知）；
//   4) 把暂存数据写回新窗口中重叠区的位置。
// ============================================================================

void Costmap2D::updateOrigin(double new_origin_x, double new_origin_y)
{
  // project the new origin into the grid
  // 新原点相对旧原点的栅格偏移量（格数）
  int cell_ox, cell_oy;
  cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  // compute the associated world coordinates for the origin cell
  // because we want to keep things grid-aligned
  // 新原点对齐到栅格线（用整数偏移反算），避免亚栅格漂移逐周期累积
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  // To save casting from unsigned int to int a bunch of times
  int size_x = size_x_;
  int size_y = size_y_;

  // we need to compute the overlap of the new and existing windows
  // 计算新旧窗口的重叠区（裁剪到旧窗口的 [0,size] 范围内）
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = std::min(std::max(cell_ox, 0), size_x);
  lower_left_y = std::min(std::max(cell_oy, 0), size_y);
  upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  // we need a map to store the obstacles in the window temporarily
  // 临时缓冲区保存重叠区数据
  unsigned char * local_map = new unsigned char[cell_size_x * cell_size_y];

  // copy the local window in the costmap to the local map
  // 把旧窗口中的重叠区拷贝到临时数组
  copyMapRegion(
    costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x,
    cell_size_x,
    cell_size_y);

  // now we'll set the costmap to be completely unknown if we track unknown space
  // 整图复位（移入的新区域为默认值/未知）
  resetMaps();

  // update the origin with the appropriate world coordinates
  // 更新原点为网格对齐后的值
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  // compute the starting cell location for copying data back in
  // 重叠区在新窗口中的起始栅格位置
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  // now we want to copy the overlapping information back into the map, but in its new location
  // 把暂存的重叠区数据写回新位置
  copyMapRegion(
    local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x,
    cell_size_y);

  // make sure to clean up
  // 释放临时缓冲区
  delete[] local_map;
}

// ============================================================================
// setConvexPolygonCost()：把一个凸多边形（世界坐标顶点）内的所有格子
// 置为指定代价值——用于"足迹清除"（机器人自身区域置为 FREE_SPACE）
// ============================================================================
bool Costmap2D::setConvexPolygonCost(
  const std::vector<geometry_msgs::msg::Point> & polygon,
  unsigned char cost_value)
{
  // we assume the polygon is given in the global_frame...
  // we need to transform it to map coordinates
  // 顶点从世界坐标转为栅格坐标，任一顶点越界则失败
  std::vector<MapLocation> map_polygon;
  for (unsigned int i = 0; i < polygon.size(); ++i) {
    MapLocation loc;
    if (!worldToMap(polygon[i].x, polygon[i].y, loc.x, loc.y)) {
      // ("Polygon lies outside map bounds, so we can't fill it");
      return false;
    }
    map_polygon.push_back(loc);
  }

  std::vector<MapLocation> polygon_cells;

  // get the cells that fill the polygon
  // 栅格化填充：得到多边形覆盖的所有格子
  convexFillCells(map_polygon, polygon_cells);

  // set the cost of those cells
  // 逐格写入目标代价值
  for (unsigned int i = 0; i < polygon_cells.size(); ++i) {
    unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
    costmap_[index] = cost_value;
  }
  return true;
}

// ============================================================================
// polygonOutlineCells()：收集凸多边形边界（轮廓）经过的所有格子
// 用 raytraceLine（Bresenham）逐边追踪，最后闭合首尾
// ============================================================================
void Costmap2D::polygonOutlineCells(
  const std::vector<MapLocation> & polygon,
  std::vector<MapLocation> & polygon_cells)
{
  // 仿函数：每追踪到一个格子就加入 polygon_cells
  PolygonOutlineCells cell_gatherer(*this, costmap_, polygon_cells);
  // 依次追踪多边形的每条边
  for (unsigned int i = 0; i < polygon.size() - 1; ++i) {
    raytraceLine(cell_gatherer, polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y);
  }
  if (!polygon.empty()) {
    unsigned int last_index = polygon.size() - 1;
    // we also need to close the polygon by going from the last point to the first
    // 闭合：从最后一个顶点追到第一个顶点
    raytraceLine(
      cell_gatherer, polygon[last_index].x, polygon[last_index].y, polygon[0].x,
      polygon[0].y);
  }
}

// ============================================================================
// convexFillCells()：凸多边形填充——求多边形覆盖的所有内部格子
// 算法（扫描线思想的简化版）：
//   1) 先取轮廓格子（polygonOutlineCells）；
//   2) 轮廓点按 x 排序（冒泡）；
//   3) 对每个 x 列，轮廓在该列上的点成对出现（凸多边形性质），
//      取 y 的最小/最大值，把整列区间内的格子加入结果。
// ============================================================================
void Costmap2D::convexFillCells(
  const std::vector<MapLocation> & polygon,
  std::vector<MapLocation> & polygon_cells)
{
  // we need a minimum polygon of a triangle
  // 至少需要 3 个顶点（三角形）
  if (polygon.size() < 3) {
    return;
  }

  // first get the cells that make up the outline of the polygon
  // 第一步：获取轮廓格子（结果先存入 polygon_cells）
  polygonOutlineCells(polygon, polygon_cells);

  // quick bubble sort to sort points by x
  // 第二步：轮廓点按 x 升序排序（冒泡排序）
  MapLocation swap;
  unsigned int i = 0;
  while (i < polygon_cells.size() - 1) {
    if (polygon_cells[i].x > polygon_cells[i + 1].x) {
      swap = polygon_cells[i];
      polygon_cells[i] = polygon_cells[i + 1];
      polygon_cells[i + 1] = swap;

      if (i > 0) {
        --i;   // 交换后回退一步继续比较
      }
    } else {
      ++i;
    }
  }

  // 第三步：逐列扫描填充
  i = 0;
  MapLocation min_pt;
  MapLocation max_pt;
  unsigned int min_x = polygon_cells[0].x;
  unsigned int max_x = polygon_cells[polygon_cells.size() - 1].x;

  // walk through each column and mark cells inside the polygon
  // 遍历每一列 x
  for (unsigned int x = min_x; x <= max_x; ++x) {
    if (i >= polygon_cells.size() - 1) {
      break;
    }

    // 凸多边形每一列恰好有两个交点：取较小/较大 y
    if (polygon_cells[i].y < polygon_cells[i + 1].y) {
      min_pt = polygon_cells[i];
      max_pt = polygon_cells[i + 1];
    } else {
      min_pt = polygon_cells[i + 1];
      max_pt = polygon_cells[i];
    }

    i += 2;
    // 同列若有更多轮廓点（数值离散造成），继续扩展 min/max
    while (i < polygon_cells.size() && polygon_cells[i].x == x) {
      if (polygon_cells[i].y < min_pt.y) {
        min_pt = polygon_cells[i];
      } else if (polygon_cells[i].y > max_pt.y) {
        max_pt = polygon_cells[i];
      }
      ++i;
    }

    MapLocation pt;
    // loop though cells in the column
    // 把该列 [min_y, max_y] 区间内所有格子加入结果（内部填充）
    for (unsigned int y = min_pt.y; y <= max_pt.y; ++y) {
      pt.x = x;
      pt.y = y;
      polygon_cells.push_back(pt);
    }
  }
}

// ---- 以下为简单几何参数访问器 ----
unsigned int Costmap2D::getSizeInCellsX() const
{
  return size_x_;   // x 方向格数
}

unsigned int Costmap2D::getSizeInCellsY() const
{
  return size_y_;   // y 方向格数
}

double Costmap2D::getSizeInMetersX() const
{
  // 最后一格中心 + 半格 = 实际跨度（(size-1)+0.5)*resolution）
  return (size_x_ - 1 + 0.5) * resolution_;
}

double Costmap2D::getSizeInMetersY() const
{
  return (size_y_ - 1 + 0.5) * resolution_;
}

double Costmap2D::getOriginX() const
{
  return origin_x_;   // 地图左下角 x（米）
}

double Costmap2D::getOriginY() const
{
  return origin_y_;   // 地图左下角 y（米）
}

double Costmap2D::getResolution() const
{
  return resolution_;   // 分辨率（米/格）
}

// ============================================================================
// saveMap()：把代价地图保存为 PGM (P2, 文本格式) 图像文件（调试用）
// 头部：P2 宽 高 最大值(255)，随后按行输出各格代价
// ============================================================================
bool Costmap2D::saveMap(std::string file_name)
{
  FILE * fp = fopen(file_name.c_str(), "w");

  if (!fp) {
    return false;
  }

  // PGM 头：P2(ASCII 格式) 宽 高 最大灰度
  fprintf(fp, "P2\n%u\n%u\n%u\n", size_x_, size_y_, 0xff);
  // 逐行逐格输出代价值
  for (unsigned int iy = 0; iy < size_y_; iy++) {
    for (unsigned int ix = 0; ix < size_x_; ix++) {
      unsigned char cost = getCost(ix, iy);
      fprintf(fp, "%d ", cost);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  return true;
}

}  // namespace nav2_costmap_2d
