// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Modified by: Shivang Patel (shivaang14@gmail.com)
// ============================================================================
// 中文注释：足迹碰撞检测器（FootprintCollisionChecker）实现文件
//
// 【文件功能】
// 本文件实现基于"足迹多边形"的碰撞检测：给定机器人在某位置的
// 足迹（世界坐标多边形），计算足迹扫过区域内的最大代价值：
//   - LETHAL_OBSTACLE(254)：必然碰撞；
//   - INSCRIBED_INFLATED_OBSTACLE(253)：中心进入必碰区；
//   - 其他：返回沿足迹的最大代价（值越大越危险）。
// 供控制器（如 Regulated Pure Pursuit）做轨迹可行性评估。
//
// 【在 Nav2 中的位置】
// 模板类，模板参数 CostmapT 可以是 Costmap2D* 或
// shared_ptr<Costmap2D>（文件末显式实例化两种）。
// costmap_topic_collision_checker（话题版）继承本类，把地图来源
// 换成实时订阅的地图话题。
//
// 【核心算法】
// 足迹本身是"边框"而非实心区域：检测方式是把多边形的每条边
// 用 Bresenham 直线（nav2_util::LineIterator）栅格化，
// 取线上经过格子的最大代价——即检查足迹"轮廓线"的碰撞情况
// （机器人本体最先进碰到的必然是边缘）。
// ============================================================================

#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/exceptions.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/line_iterator.hpp"

using namespace std::chrono_literals;

namespace nav2_costmap_2d
{

// 默认构造：代价地图指针为空（后续 setCostmap 注入）
template<typename CostmapT>
FootprintCollisionChecker<CostmapT>::FootprintCollisionChecker()
: costmap_(nullptr)
{
}

// 带地图的构造：直接注入代价地图
template<typename CostmapT>
FootprintCollisionChecker<CostmapT>::FootprintCollisionChecker(
  CostmapT costmap)
: costmap_(costmap)
{
}

// ============================================================================
// footprintCost()：计算足迹（世界坐标多边形）的碰撞代价
// 逐边栅格化取最大代价；任一顶点在地图外 → 直接视为致命障碍
// ============================================================================

template<typename CostmapT>
double FootprintCollisionChecker<CostmapT>::footprintCost(const Footprint footprint)
{
  // now we really have to lay down the footprint in the costmap_ grid
  unsigned int x0, x1, y0, y1;
  double footprint_cost = 0.0;

  // get the cell coord of the first point
  // 第一个顶点：世界坐标 → 栅格坐标，出界视为致命（地图外=未知风险）
  if (!worldToMap(footprint[0].x, footprint[0].y, x0, y0)) {
    return static_cast<double>(LETHAL_OBSTACLE);
  }

  // cache the start to eliminate a worldToMap call
  // 缓存起点（最后闭合边还要用，避免重复换算）
  unsigned int xstart = x0;
  unsigned int ystart = y0;

  // we need to rasterize each line in the footprint
  // 遍历足迹的每条边 (i → i+1)
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    // get the cell coord of the second point
    // 边的终点：出界同样视为致命
    if (!worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
      return static_cast<double>(LETHAL_OBSTACLE);
    }

    // 该边经过格子的最大代价，与累计值取 max
    footprint_cost = std::max(lineCost(x0, x1, y0, y1), footprint_cost);

    // the second point is next iteration's first point
    // 当前边的终点是下一条边的起点
    x0 = x1;
    y0 = y1;

    // if in collision, no need to continue
    // 已致命碰撞：提前返回（短路优化）
    if (footprint_cost == static_cast<double>(LETHAL_OBSTACLE)) {
      return footprint_cost;
    }
  }

  // we also need to connect the first point in the footprint to the last point
  // the last iteration's x1, y1 are the last footprint point's coordinates
  // 闭合边：最后一个顶点 → 第一个顶点
  return std::max(lineCost(xstart, x1, ystart, y1), footprint_cost);
}

// ============================================================================
// lineCost()：一条线段（栅格坐标）的代价
// 用 LineIterator（Bresenham 直线）遍历线上每个格子，
// 返回最大代价值；遇到致命障碍提前返回
// ============================================================================

template<typename CostmapT>
double FootprintCollisionChecker<CostmapT>::lineCost(int x0, int x1, int y0, int y1) const
{
  double line_cost = 0.0;
  double point_cost = -1.0;

  // LineIterator：Bresenham 直线栅格化迭代器
  for (nav2_util::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    point_cost = pointCost(line.getX(), line.getY());   // Score the current point

    // if in collision, no need to continue
    if (point_cost == static_cast<double>(LETHAL_OBSTACLE)) {
      return point_cost;   // 致命：短路返回
    }

    if (line_cost < point_cost) {
      line_cost = point_cost;   // 保留线上最大代价
    }
  }

  return line_cost;
}

// worldToMap()：世界坐标 → 栅格坐标（转发给注入的代价地图）
template<typename CostmapT>
bool FootprintCollisionChecker<CostmapT>::worldToMap(
  double wx, double wy, unsigned int & mx, unsigned int & my)
{
  return costmap_->worldToMap(wx, wy, mx, my);
}

// pointCost()：单个格子的代价（index = x + y*size_x 由 getCost 内部处理）
template<typename CostmapT>
double FootprintCollisionChecker<CostmapT>::pointCost(int x, int y) const
{
  return costmap_->getCost(x, y);
}

// setCostmap()：注入/更换代价地图
template<typename CostmapT>
void FootprintCollisionChecker<CostmapT>::setCostmap(CostmapT costmap)
{
  costmap_ = costmap;
}

// ============================================================================
// footprintCostAtPose()：给定机器人位姿 (x,y,theta) 与车体系足迹，
// 先旋转平移足迹到世界坐标，再计算碰撞代价
// （控制器评估候选轨迹时的标准入口）
// ============================================================================

template<typename CostmapT>
double FootprintCollisionChecker<CostmapT>::footprintCostAtPose(
  double x, double y, double theta, const Footprint footprint)
{
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  Footprint oriented_footprint;
  // 二维旋转矩阵 + 平移：把车体系顶点变换到世界系
  for (unsigned int i = 0; i < footprint.size(); ++i) {
    geometry_msgs::msg::Point new_pt;
    new_pt.x = x + (footprint[i].x * cos_th - footprint[i].y * sin_th);
    new_pt.y = y + (footprint[i].x * sin_th + footprint[i].y * cos_th);
    oriented_footprint.push_back(new_pt);
  }

  return footprintCost(oriented_footprint);
}

// declare our valid template parameters
// 显式实例化支持的模板参数：裸指针与 shared_ptr 两种地图持有方式
template class FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>;
template class FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>;

}  // namespace nav2_costmap_2d
