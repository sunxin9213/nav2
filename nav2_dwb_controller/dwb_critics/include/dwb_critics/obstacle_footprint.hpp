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

#ifndef DWB_CRITICS__OBSTACLE_FOOTPRINT_HPP_
#define DWB_CRITICS__OBSTACLE_FOOTPRINT_HPP_

#include <vector>
#include "dwb_critics/base_obstacle.hpp"

// ============================================================================
// 模块说明：足印障碍物评价器（ObstacleFootprintCritic）
// ----------------------------------------------------------------------------
// 在 BaseObstacleCritic（圆形机器人单格检测）的基础上，支持任意多边形
// footprint 的精确碰撞检测，适用于非圆形机器人。
// 核心思路（边界检查，border checking）：
//   1. prepare 时把 footprint_spec_（以机器人中心为原点的多边形顶点）按
//      当前位姿旋转平移，得到"定向足印"（oriented footprint）；
//   2. 对多边形的每条边用 LineIterator（Bresenham 算法）栅格化；
//   3. 检查边经过的每个格子的代价值（pointCost），任一非法即整条边/轨迹
//      判为碰撞（负分）。
// 为什么只检查边界就足够？——前提是局部代价地图的障碍物已经过膨胀
// （inflation）：如果 footprint 边界上任意一点距离障碍都大于 0（处于
// 障碍物的膨胀代价区内且代价值可接受），则整个 footprint 内部必然也不
// 碰撞。若不膨胀，则需要更鲁棒（但更慢）的"内部全格检查"。
// ============================================================================

namespace dwb_critics
{
typedef std::vector<geometry_msgs::msg::Point> Footprint;  // 足印多边形 = 顶点列表

/**
 * @brief Transform the footprint spec to be centered at the given pose
 * @param pose Robot pose
 * @param footprint_spec List of points that make up the footprint spec, centered at 0,0
 * @return oriented footprint
 */
// 将以 (0,0) 为中心的足印规格按指定位姿旋转平移，得到世界坐标系下的定向足印
Footprint getOrientedFootprint(
  const geometry_msgs::msg::Pose2D & pose,
  const Footprint & footprint_spec);

/**
 * @class ObstacleFootprintCritic
 * @brief Uses costmap 2d to assign negative costs if robot footprint is in obstacle on any point of the trajectory.
 *
 * Internally, this technically only checks if the border of the footprint collides with anything for computational
 * efficiency. This is valid if the obstacles in the local costmap are inflated.
 *
 * A more robust class could check every cell within the robot's footprint without inflating the obstacles,
 * at some computational cost. That is left as an excercise to the reader.
 */
// 足印障碍物评价器：通过栅格化足印边界线检测非圆形机器人的碰撞
class ObstacleFootprintCritic : public BaseObstacleCritic
{
public:
  // 每周期准备：读取代价地图当前的 footprint 并保存到 footprint_spec_
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  // 打分重载（基类接口）：先把位姿转成定向足印，再调用下方带足印参数的版本
  double scorePose(const geometry_msgs::msg::Pose2D & pose) override;
  // 打分核心：对定向足印的每条边调用 lineCost 检查，返回最大（最差）边代价
  virtual double scorePose(
    const geometry_msgs::msg::Pose2D & pose,
    const Footprint & oriented_footprint);
  // 重写 getScale：与基类不同，这里乘整格分辨率（而非一半），因为检查的
  // 是边界线而非中心点，代价粒度是完整的一格
  double getScale() const override {return costmap_->getResolution() * scale_;}

protected:
  /**
   * @brief Rasterizes a line in the costmap grid and checks for collisions
   * @param x0 The x position of the first cell in grid coordinates
   * @param y0 The y position of the first cell in grid coordinates
   * @param x1 The x position of the second cell in grid coordinates
   * @param y1 The y position of the second cell in grid coordinates
   * @return A positive cost for a legal line... negative otherwise
   */
  // 用 Bresenham 线迭代器栅格化一条边并逐格检查 pointCost；
  // 返回该线上的最大（最差）代价值，遇非法格返回负分
  double lineCost(int x0, int x1, int y0, int y1);

  /**
   * @brief Checks the cost of a point in the costmap
   * @param x The x position of the point in cell coordinates
   * @param y The y position of the point in cell coordinates
   * @return A positive cost for a legal point... negative otherwise
   */
  // 检查单个格子的代价值：越界或非法（LETHAL/INSCRIBED/NO_INFORMATION）返回负分
  double pointCost(int x, int y);

  Footprint footprint_spec_;  // 足印规格（以机器人中心为原点的多边形顶点表）
};
}  // namespace dwb_critics

#endif  // DWB_CRITICS__OBSTACLE_FOOTPRINT_HPP_
