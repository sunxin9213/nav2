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

#ifndef DWB_CRITICS__BASE_OBSTACLE_HPP_
#define DWB_CRITICS__BASE_OBSTACLE_HPP_

#include <string>
#include <vector>
#include <utility>

#include "dwb_core/trajectory_critic.hpp"

// ============================================================================
// 模块说明：基础障碍物评价器（BaseObstacleCritic）
// ----------------------------------------------------------------------------
// DWB 轨迹评价器家族中最简单的碰撞检测基类，仅适用于"圆形机器人"假设：
//   机器人被近似为一个圆（内切圆半径），因此只需检查轨迹上每个位姿所在的
//   单个代价格（cell）的代价，即可判断是否碰撞，无需检查整个 footprint。
// 评分规则：
//   - 若任一位姿所在格为障碍物（LETHAL）、内切障碍（INSCRIBED）或未知
//     （NO_INFORMATION），返回负分数（表示轨迹非法，直接被淘汰）；
//   - 否则根据 sum_scores 参数二选一：
//       false（默认）：只返回轨迹最后一个位姿的代价值（终点代价）；
//       true        ：返回轨迹上所有位姿代价之和（累计代价）。
// 派生类（如 ObstacleFootprintCritic）可重写 scorePose 以支持更复杂的
// （非圆形）机器人碰撞检测。
// ============================================================================

namespace dwb_critics
{
/**
 * @class BaseObstacleCritic
 * @brief Uses costmap 2d to assign negative costs if a circular robot would collide at any point of the trajectory.
 *
 * This class can only be used to figure out if a circular robot is in collision. If the cell corresponding
 * with any of the poses in the Trajectory is an obstacle, inscribed obstacle or unknown, it will return a
 * negative cost. Otherwise it will return either the final pose's cost, or the sum of all poses, depending
 * on the sum_scores parameter.
 *
 * Other classes (like ObstacleFootprintCritic) can do more advanced checking for collisions.
 */
// 基础障碍物评价器：针对圆形机器人的单格碰撞检测。任何位姿落在
// LETHAL/INSCRIBED/NO_INFORMATION 格上即判为碰撞（负分 = 非法轨迹）。
class BaseObstacleCritic : public dwb_core::TrajectoryCritic
{
public:
  // 初始化：声明 sum_scores 参数并保存代价地图指针
  void onInit() override;
  // 对整条轨迹打分：逐位姿调用 scorePose，按 sum_scores 决定求和或取终点值
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  // 添加可视化通道：将本评价器的原始代价值输出到调试用 cost_channels
  void addCriticVisualization(
    std::vector<std::pair<std::string, std::vector<float>>> & cost_channels) override;

  /**
   * @brief Return the obstacle score for a particular pose
   * @param pose Pose to check
   */
  // 返回单个位姿的障碍代价：查该位姿所在格的代价值（虚拟函数，供派生类扩展）
  virtual double scorePose(const geometry_msgs::msg::Pose2D & pose);

  /**
   * @brief Check to see whether a given cell cost is valid for driving through.
   * @param cost Cost of the cell
   * @return Return true if valid cell
   */
  // 判断给定格代价值是否可通过：代价为 LETHAL/INSCRIBED/NO_INFORMATION 时返回 false
  virtual bool isValidCost(const unsigned char cost);

protected:
  nav2_costmap_2d::Costmap2D * costmap_;  // 底层代价地图（不拥有所有权，仅借用指针）
  bool sum_scores_;  // true=所有位姿代价求和；false=只取最后位姿代价
};
}  // namespace dwb_critics

#endif  // DWB_CRITICS__BASE_OBSTACLE_HPP_
