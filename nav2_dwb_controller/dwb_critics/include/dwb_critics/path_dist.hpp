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
#ifndef DWB_CRITICS__PATH_DIST_HPP_
#define DWB_CRITICS__PATH_DIST_HPP_

#include "dwb_critics/map_grid.hpp"

// ============================================================================
// 模块说明：路径距离评价器（PathDistCritic）
// ----------------------------------------------------------------------------
// 继承自 MapGridCritic。与 GoalDistCritic（以"最远可达路径点"为目标）不同，
// PathDistCritic 以"全局路径本身"为源点集合：
//   prepare 阶段把落在局部代价地图内的全局路径点全部作为 BFS 源点，
//   生成"到全局路径的距离场"——离路径越近的格子分值越低。
// 打分时查询轨迹终点所在格的距离场值，从而惩罚远离全局路径的轨迹。
// 它与 GoalDistCritic 的分工：
//   - PathDistCritic：保证轨迹"贴着路径走"（横向偏差小）；
//   - GoalDistCritic：保证轨迹"朝目标方向前进"（取得进展）。
// 两者组合即可同时约束横向偏差与纵向进度。
// ============================================================================

namespace dwb_critics
{
/**
 * @class PathDistCritic
 * @brief Scores trajectories based on how far from the global path they end up.
 */
// 路径距离评价器：以全局路径为源点的 BFS 距离场惩罚轨迹偏离路径
class PathDistCritic : public MapGridCritic
{
public:
  // 每周期准备：将局部代价地图内的全局路径点作为源点，重建 BFS 距离场
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS__PATH_DIST_HPP_
