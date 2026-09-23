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
#ifndef DWB_CRITICS__GOAL_DIST_HPP_
#define DWB_CRITICS__GOAL_DIST_HPP_

#include <vector>
#include "dwb_critics/map_grid.hpp"

// ============================================================================
// 模块说明：目标距离评价器（GoalDistCritic）
// ----------------------------------------------------------------------------
// 继承自 MapGridCritic（基于 BFS 的地图栅格距离场评价器）。
// 作用：保证机器人沿全局路径取得"进展"（progress）。
// 工作流程（在 prepare 阶段完成）：
//   1. 在全局路径上找到"距离机器人最远、且仍落在代价地图范围内"的位姿
//      （即 getLastPoseOnCostmap，从局部代价地图边界截取全局路径）；
//   2. 将该位姿所在的栅格作为 MapGridCritic 的"源点"（source cell），
//      BFS 扩散生成到该点的距离场；
//   3. 打分时查询轨迹位姿所在格的距离场值——离临时目标越近分越低。
// 由于局部代价地图有限，"最远可达位姿"会随机器人前进不断更新，机器人
// 因此被持续"牵引"沿全局路径前进。
// ============================================================================

namespace dwb_critics
{
/**
 * @class GoalDistCritic
 * @brief Scores trajectories based on how far along the global path they end up.
 *
 * This trajectory critic helps ensure progress along the global path. It finds the pose from the
 * global path farthest from the robot that is still on the costmap, and aims for that point by
 * assigning the lowest cost to the cell corresponding with that farthest pose.
 */
// 目标距离评价器：以"局部代价地图内最远的全局路径点"为临时目标的距离场打分
class GoalDistCritic : public MapGridCritic
{
public:
  // 每个评分周期的准备：调用 getLastPoseOnCostmap 找临时目标点，再以该点为
  // 源点调用 MapGridCritic::reset 重建 BFS 距离场
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;

protected:
  // 在全局路径上找到最后一个仍位于代价地图范围内的位姿，输出其栅格坐标 (x, y)；
  // 找不到时返回 false（此时该评价器跳过本轮打分）
  bool getLastPoseOnCostmap(
    const nav_2d_msgs::msg::Path2D & global_plan, unsigned int & x,
    unsigned int & y);
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS__GOAL_DIST_HPP_
