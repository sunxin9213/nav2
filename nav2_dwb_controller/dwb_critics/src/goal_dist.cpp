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

#include "dwb_critics/goal_dist.hpp"
#include <vector>
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/path_ops.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace dwb_critics
{
// 中文说明（文件级）：
// 目标距离（Goal Dist）critic：以“局部地图内的最后一个路径点”
//（即局部目标）为唯一种子构建曼哈顿距离场，惩罚轨迹终点离
// 局部目标的距离，驱动机器人沿全局路径向目标推进。

// 周期准备：把局部目标设为 0 距离种子并扩散
bool GoalDistCritic::prepare(
  const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  reset();  // 清零距离场

  // 找到局部代价地图内的最后一个路径点
  unsigned int local_goal_x, local_goal_y;
  if (!getLastPoseOnCostmap(global_plan, local_goal_x, local_goal_y)) {
    return false;
  }

  // Enqueue just the last pose
  //（只把该点作为种子：距离 0 并入队）
  int index = costmap_->getIndex(local_goal_x, local_goal_y);
  cell_values_[index] = 0.0;
  queue_->enqueueCell(local_goal_x, local_goal_y);

  // 从局部目标扩散曼哈顿距离场
  propogateManhattanDistances();

  return true;
}

// 查找全局路径在局部代价地图内的最后一个点
bool GoalDistCritic::getLastPoseOnCostmap(
  const nav_2d_msgs::msg::Path2D & global_plan,
  unsigned int & x, unsigned int & y)
{
  // 先把路径重采样到地图分辨率
  nav_2d_msgs::msg::Path2D adjusted_global_plan = nav_2d_utils::adjustPlanResolution(
    global_plan,
    costmap_->getResolution());
  bool started_path = false;

  // skip global path points until we reach the border of the local map
  //（遍历路径，持续更新“最近一次落在地图内”的点）
  for (unsigned int i = 0; i < adjusted_global_plan.poses.size(); ++i) {
    double g_x = adjusted_global_plan.poses[i].x;
    double g_y = adjusted_global_plan.poses[i].y;
    unsigned int map_x, map_y;
    if (costmap_->worldToMap(
        g_x, g_y, map_x,
        map_y) && costmap_->getCost(map_x, map_y) != nav2_costmap_2d::NO_INFORMATION)
    {
      // Still on the costmap. Continue.
      //（仍在地图内：记录该点，继续向后找）
      x = map_x;
      y = map_y;
      started_path = true;
    } else if (started_path) {
      // Off the costmap after being on the costmap. Return the last saved indices.
      //（路径已离开局部地图：返回最后记录的点）
      return true;
    }
    // else, we have not yet found a point on the costmap, so we just continue
    //（尚未进入地图：继续向后扫描）
  }

  if (started_path) {
    return true;  // 整条路径都在地图内：返回其最后一点
  } else {
    // 路径完全不在局部地图内：无法评分
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "GoalDistCritic"), "None of the points of the global plan were in the local costmap.");
    return false;
  }
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::GoalDistCritic, dwb_core::TrajectoryCritic)
