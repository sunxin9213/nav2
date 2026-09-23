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

#include "dwb_critics/path_dist.hpp"
#include <vector>
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/path_ops.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace dwb_critics
{
// 中文说明（文件级）：
// 路径距离（Path Dist）critic：以全局路径（限局部代价地图内的部分）
// 为距离场种子，惩罚轨迹点到全局路径的曼哈顿距离，
// 使候选轨迹贴近全局路径行进。继承 MapGridCritic 的距离扩散机制。

// 周期准备：将局部地图内的全局路径点作为 0 距离种子并扩散距离场
bool PathDistCritic::prepare(
  const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  reset();  // 清零距离场（全部设为不可达）
  bool started_path = false;

  // 将全局路径重采样到代价地图分辨率（过稀的路径段插入中间点）
  nav_2d_msgs::msg::Path2D adjusted_global_plan =
    nav_2d_utils::adjustPlanResolution(global_plan, costmap_->getResolution());

  if (adjusted_global_plan.poses.size() != global_plan.poses.size()) {
    RCLCPP_DEBUG(
      rclcpp::get_logger(
        "PathDistCritic"), "Adjusted global plan resolution, added %zu points",
      adjusted_global_plan.poses.size() - global_plan.poses.size());
  }

  unsigned int i;
  // put global path points into local map until we reach the border of the local map
  //（把落在局部代价地图内且非未知区域的路径点设为 0 距离种子；
  //  一旦离开局部地图即停止，只跟踪局部段）
  for (i = 0; i < adjusted_global_plan.poses.size(); ++i) {
    double g_x = adjusted_global_plan.poses[i].x;
    double g_y = adjusted_global_plan.poses[i].y;
    unsigned int map_x, map_y;
    if (costmap_->worldToMap(
        g_x, g_y, map_x,
        map_y) && costmap_->getCost(map_x, map_y) != nav2_costmap_2d::NO_INFORMATION)
    {
      // 该路径点在局部地图内：设为种子（距离 0）并入扩散队列
      int index = costmap_->getIndex(map_x, map_y);
      cell_values_[index] = 0.0;
      queue_->enqueueCell(map_x, map_y);
      started_path = true;
    } else if (started_path) {
      break;  // 路径已离开局部地图：停止收集种子
    }
  }
  if (!started_path) {
    // 局部地图内没有任何路径点：评分无法进行
    RCLCPP_ERROR(
      rclcpp::get_logger("PathDistCritic"),
      "None of the %d first of %zu (%zu) points of the global plan were in "
      "the local costmap and free",
      i, adjusted_global_plan.poses.size(), global_plan.poses.size());
    return false;
  }

  // 从种子栅格做曼哈顿距离扩散，生成完整距离场
  propogateManhattanDistances();

  return true;
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::PathDistCritic, dwb_core::TrajectoryCritic)
