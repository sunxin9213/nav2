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

#include "dwb_critics/obstacle_footprint.hpp"
#include <algorithm>
#include <vector>
#include "dwb_critics/line_iterator.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::ObstacleFootprintCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{
// 中文说明（文件级）：
// 足迹障碍物 critic：用完整多边形足迹（而非圆形近似）做碰撞检查。
// 继承 BaseObstacleCritic，重写 scorePose：将机器人足迹按位姿朝向
// 旋转平移后栅格化到代价地图，对足迹每条边用 LineIterator 光栅化，
// 取边上所有栅格的最大代价值作为该位姿得分；任一栅格为致命障碍或
// 未知区域则抛 IllegalTrajectoryException 剔除轨迹。

// 将足迹规格按位姿（位置+朝向）旋转平移，得到世界坐标系下的定向足迹
Footprint getOrientedFootprint(
  const geometry_msgs::msg::Pose2D & pose,
  const Footprint & footprint_spec)
{
  std::vector<geometry_msgs::msg::Point> oriented_footprint;
  oriented_footprint.resize(footprint_spec.size());
  double cos_th = cos(pose.theta);
  double sin_th = sin(pose.theta);
  // 二维旋转：p' = pose + R(θ) * p
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
    geometry_msgs::msg::Point & new_pt = oriented_footprint[i];
    new_pt.x = pose.x + footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th;
    new_pt.y = pose.y + footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th;
  }
  return oriented_footprint;
}

// 周期准备：从代价地图获取机器人足迹规格
bool ObstacleFootprintCritic::prepare(
  const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &)
{
  footprint_spec_ = costmap_ros_->getRobotFootprint();
  if (footprint_spec_.size() == 0) {
    // 未设置足迹（可能仍在使用圆形模型）：无法用足迹检查
    RCLCPP_ERROR(
      rclcpp::get_logger("ObstacleFootprintCritic"),
      "Footprint spec is empty, maybe missing call to setFootprint?");
    return false;
  }
  return true;
}

// 单点位姿评分：先做中心点越界检查，再用定向足迹评分
double ObstacleFootprintCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  unsigned int cell_x, cell_y;
  if (!costmap_->worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Goes Off Grid.");
  }
  return scorePose(pose, getOrientedFootprint(pose, footprint_spec_));
}

// 足迹评分：对足迹各边栅格化并取最大代价值
double ObstacleFootprintCritic::scorePose(
  const geometry_msgs::msg::Pose2D &,
  const Footprint & footprint)
{
  // now we really have to lay down the footprint in the costmap grid
  unsigned int x0, x1, y0, y1;
  double line_cost = 0.0;
  double footprint_cost = 0.0;

  // we need to rasterize each line in the footprint
  //（逐条边栅格化：检查相邻顶点连成的每条边）
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    // get the cell coord of the first point
    //（边的起点转栅格坐标，越界即非法）
    if (!costmap_->worldToMap(footprint[i].x, footprint[i].y, x0, y0)) {
      throw dwb_core::
            IllegalTrajectoryException(name_, "Footprint Goes Off Grid.");
    }

    // get the cell coord of the second point
    //（边的终点转栅格坐标）
    if (!costmap_->worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
      throw dwb_core::
            IllegalTrajectoryException(name_, "Footprint Goes Off Grid.");
    }

    // 该边的最大代价值，取与已检查边的最大值
    line_cost = lineCost(x0, x1, y0, y1);
    footprint_cost = std::max(line_cost, footprint_cost);
  }

  // we also need to connect the first point in the footprint to the last point
  //（闭合多边形：还需检查“最后顶点 → 首顶点”这条边）
  if (!costmap_->worldToMap(footprint.back().x, footprint.back().y, x0, y0)) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Footprint Goes Off Grid.");
  }

  if (!costmap_->worldToMap(footprint.front().x, footprint.front().y, x1, y1)) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Footprint Goes Off Grid.");
  }

  line_cost = lineCost(x0, x1, y0, y1);
  footprint_cost = std::max(line_cost, footprint_cost);

  // if all line costs are legal... then we can return that the footprint is legal
  //（足迹所有边的最大代价值即该位姿得分）
  return footprint_cost;
}

// 线段评分：用 Bresenham 直线迭代器遍历线段上的所有栅格，取最大代价
double ObstacleFootprintCritic::lineCost(int x0, int x1, int y0, int y1)
{
  double line_cost = 0.0;
  double point_cost = -1.0;

  for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    point_cost = pointCost(line.getX(), line.getY());   // Score the current point

    if (line_cost < point_cost) {
      line_cost = point_cost;  // 保留线上最大代价值
    }
  }

  return line_cost;
}

// 单栅格评分：查询代价值，致命障碍/未知区域抛异常
double ObstacleFootprintCritic::pointCost(int x, int y)
{
  unsigned char cost = costmap_->getCost(x, y);
  // if the cell is in an obstacle the path is invalid or unknown
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Hits Obstacle.");
  } else if (cost == nav2_costmap_2d::NO_INFORMATION) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Hits Unknown Region.");
  }

  return cost;
}

}  // namespace dwb_critics
