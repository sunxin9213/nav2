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

#include "dwb_critics/map_grid.hpp"
#include <cmath>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <memory>
#include "dwb_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"

using std::abs;
using costmap_queue::CellData;

namespace dwb_critics
{

// 中文说明（文件级）：
// 地图栅格（Map Grid）critic 基类：基于“曼哈顿距离场”的一族评分器。
// 核心思想：将参考目标（全局路径点或目标点）作为距离为 0 的种子，
// 用 CostmapQueue 从种子做宽度优先扩散，为每个栅格计算到最近种子的
// 曼哈顿距离（|dx|+|dy|），存入 cell_values_；轨迹得分即其位姿点
// 对应的栅格距离。派生类（PathDist/GoalDist/PathAlign/GoalAlign/
// PreferForward/Oscillation）只需在 prepare() 中决定哪些栅格做种子。
// 支持三种聚合方式（last/sum/product）。

// Customization of the CostmapQueue validCellToQueue method
//（重写扩散队列的“可入队”判定：本基类允许所有栅格入队）
bool MapGridCritic::MapGridQueue::validCellToQueue(const costmap_queue::CellData & /*cell*/)
{
  return true;
}

// 初始化：创建扩散队列并读取聚合方式参数
void MapGridCritic::onInit()
{
  costmap_ = costmap_ros_->getCostmap();
  queue_ = std::make_shared<MapGridQueue>(*costmap_, *this);

  // Always set to true, but can be overriden by subclasses
  //（默认在失败时抛异常；子类可覆盖，如 OscillationCritic 置 false）
  stop_on_failure_ = true;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // 聚合方式：last=只取轨迹最后一点的距离；sum=各点求和；product=各点求积
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".aggregation_type",
    rclcpp::ParameterValue(std::string("last")));

  std::string aggro_str;
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".aggregation_type", aggro_str);
  std::transform(aggro_str.begin(), aggro_str.end(), aggro_str.begin(), ::tolower);
  if (aggro_str == "last") {
    aggregationType_ = ScoreAggregationType::Last;
  } else if (aggro_str == "sum") {
    aggregationType_ = ScoreAggregationType::Sum;
  } else if (aggro_str == "product") {
    aggregationType_ = ScoreAggregationType::Product;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "MapGridCritic"), "aggregation_type parameter \"%s\" invalid. Using Last.",
      aggro_str.c_str());
    aggregationType_ = ScoreAggregationType::Last;
  }
}

// 将某栅格标记为障碍（距离记为 obstacle_score_ 最大罚分）
void MapGridCritic::setAsObstacle(unsigned int index)
{
  cell_values_[index] = obstacle_score_;
}

// 重置：清空队列并将所有栅格距离初始化为“不可达”罚分
void MapGridCritic::reset()
{
  queue_->reset();
  cell_values_.resize(costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY());
  // 障碍罚分 = 栅格总数（保证大于任何可达距离）
  obstacle_score_ = static_cast<double>(cell_values_.size());
  unreachable_score_ = obstacle_score_ + 1.0;  // 不可达罚分再 +1
  std::fill(cell_values_.begin(), cell_values_.end(), unreachable_score_);
}

// 曼哈顿距离扩散：从种子栅格 BFS 扩散，逐格填入到最近种子的曼哈顿距离
void MapGridCritic::propogateManhattanDistances()
{
  while (!queue_->isEmpty()) {
    costmap_queue::CellData cell = queue_->getNextCell();
    // 距离 = |源格x - 当前格x| + |源格y - 当前格y|（L1 范数）
    cell_values_[cell.index_] = CellData::absolute_difference(cell.src_x_, cell.x_) +
      CellData::absolute_difference(cell.src_y_, cell.y_);
  }
}

// 轨迹评分：逐点查询栅格距离，按聚合方式合成总分
double MapGridCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  double score = 0.0;
  unsigned int start_index = 0;
  if (aggregationType_ == ScoreAggregationType::Product) {
    score = 1.0;  // 乘积模式初值为 1
  } else if (aggregationType_ == ScoreAggregationType::Last && !stop_on_failure_) {
    start_index = traj.poses.size() - 1;  // last 且允许失败时只看最后一个点
  }
  double grid_dist;

  for (unsigned int i = start_index; i < traj.poses.size(); ++i) {
    grid_dist = scorePose(traj.poses[i]);
    if (stop_on_failure_) {
      // 命中障碍或不可达区域：直接抛非法轨迹异常剔除该轨迹
      if (grid_dist == obstacle_score_) {
        throw dwb_core::
              IllegalTrajectoryException(name_, "Trajectory Hits Obstacle.");
      } else if (grid_dist == unreachable_score_) {
        throw dwb_core::
              IllegalTrajectoryException(name_, "Trajectory Hits Unreachable Area.");
      }
    }

    // 按聚合方式累加
    switch (aggregationType_) {
      case ScoreAggregationType::Last:
        score = grid_dist;  // 只保留最后一个点的距离
        break;
      case ScoreAggregationType::Sum:
        score += grid_dist;  // 各点距离求和
        break;
      case ScoreAggregationType::Product:
        if (score > 0) {
          score *= grid_dist;  // 各点距离连乘
        }
        break;
    }
  }

  return score;
}

// 单点评分：位姿转栅格后查距离表
double MapGridCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  unsigned int cell_x, cell_y;
  // we won't allow trajectories that go off the map... shouldn't happen that often anyways
  //（出地图的轨迹视为非法）
  if (!costmap_->worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Goes Off Grid.");
  }
  return getScore(cell_x, cell_y);
}

// 导出距离场为可视化通道（整幅栅格的得分）
void MapGridCritic::addCriticVisualization(
  std::vector<std::pair<std::string, std::vector<float>>> & cost_channels)
{
  std::pair<std::string, std::vector<float>> grid_scores;
  grid_scores.first = name_;

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();
  grid_scores.second.resize(size_x * size_y);
  unsigned int i = 0;
  for (unsigned int cy = 0; cy < size_y; cy++) {
    for (unsigned int cx = 0; cx < size_x; cx++) {
      grid_scores.second[i] = getScore(cx, cy);
      i++;
    }
  }
  cost_channels.push_back(grid_scores);
}

}  // namespace dwb_critics
