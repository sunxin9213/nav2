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
#include <vector>
#include <string>
#include <utility>

#include "dwb_critics/base_obstacle.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::BaseObstacleCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

// 中文说明（文件级）：
// 基础障碍物 critic：检查轨迹上每个位姿点的代价地图代价值。
// 任一点处于致命障碍/内切障碍/未知区域则抛 IllegalTrajectoryException
// 将该轨迹直接判为非法（不参与排序）；合法轨迹的得分为各点代价值
//（可配置取总和或最大/末值）。是 ObstacleFootprintCritic 的圆形基类。

// 初始化：获取代价地图并读取评分模式参数
void BaseObstacleCritic::onInit()
{
  costmap_ = costmap_ros_->getCostmap();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // sum_scores：true 时轨迹得分=各点代价之和；false 时=最后一点的代价
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".sum_scores", rclcpp::ParameterValue(false));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".sum_scores", sum_scores_);
}

// 轨迹评分：逐点查询代价值并按模式聚合
double BaseObstacleCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  double score = 0.0;
  for (unsigned int i = 0; i < traj.poses.size(); ++i) {
    double pose_score = scorePose(traj.poses[i]);
    // Optimized/branchless version of if (sum_scores_) score += pose_score,
    // else score = pose_score;
    //（无分支优化：sum_scores_ 为 0/1，等价于“求和或只取当前点”两种模式）
    score = static_cast<double>(sum_scores_) * score + pose_score;
  }
  return score;
}

// 单点位姿评分：查询代价地图，非法则抛异常使整条轨迹被剔除
double BaseObstacleCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  unsigned int cell_x, cell_y;
  // 世界坐标转栅格坐标；越界（出地图）视为非法轨迹
  if (!costmap_->worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Goes Off Grid.");
  }
  unsigned char cost = costmap_->getCost(cell_x, cell_y);
  // 碰到障碍物同样抛非法轨迹异常
  if (!isValidCost(cost)) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Hits Obstacle.");
  }
  return cost;  // 返回该点的代价值
}

// 判断代价值是否合法（非致命障碍、非内切障碍、非未知）
bool BaseObstacleCritic::isValidCost(const unsigned char cost)
{
  return cost != nav2_costmap_2d::LETHAL_OBSTACLE &&
         cost != nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
         cost != nav2_costmap_2d::NO_INFORMATION;
}

// 生成 critic 的可视化通道：把整幅代价地图作为浮点网格导出
void BaseObstacleCritic::addCriticVisualization(
  std::vector<std::pair<std::string, std::vector<float>>> & cost_channels)
{
  std::pair<std::string, std::vector<float>> grid_scores;
  grid_scores.first = name_;  // 通道名 = critic 名

  // 逐行展开代价地图为长度 size_x*size_y 的数组
  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  grid_scores.second.resize(size_x * size_y);
  unsigned int i = 0;
  for (unsigned int cy = 0; cy < size_y; cy++) {
    for (unsigned int cx = 0; cx < size_x; cx++) {
      grid_scores.second[i] = costmap_->getCost(cx, cy);
      i++;
    }
  }
  cost_channels.push_back(grid_scores);
}

}  // namespace dwb_critics
