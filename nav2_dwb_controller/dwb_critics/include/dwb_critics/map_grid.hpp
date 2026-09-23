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

#ifndef DWB_CRITICS__MAP_GRID_HPP_
#define DWB_CRITICS__MAP_GRID_HPP_

#include <vector>
#include <memory>
#include <string>
#include <utility>

#include "dwb_core/trajectory_critic.hpp"
#include "costmap_queue/costmap_queue.hpp"

// ============================================================================
// 模块说明：地图栅格评价器（MapGridCritic）
// ----------------------------------------------------------------------------
// DWB 中最重要的评价器基类，为 GoalDistCritic、PathDistCritic 及其派生类
// （GoalAlign、PathAlign 等）提供底层支撑。
// 核心算法：广度优先搜索（BFS）距离场 / 波前扩散（wavefront propagation）：
//   1. prepare 阶段：由派生类确定"源点"集合（如全局路径点、临时目标点），
//      将对应栅格标记为初始分值（通常为 0）；
//   2. 从源点开始 BFS 扩散（借助 costmap_queue 包的 CostmapQueue 实现，
//      使用曼哈顿距离作为优先级），逐格填充 cell_values_ 数组：
//      每个格的分值 = 到最近源点的距离（障碍物格被跳过）；
//   3. scoreTrajectory 阶段：查询轨迹各位姿所在格的 cell_values_ 值即可，
//      O(1) 查表，无需逐轨迹比较路径点。
// 设计动机（计算效率）：一次性预计算整个距离场（O(地图大小)），
// 之后每条候选轨迹打分只需 O(轨迹长度) 次查表，远快于对每条轨迹
// 单独计算到源点集合距离的 O(轨迹长度 x 源点数)。
// 聚合方式（aggregationType_）：轨迹总分 = 所有位姿分值的 Last/Sum/Product。
// ============================================================================

namespace dwb_critics
{
/**
 * @class MapGridCritic
 * @brief breadth-first scoring of all the cells in the costmap
 *
 * This TrajectoryCritic assigns a score to every cell in the costmap based on
 * the distance to the cell from some set of source points. The cells corresponding
 * with the source points are marked with some initial score, and then every other cell
 * is updated with a score based on its relation to the closest source cell, based on a
 * breadth-first exploration of the cells of the costmap.
 *
 * This approach was chosen for computational efficiency, such that each trajectory
 * need not be compared to the list of source points.
 */
// 地图栅格评价器：用 BFS 距离场为全图每个格子预计算到源点集合的距离
class MapGridCritic : public dwb_core::TrajectoryCritic
{
public:
  // Standard TrajectoryCritic Interface
  // 初始化：读取 aggregation_type / obstacle_score / unreachable_score 等参数
  void onInit() override;
  // 打分：遍历轨迹所有位姿查 cell_values_ 距离场，按聚合方式（Last/Sum/Product）合并
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  // 可视化：输出原始距离场 channel，便于在 rviz 中调试
  void addCriticVisualization(
    std::vector<std::pair<std::string, std::vector<float>>> & cost_channels) override;
  // 重写 getScale：由于 cell_values_ 的单位是"格"（cell），乘以地图分辨率的一半
  // 转换为米（BFS 曼哈顿距离近似欧氏距离的斜边一半），再乘用户设定的 scale_
  double getScale() const override {return costmap_->getResolution() * 0.5 * scale_;}

  // Helper Functions
  /**
   * @brief Retrieve the score for a single pose
   * @param pose The pose to score, assumed to be in the same frame as the costmap
   * @return The score associated with the cell of the costmap where the pose lies
   */
  // 查询单个位姿的分数：将世界坐标转换为栅格坐标后查 cell_values_ 距离场
  virtual double scorePose(const geometry_msgs::msg::Pose2D & pose);

  /**
   * @brief Retrieve the score for a particular cell of the costmap
   * @param x x-coordinate within the costmap
   * @param y y-coordinate within the costmap
   * @return the score associated with that cell.
   */
  // 按栅格坐标查询分数：costmap_->getIndex 把 (x,y) 转为一维索引
  inline double getScore(unsigned int x, unsigned int y)
  {
    return cell_values_[costmap_->getIndex(x, y)];
  }

  /**
   * @brief Sets the score of a particular cell to the obstacle cost
   * @param index Index of the cell to mark
   */
  // 将某格标记为障碍物：其值被设为 obstacle_score_（负大数，表示不可通行），
  // BFS 扩散时遇到该格不会继续传播（见 MapGridQueue::validCellToQueue）
  void setAsObstacle(unsigned int index);

protected:
  /**
   * @brief Separate modes for aggregating scores across the multiple poses in a trajectory.
   *
   * Last returns the score associated with the last pose in the trajectory
   * Sum returns the sum of all the scores
   * Product returns the product of all the (non-zero) scores
   */
  // 轨迹分数聚合方式：Last=只取末位姿分值；Sum=求和；Product=连乘
  // cppcheck-suppress syntaxError
  enum class ScoreAggregationType {Last, Sum, Product};

  /**
   * @class MapGridQueue
   * @brief Subclass of CostmapQueue that avoids Obstacles and Unknown Values
   */
  // 定制化的代价地图 BFS 队列：跳过障碍物格与未知格，扩散不穿过障碍物
  class MapGridQueue : public costmap_queue::CostmapQueue
  {
public:
    // 构造：manhattan=true 使用曼哈顿距离作为队列优先级
    MapGridQueue(nav2_costmap_2d::Costmap2D & costmap, MapGridCritic & parent)
    : costmap_queue::CostmapQueue(costmap, true), parent_(parent) {}
    virtual ~MapGridQueue() = default;
    // 只把"可通过"的格子（调用父评价器的 isValidCost）加入扩散队列
    bool validCellToQueue(const costmap_queue::CellData & cell) override;

protected:
    MapGridCritic & parent_;  // 反向引用宿主评价器（用于查询代价值）
  };

  /**
   * @brief Clear the queuDWB_CRITICS_MAP_GRID_He and set cell_values_ to the appropriate number of unreachableCellScore
   */
  // 重置：清空 BFS 队列，将 cell_values_ 全部初始化为 unreachable_score_（未达值）
  void reset() override;

  /**
   * @brief Go through the queue and set the cells to the Manhattan distance from their parents
   */
  // 核心：从源点出发做 BFS 扩散，按曼哈顿距离逐格填充 cell_values_
  void propogateManhattanDistances();

  std::shared_ptr<MapGridQueue> queue_;   // BFS 扩散队列（每次 reset 后重建）
  nav2_costmap_2d::Costmap2D * costmap_;  // 底层代价地图（借用指针）
  std::vector<double> cell_values_;       // 距离场数组（按 costmap 一维索引）
  double obstacle_score_, unreachable_score_;  ///< Special cell_values（障碍分值/不可达分值）
  bool stop_on_failure_;                   // 若轨迹终点落在障碍格上是否直接判非法
  ScoreAggregationType aggregationType_;   // 轨迹分数聚合方式
};
}  // namespace dwb_critics

#endif  // DWB_CRITICS__MAP_GRID_HPP_
