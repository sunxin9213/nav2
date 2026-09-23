// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_HPP_

#include <memory>
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for avoiding obstacles, allowing it to deviate off
 * the planned path. This is important to tune in tandem with PathAlign to make a balance
 * between path-tracking and dynamic obstacle avoidance capabilities as desirable for a
 * particular application
 * （障碍物评价器：基于"到障碍物的距离"做斥力型惩罚（由膨胀代价反解），
 *  允许轨迹偏离参考路径绕开障碍物。需与 PathAlign 权重配合调节，
 *  在"跟得紧"与"躲得开"之间取得平衡。）
 */
class ObstaclesCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;  // 读取权重/斥力参数/膨胀层参数并初始化碰撞检测器

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
  void score(CriticData & data) override;  // 逐轨迹点计算距离惩罚（近障碍大，远障碍衰减为 0）

protected:
  /**
    * @brief Checks if cost represents a collision
    * @param cost Costmap cost
    * @return bool if in collision
    */
  // 代价值达到碰撞阈值即视为碰撞
  inline bool inCollision(float cost) const;

  /**
    * @brief cost at a robot pose
    * @param x X of pose
    * @param y Y of pose
    * @param theta theta of pose
    * @return Collision information at pose
    */
  // 查询位姿处的碰撞信息（含足迹检测分支）
  inline CollisionCost costAtPose(float x, float y, float theta);

  /**
    * @brief Distance to obstacle from cost
    * @param cost Costmap cost
    * @return float Distance to the obstacle represented by cost
    */
  // 由膨胀代价值反解到障碍物的距离（利用指数衰减函数的逆映射）
  inline float distanceToObstacle(const CollisionCost & cost);

  /**
    * @brief Find the min cost of the inflation decay function for which the robot MAY be
    * in collision in any orientation
    * @param costmap Costmap2DROS to get minimum inscribed cost (e.g. 128 in inflation layer documentation)
    * @return double circumscribed cost, any higher than this and need to do full footprint collision checking
    * since some element of the robot could be in collision
    */
  // 反解外接圆代价：中心代价超过该值时必须做完整足迹碰撞检测
  float findCircumscribedCost(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap);

protected:
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
  collision_checker_{nullptr};  // 足迹碰撞检测器

  bool consider_footprint_{true};       // 是否使用完整足迹
  float collision_cost_{0};            // 判定碰撞的代价值
  float inflation_scale_factor_{0}, inflation_radius_{0};  // 膨胀层指数衰减系数与半径

  float possibly_inscribed_cost_;       // 内切代价阈值
  float collision_margin_distance_;    // 碰撞安全裕量（额外留出的距离）
  float near_goal_distance_;           // 距目标该距离内停止惩罚（避免终点附近卡死）
  float circumscribed_cost_{0}, circumscribed_radius_{0};  // 外接圆代价与半径

  unsigned int power_{0};              // 代价幂次
  float repulsion_weight_, critical_weight_{0};  // 斥力权重与临界碰撞权重
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_HPP_
