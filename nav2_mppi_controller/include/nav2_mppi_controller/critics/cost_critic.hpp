// Copyright (c) 2023 Robocc Brice Renaudeau
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::CostCritic
 * @brief Critic objective function for avoiding obstacles using costmap's inflated cost
 * （代价评价器：直接以代价地图的膨胀代价值作为轨迹点代价，
 *  在保持跟踪参考的同时远离高代价格（如障碍物附近）。
 *  当代价值超过外接圆代价时启用完整足迹碰撞检测。）
 */
class CostCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;  // 读取权重/幂次/碰撞代价等参数并初始化碰撞检测器

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
  void score(CriticData & data) override;  // 累加轨迹点的代价地图代价值

protected:
  /**
    * @brief Checks if cost represents a collision
    * @param cost Point cost at pose center
    * @param x X of pose
    * @param y Y of pose
    * @param theta theta of pose
    * @return bool if in collision
    */
  // 判断是否碰撞：代价超高时（可能压到足迹）做完整足迹碰撞检测
  bool inCollision(float cost, float x, float y, float theta);

  /**
    * @brief cost at a robot pose
    * @param x X of pose
    * @param y Y of pose
    * @return Collision information at pose
    */
  // 查询位姿中心点的代价地图代价值
  float costAtPose(float x, float y);

  /**
    * @brief Find the min cost of the inflation decay function for which the robot MAY be
    * in collision in any orientation
    * @param costmap Costmap2DROS to get minimum inscribed cost (e.g. 128 in inflation layer documentation)
    * @return double circumscribed cost, any higher than this and need to do full footprint collision checking
    * since some element of the robot could be in collision
    */
  // 反解外接圆代价：机器人外接圆半径映射到的膨胀代价值。
  // 中心代价超过该值说明外接圆已进入膨胀区，必须做足迹级碰撞检测
  float findCircumscribedCost(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap);

protected:
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
  collision_checker_{nullptr};   // 足迹碰撞检测器（对代价地图操作）
  float possibly_inscribed_cost_;  // 内切代价阈值（超过即视为潜在碰撞）

  bool consider_footprint_{true};      // 是否使用完整足迹（否则按点/圆处理）
  float circumscribed_radius_{0};     // 机器人外接圆半径
  float circumscribed_cost_{0};       // 外接圆对应的膨胀代价值
  float collision_cost_{0};           // 判定碰撞的代价值
  float critical_cost_{0};            // 临界代价值（触发 fail_flag）
  float weight_{0};                   // 评价器权重

  float near_goal_distance_;          // 距目标该距离内放宽/改变打分逻辑
  std::string inflation_layer_name_;  // 膨胀层名称（用于查询其参数）

  unsigned int power_{0};             // 代价幂次
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_
