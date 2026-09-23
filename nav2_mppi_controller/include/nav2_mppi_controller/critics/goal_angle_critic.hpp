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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__GOAL_ANGLE_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__GOAL_ANGLE_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for driving towards goal orientation
 * （目标朝向评价器：惩罚轨迹末端的偏航角与目标朝向的偏差，
 *  使机器人到位后朝向正确。仅当机器人接近目标时生效。）
 */
class GoalAngleCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;  // 读取权重/幂次/生效阈值/对称容差参数

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  // 只在机器人接近目标（threshold_to_consider_ 内）时计算末端角度偏差代价
  void score(CriticData & data) override;

protected:
  bool symmetric_yaw_tolerance_{false};  // 对称角度容差：允许 ±pi 内双向达标
  float threshold_to_consider_{0};       // 距目标该距离内评价器才生效
  unsigned int power_{0};                // 代价幂次
  float weight_{0};                      // 评价器权重
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__GOAL_ANGLE_CRITIC_HPP_
