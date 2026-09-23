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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for enforcing feasible constraints
 * （约束评价器：惩罚超出机器人速度/加速度等运动学可行范围的采样轨迹，
 *  保证 MPPI 输出的控制序列物理可执行。）
 */
class ConstraintCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;  // 读取权重、幂次与速度上下限参数

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData & data) override;  // 逐轨迹统计超限速度的惩罚代价

  float getMaxVelConstraint() {return max_vel_;}  // 速度上限（供其他模块查询）
  float getMinVelConstraint() {return min_vel_;}  // 速度下限（如倒车限速）

protected:
  unsigned int power_{0};   // 代价幂次（放大对超限的惩罚）
  float weight_{0};        // 该评价器的整体权重
  float min_vel_;           // 最小线速度（负值为倒车下限）
  float max_vel_;           // 最大线速度
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_
