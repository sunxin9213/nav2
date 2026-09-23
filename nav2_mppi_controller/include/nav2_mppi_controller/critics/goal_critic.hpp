// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__GOAL_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__GOAL_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for driving towards goal
 * （目标评价器：惩罚轨迹终点到当前局部目标点的距离，
 *  引导采样轨迹收敛到目标。仅在距目标 threshold_to_consider_ 内生效。）
 */
class GoalCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;  // 读取权重/幂次/生效距离阈值参数

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData & data) override;  // 计算轨迹终点到目标的欧氏距离平方

protected:
  unsigned int power_{0};       // 代价幂次
  float weight_{0};            // 评价器权重
  float threshold_to_consider_{0};  // 距目标该距离内才计入此代价（远处由其他评价器主导）
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__GOAL_CRITIC_HPP_
