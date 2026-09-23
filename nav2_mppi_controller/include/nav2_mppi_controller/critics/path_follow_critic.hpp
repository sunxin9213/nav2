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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_FOLLOW_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_FOLLOW_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for following the path approximately
 * To allow for deviation from path in case of dynamic obstacles. Path Align
 * is what aligns the trajectories to the path more or less precisely, if desireable.
 * A higher weight here with an offset > 1 will accelerate the samples to full speed
 * faster and push the follow point further ahead, creating some shortcutting.
 * （路径跟随评价器：惩罚轨迹到"前瞻路径点"的距离（而非逐点贴齐），
 *  允许为躲避动态障碍物而偏离路径。offset 越大前瞻点越远，
 *  会加快提速并产生适当"抄近路"行为。）
 */
class PathFollowCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;  // 读取权重/生效阈值/前瞻偏移参数

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  void score(CriticData & data) override;  // 计算轨迹到前瞻目标的距离代价

protected:
  float threshold_to_consider_{0};    // 与路径横向偏差小于该值时不惩罚
  size_t offset_from_furthest_{0};    // 前瞻偏移：在可达最远点之上再前进的点数

  unsigned int power_{0};             // 代价幂次
  float weight_{0};                   // 评价器权重
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_FOLLOW_CRITIC_HPP_
