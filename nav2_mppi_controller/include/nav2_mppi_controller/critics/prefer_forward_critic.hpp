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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for preferring forward motion
 * （前进偏好评价器：惩罚轨迹中的负向线速度（倒车），
 *  使机器人在无必要时保持前进。）
 */
class PreferForwardCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;  // 读取权重/幂次/死区阈值参数

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  void score(CriticData & data) override;  // 累加 vx < 0 部分的惩罚

protected:
  unsigned int power_{0};            // 代价幂次
  float weight_{0};                  // 评价器权重
  float threshold_to_consider_{0};   // 倒车速度小于该绝对值时不惩罚（死区）
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_
