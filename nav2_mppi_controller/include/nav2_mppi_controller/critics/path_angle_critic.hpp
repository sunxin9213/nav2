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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_ANGLE_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_ANGLE_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for aligning to path in cases of extreme misalignment
 * or turning
 * （路径角度评价器：惩罚轨迹点朝向与"指向目标路径点方向"的夹角，
 *  用于极端失准（如刚接到新路径、需要大转向）时快速对准路径。）
 */
class PathAngleCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;  // 读取权重/角度阈值/倒车与前进偏好等参数

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  void score(CriticData & data) override;  // 计算朝向与目标方向的最短角差代价

protected:
  float max_angle_to_furthest_{0};   // 与最远路径点夹角超过该值时评价器才生效
  float threshold_to_consider_{0};   // 生效距离/角度阈值

  size_t offset_from_furthest_{0};   // 相对最远可达路径点的索引偏移（选打分参考点）
  bool reversing_allowed_{true};     // 是否允许倒车对准
  bool forward_preference_{true};    // 是否偏向前进方向对准

  unsigned int power_{0};            // 代价幂次
  float weight_{0};                  // 评价器权重
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_ANGLE_CRITIC_HPP_
