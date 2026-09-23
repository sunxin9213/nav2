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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for aligning to the path. Note:
 * High settings of this will follow the path more precisely, but also makes it
 * difficult (or impossible) to deviate in the presence of dynamic obstacles.
 * This is an important critic to tune and consider in tandem with Obstacle.
 * （路径对齐评价器：惩罚轨迹侧向偏离路径的分量（垂直于路径方向的距离），
 *  权重过高会贴死路径而无法绕开动态障碍物，需与 Obstacles 权重权衡。）
 */
class PathAlignCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;  // 读取权重/偏移/占用比例阈值等参数

  /**
   * @brief Evaluate cost related to trajectories path alignment
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData & data) override;  // 基于弧长匹配计算侧向偏差代价

protected:
  size_t offset_from_furthest_{0};      // 相对轨迹可达最远路径点的索引偏移
  int trajectory_point_step_{0};         // 轨迹点采样步长（跳点打分提速）
  float threshold_to_consider_{0};       // 该横向偏差内不惩罚（死区）
  float max_path_occupancy_ratio_{0};    // 路径被障碍占用的比例上限（超过则绕行、放宽对齐）
  bool use_path_orientations_{false};   // 是否用路径点自身朝向（否则用切向）
  unsigned int power_{0};               // 代价幂次
  float weight_{0};                     // 评价器权重
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_
