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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_

namespace mppi::models
{

/**
 * @struct mppi::models::ControlConstraints
 * @brief Constraints on control
 *（控制约束：速度采样的上下限，用于裁剪控制序列）
 */
struct ControlConstraints
{
  float vx_max;  // 最大前向线速度（m/s）
  float vx_min;  // 最小前向线速度（可为负值，表示允许倒车）
  float vy;     // 横向线速度上限（全向机器人使用，否则为 0）
  float wz;     // 角速度上限（rad/s）
};

/**
 * @struct mppi::models::SamplingStd
 * @brief Noise parameters for sampling trajectories
 *（采样噪声标准差：MPPI 通过向名义控制序列叠加该标准差的高斯白噪声来批量采样候选轨迹）
 */
struct SamplingStd
{
  float vx;  // x 方向线速度噪声标准差
  float vy;  // y 方向线速度噪声标准差
  float wz;  // 角速度噪声标准差
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_
