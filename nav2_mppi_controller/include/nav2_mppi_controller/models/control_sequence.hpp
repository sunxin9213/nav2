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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__CONTROL_SEQUENCE_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__CONTROL_SEQUENCE_HPP_

#include <xtensor/xtensor.hpp>

namespace mppi::models
{

/**
 * @struct mppi::models::Control
 * @brief A set of controls
 *（单个控制量：某一时刻的速度指令三元组）
 */
struct Control
{
  float vx, vy, wz;  // x/y 方向线速度与角速度
};

/**
 * @struct mppi::models::ControlSequence
 * @brief A control sequence over time (e.g. trajectory)
 *（控制序列：MPPI 优化的核心变量，即未来 time_steps 个时间步的控制指令）
 *
 * 中文说明：
 * MPPI 每个周期对该序列进行加权更新，并只输出第一个时间步的控制量
 * （滚动时域优化）。下一周期通过 shift 操作将序列向前滚动一步，
 * 实现控制序列的跨周期延续。
 */
struct ControlSequence
{
  xt::xtensor<float, 1> vx;  // 各时间步的 x 方向线速度指令
  xt::xtensor<float, 1> vy;  // 各时间步的 y 方向线速度指令
  xt::xtensor<float, 1> wz;  // 各时间步的角速度指令

  // 重置控制序列：按时间步数分配并清零
  void reset(unsigned int time_steps)
  {
    vx = xt::zeros<float>({time_steps});
    vy = xt::zeros<float>({time_steps});
    wz = xt::zeros<float>({time_steps});
  }
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__CONTROL_SEQUENCE_HPP_
