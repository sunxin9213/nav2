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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

namespace mppi::models
{

/**
 * @class mppi::models::Trajectories
 * @brief Candidate Trajectories
 *（候选轨迹模型：rollout 推演得到的批量轨迹位姿）
 *
 * 中文说明：
 * 由噪声控制序列经运动模型（运动学/运动动力学）前向积分得到，
 * 每行对应一条候选轨迹，每列为一个预测时间步的位姿。
 * 这些轨迹随后被各 critic（代价函数）评分，再经 softmax 加权平均
 * 得到最终更新的控制序列。
 */
struct Trajectories
{
  xt::xtensor<float, 2> x;     // 各轨迹各时间步的 x 坐标 [batch_size, time_steps]
  xt::xtensor<float, 2> y;     // 各轨迹各时间步的 y 坐标
  xt::xtensor<float, 2> yaws;  // 各轨迹各时间步的朝向角（rad）

  /**
    * @brief Reset state data
    *（重置轨迹数据：按批次大小与时间步数分配并清零）
    */
  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    x = xt::zeros<float>({batch_size, time_steps});
    y = xt::zeros<float>({batch_size, time_steps});
    yaws = xt::zeros<float>({batch_size, time_steps});
  }
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_
