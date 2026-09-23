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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_

#include <cstddef>
#include "nav2_mppi_controller/models/constraints.hpp"

namespace mppi::models
{

/**
 * @struct mppi::models::OptimizerSettings
 * @brief Settings for the optimizer to use
 *（优化器配置：MPPI 算法的全部超参数集合）
 *
 * 中文说明——各参数在 MPPI 中的作用：
 * - batch_size：每个周期并行采样的噪声轨迹数量（如 2000 条）；
 * - time_steps：预测时域的离散时间步数（如 56 步 * model_dt）；
 * - sampling_std：高斯噪声标准差，决定探索范围；
 * - temperature：软最大（softmax）温度，控制轨迹权重的尖锐程度
 *   （值越小越接近硬选择最优轨迹，值越大权重越平滑）；
 * - gamma：折扣因子，用于加权平均时强调近期代价；
 * - iteration_count：单个控制周期内的优化迭代次数；
 * - shift_control_sequence：是否在周期结束时将控制序列向前滚动一步。
 */
struct OptimizerSettings
{
  models::ControlConstraints base_constraints{0, 0, 0, 0};  // 基础控制约束（参数原值，未受速度限制影响）
  models::ControlConstraints constraints{0, 0, 0, 0};      // 当前生效的控制约束（可能被 setSpeedLimit 缩放）
  models::SamplingStd sampling_std{0, 0, 0};                // 噪声采样标准差
  float model_dt{0};        // 轨迹推演的时间步长（秒）
  float temperature{0};    // softmax 温度系数
  float gamma{0};           // 代价折扣/加权因子
  unsigned int batch_size{0};       // 采样轨迹批次大小
  unsigned int time_steps{0};       // 预测时间步数
  unsigned int iteration_count{0};  // 每周期优化迭代次数
  bool shift_control_sequence{false};  // 是否启用控制序列滚动
  size_t retry_attempt_limit{0};       // 优化失败（如无有效轨迹）时的重试次数上限
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_
