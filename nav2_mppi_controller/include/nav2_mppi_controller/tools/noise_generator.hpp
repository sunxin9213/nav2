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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "nav2_mppi_controller/models/optimizer_settings.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"

namespace mppi
{

/**
 * @class mppi::NoiseGenerator
 * @brief Generates noise trajectories from optimal trajectory
 *（噪声生成器：在上周期最优控制序列基础上叠加高斯噪声，批量采样候选控制）
 *
 * 中文说明：
 * MPPI 的"探索"来自随机采样：以当前最优控制序列为均值、以
 * sampling_std 为标准差生成 [batch_size, time_steps] 的高斯噪声矩阵。
 * 为不阻塞控制循环，噪声的生成放在独立线程（noiseThread）中异步进行：
 * 主线程在计算本周期控制量的同时，噪声线程已在为下一周期准备新噪声，
 * 二者通过条件变量同步。注意：若在 x 轴正方向噪声生成速度过慢导致
 * ready_ 未置位，主线程会复用上一次的噪声（详见 setNoisedControls）。
 */
class NoiseGenerator
{
public:
  /**
    * @brief Constructor for mppi::NoiseGenerator
    */
  NoiseGenerator() = default;

  /**
   * @brief Initialize noise generator with settings and model types
   * @param settings Settings of controller
   * @param is_holonomic If base is holonomic
   * @param name Namespace for configs
   * @param param_handler Get parameters util
   */
  void initialize(
    mppi::models::OptimizerSettings & settings,
    bool is_holonomic, const std::string & name, ParametersHandler * param_handler);

  /**
   * @brief Shutdown noise generator thread
   */
  void shutdown();

  /**
   * @brief Signal to the noise thread the controller is ready to generate a new
   * noised control for the next iteration
   */
  void generateNextNoises();

  /**
   * @brief set noised control_sequence to state controls
   * @return noises vx, vy, wz
   */
  void setNoisedControls(models::State & state, const models::ControlSequence & control_sequence);

  /**
   * @brief Reset noise generator with settings and model types
   * @param settings Settings of controller
   * @param is_holonomic If base is holonomic
   */
  void reset(mppi::models::OptimizerSettings & settings, bool is_holonomic);

protected:
  /**
   * @brief Thread to execute noise generation process
   */
  void noiseThread();

  /**
   * @brief Generate random controls by gaussian noise with mean in
   * control_sequence_
   *
   * @return tensor of shape [ batch_size_, time_steps_, 2]
   * where 2 stands for v, w
   */
  void generateNoisedControls();

  // 各控制分量的噪声张量 [batch_size, time_steps]：noised = control + noise
  xt::xtensor<float, 2> noises_vx_;  // x 方向线速度噪声
  xt::xtensor<float, 2> noises_vy_;  // y 方向线速度噪声（仅全向底盘使用）
  xt::xtensor<float, 2> noises_wz_;  // 角速度噪声

  mppi::models::OptimizerSettings settings_;  // 优化器设置（含采样标准差）
  bool is_holonomic_;                         // 底盘是否全向（决定是否生成 vy 噪声）

  std::thread noise_thread_;            // 异步噪声生成线程
  std::condition_variable noise_cond_;  // 与主线程同步的条件变量
  std::mutex noise_lock_;               // 保护噪声数据的互斥锁
  bool active_{false}, ready_{false}, regenerate_noises_{false};
  // active：线程是否存活；ready：新噪声是否就绪；regenerate_noises：是否需要重新生成噪声的标志
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_
