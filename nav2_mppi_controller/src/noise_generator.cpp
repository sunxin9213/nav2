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

#include "nav2_mppi_controller/tools/noise_generator.hpp"

#include <memory>
#include <mutex>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

namespace mppi
{

// 初始化噪声生成器：读取参数并决定是否启用异步噪声线程
void NoiseGenerator::initialize(
  mppi::models::OptimizerSettings & settings, bool is_holonomic,
  const std::string & name, ParametersHandler * param_handler)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;
  active_ = true;  // 标记生成器处于活跃状态

  auto getParam = param_handler->getParamGetter(name);
  // regenerate_noises：是否每周期重新生成噪声（异步线程模式）。
  // false 时只在初始化/重置时生成一次噪声并长期复用（计算更快，
  // 但采样分布固定；true 时更接近理论上的随机采样）
  getParam(regenerate_noises_, "regenerate_noises", false);

  if (regenerate_noises_) {
    // 启动后台线程：每周期为下一轮迭代准备新噪声
    noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
  } else {
    // 静态模式：立即生成一批噪声并复用
    generateNoisedControls();
  }
}

// 关闭：唤醒并回收噪声线程（保证进程退出时线程安全结束）
void NoiseGenerator::shutdown()
{
  active_ = false;   // 让线程循环退出
  ready_ = true;     // 置位以唤醒可能阻塞在 wait 的线程
  noise_cond_.notify_all();
  if (noise_thread_.joinable()) {
    noise_thread_.join();  // 等待线程结束
  }
}

// 通知噪声线程开始为下一次迭代生成噪声
void NoiseGenerator::generateNextNoises()
{
  // Trigger the thread to run in parallel to this iteration
  // to generate the next iteration's noises (if applicable).
  //（触发线程与本轮迭代并行执行，为下一轮迭代生成噪声）
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    ready_ = true;  // 置位就绪标志，线程从 wait 中醒来
  }
  noise_cond_.notify_all();
}

// 将噪声叠加到名义控制序列上，写入 state 的控制张量（cvx/cvy/cwz）
void NoiseGenerator::setNoisedControls(
  models::State & state,
  const models::ControlSequence & control_sequence)
{
  // 加锁保护：若噪声线程正在写入噪声张量，此处需等待
  std::unique_lock<std::mutex> guard(noise_lock_);

  // 噪声控制 = 当前最优控制序列 + 高斯噪声（逐元素相加）
  xt::noalias(state.cvx) = control_sequence.vx + noises_vx_;
  xt::noalias(state.cvy) = control_sequence.vy + noises_vy_;
  xt::noalias(state.cwz) = control_sequence.wz + noises_wz_;
}

// 重置：清零噪声张量并按模式重新生成（初始化/参数变更/回退时调用）
void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;

  // Recompute the noises on reset, initialization, and fallback
  //（在重置/初始化/失败回退时重新计算噪声）
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    // 先清零，避免旧尺寸数据残留（batch/time_steps 可能已变化）
    xt::noalias(noises_vx_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(noises_vy_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(noises_wz_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    ready_ = true;
  }

  if (regenerate_noises_) {
    // 异步模式：唤醒后台线程生成新噪声
    noise_cond_.notify_all();
  } else {
    // 静态模式：直接在调用线程生成
    generateNoisedControls();
  }
}

// 噪声生成线程主循环：等待就绪信号 → 生成噪声，直到被关闭
void NoiseGenerator::noiseThread()
{
  do {
    std::unique_lock<std::mutex> guard(noise_lock_);
    // 阻塞等待 ready_ 置位（由 generateNextNoises / shutdown 触发），
    // 谓词条件避免虚假唤醒
    noise_cond_.wait(guard, [this]() {return ready_;});
    ready_ = false;  // 消费信号，避免重复生成
    generateNoisedControls();
  } while (active_);
}

// 生成高斯白噪声张量：均值 0、标准差为各控制分量的 sampling_std
void NoiseGenerator::generateNoisedControls()
{
  auto & s = settings_;

  // 为 vx、wz 生成 [batch_size, time_steps] 的正态分布随机张量
  xt::noalias(noises_vx_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.vx);
  xt::noalias(noises_wz_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.wz);
  if (is_holonomic_) {
    // 仅全向底盘需要 vy 噪声
    xt::noalias(noises_vy_) = xt::random::randn<float>(
      {s.batch_size, s.time_steps}, 0.0f,
      s.sampling_std.vy);
  }
}

}  // namespace mppi
