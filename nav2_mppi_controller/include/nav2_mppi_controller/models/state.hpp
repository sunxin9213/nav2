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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__STATE_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__STATE_HPP_

#include <xtensor/xtensor.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace mppi::models
{

/**
 * @struct mppi::models::State
 * @brief State information: velocities, controls, poses, speed
 *（状态模型：优化器内部使用的批量状态张量与机器人当前位姿/速度）
 *
 * 中文说明：
 * MPPI（模型预测路径积分控制）在每个控制周期会并行采样 batch_size 条
 * 噪声扰动轨迹，本结构体以二维张量 [batch_size, time_steps] 存储这些
 * 轨迹对应的速度与控制量，是 rollout（轨迹推演）的核心数据容器。
 */
struct State
{
  // 采样轨迹的线速度 x 分量：行=轨迹批次，列=预测时间步
  xt::xtensor<float, 2> vx;
  // 采样轨迹的线速度 y 分量（全向轮机器人非零，差速/阿克曼为 0）
  xt::xtensor<float, 2> vy;
  // 采样轨迹的角速度（绕 z 轴）
  xt::xtensor<float, 2> wz;

  // 施加噪声前的名义控制序列拷贝（control sequence + noise 的基础）
  xt::xtensor<float, 2> cvx;
  xt::xtensor<float, 2> cvy;
  xt::xtensor<float, 2> cwz;

  geometry_msgs::msg::PoseStamped pose;  // 机器人当前位姿（rollout 的起始位姿）
  geometry_msgs::msg::Twist speed;      // 机器人当前速度（来自里程计）

  /**
    * @brief Reset state data
    *（重置状态：按批次大小与时间步数分配并清零所有张量）
    */
  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    // 为每种速度/控制分量分配 batch_size x time_steps 的零张量
    vx = xt::zeros<float>({batch_size, time_steps});
    vy = xt::zeros<float>({batch_size, time_steps});
    wz = xt::zeros<float>({batch_size, time_steps});

    cvx = xt::zeros<float>({batch_size, time_steps});
    cvy = xt::zeros<float>({batch_size, time_steps});
    cwz = xt::zeros<float>({batch_size, time_steps});
  }
};
}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__STATE_HPP_
