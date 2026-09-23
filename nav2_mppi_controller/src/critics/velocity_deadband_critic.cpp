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

#include "nav2_mppi_controller/critics/velocity_deadband_critic.hpp"

namespace mppi::critics
{

// 中文说明（文件级）：
// 速度死区（Velocity Deadband）critic：惩罚落在死区范围内的速度指令。
// 电机存在死区：低于某阈值的速度指令无法驱动电机，只会发热不做功。
// 本 critic 对 |v| < deadband 的采样施加惩罚，促使优化器选择
// 要么静止、要么输出足够大以克服死区的速度指令。

// 初始化：读取参数
void VelocityDeadbandCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);  // 代价幂次
  getParam(weight_, "cost_weight", 35.0);  // 权重（默认很大，强惩罚死区速度）

  // Recast double to float
  //（读取 [vx, vy, wz] 三轴死区阈值并转为 float 存储）
  std::vector<double> deadband_velocities{0.0, 0.0, 0.0};
  getParam(deadband_velocities, "deadband_velocities", std::vector<double>{0.0, 0.0, 0.0});
  std::transform(
    deadband_velocities.begin(), deadband_velocities.end(), deadband_velocities_.begin(),
    [](double d) {return static_cast<float>(d);});

  RCLCPP_INFO_STREAM(
    logger_, "VelocityDeadbandCritic instantiated with "
      << power_ << " power, " << weight_ << " weight, deadband_velocity ["
      << deadband_velocities_.at(0) << "," << deadband_velocities_.at(1) << ","
      << deadband_velocities_.at(2) << "]");
}

// 评分：惩罚落在死区内（|v| < deadband）的速度分量
void VelocityDeadbandCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;

  if (!enabled_) {
    return;  // 已禁用则跳过
  }

  auto & vx = data.state.vx;
  auto & wz = data.state.wz;

  if (data.motion_model->isHolonomic()) {
    // 全向底盘：检查 vx、vy、wz 三轴死区
    auto & vy = data.state.vy;
    if (power_ > 1u) {
      // 死区缺口 = max(deadband - |v|, 0)：速度离死区边界还差多少
      data.costs += xt::pow(
        xt::sum(
          std::move(
            xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0) +
            xt::maximum(fabs(deadband_velocities_.at(1)) - xt::fabs(vy), 0) +
            xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0)) *
          data.model_dt,
          {1}, immediate) *
        weight_,
        power_);
    } else {
      // power == 1 时省略 pow 调用（小优化）
      data.costs += xt::sum(
        (std::move(
          xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0) +
          xt::maximum(fabs(deadband_velocities_.at(1)) - xt::fabs(vy), 0) +
          xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0))) *
        data.model_dt,
        {1}, immediate) *
        weight_;
    }
    return;
  }

  // 非全向底盘：仅检查 vx、wz 两轴
  if (power_ > 1u) {
    data.costs += xt::pow(
      xt::sum(
        std::move(
          xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0) +
          xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0)) *
        data.model_dt,
        {1}, immediate) *
      weight_,
      power_);
  } else {
    data.costs += xt::sum(
      (std::move(
        xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0) +
        xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0))) *
      data.model_dt,
      {1}, immediate) *
      weight_;
  }
  return;
}

}  // namespace mppi::critics

// pluginlib 导出为 critic 插件
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::VelocityDeadbandCritic, mppi::critics::CriticFunction)
