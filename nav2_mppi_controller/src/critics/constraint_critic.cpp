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

#include "nav2_mppi_controller/critics/constraint_critic.hpp"

namespace mppi::critics
{

// 中文说明（文件级）：
// 约束 critic：惩罚超出速度上下限（以及阿克曼最小转弯半径）的轨迹点。
// 虽然优化器在更新后会硬裁剪控制序列，但该 critic 在评分阶段即对
// 超限采样施加惩罚，使 softmax 加权结果天然趋向可行域。

// 初始化：读取权重与父控制器的速度上下限参数
void ConstraintCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);

  getParam(power_, "cost_power", 1);  // 代价幂次
  getParam(weight_, "cost_weight", 4.0);  // 权重
  RCLCPP_INFO(
    logger_, "ConstraintCritic instantiated with %d power and %f weight.",
    power_, weight_);

  // 从父控制器（MPPI 优化器）读取速度限值
  float vx_max, vy_max, vx_min;
  getParentParam(vx_max, "vx_max", 0.5);
  getParentParam(vy_max, "vy_max", 0.0);
  getParentParam(vx_min, "vx_min", -0.35);

  // 计算合成速度的上下限（把 vx/vy 合成为总速度模长，保留倒车符号）
  const float min_sgn = vx_min > 0.0 ? 1.0 : -1.0;
  max_vel_ = sqrtf(vx_max * vx_max + vy_max * vy_max);
  min_vel_ = min_sgn * sqrtf(vx_min * vx_min + vy_max * vy_max);
}

// 评分：惩罚超出速度界限的轨迹点
void ConstraintCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;

  if (!enabled_) {
    return;  // 已禁用则跳过
  }

  // 带符号的合成速度模长：|v| = sqrt(vx² + vy²)，符号取 vx 符号（区分前进/倒车）
  auto sgn = xt::where(data.state.vx > 0.0, 1.0, -1.0);
  auto vel_total = sgn * xt::sqrt(data.state.vx * data.state.vx + data.state.vy * data.state.vy);
  // 超上限的量：max(vel - max_vel, 0)，仅超限部分被惩罚
  auto out_of_max_bounds_motion = xt::maximum(vel_total - max_vel_, 0);
  // 低于下限的量：max(min_vel - vel, 0)
  auto out_of_min_bounds_motion = xt::maximum(min_vel_ - vel_total, 0);

  // 阿克曼运动模型：额外惩罚转弯半径小于最小转弯半径的点
  auto acker = dynamic_cast<AckermannMotionModel *>(data.motion_model.get());
  if (acker != nullptr) {
    auto & vx = data.state.vx;
    auto & wz = data.state.wz;
    // 转弯半径 r = |vx / wz|，不足最小转弯半径的部分被惩罚
    auto out_of_turning_rad_motion = xt::maximum(
      acker->getMinTurningRadius() - (xt::fabs(vx) / xt::fabs(wz)), 0.0);

    // 三项超限量之和按时间积分（乘 model_dt），沿轨迹累加后加权、取幂
    data.costs += xt::pow(
      xt::sum(
        (std::move(out_of_max_bounds_motion) +
        std::move(out_of_min_bounds_motion) +
        std::move(out_of_turning_rad_motion)) *
        data.model_dt, {1}, immediate) * weight_, power_);
    return;
  }

  // 差速/全向底盘：仅速度上下限两项
  data.costs += xt::pow(
    xt::sum(
      (std::move(out_of_max_bounds_motion) +
      std::move(out_of_min_bounds_motion)) *
      data.model_dt, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

// pluginlib 导出为 critic 插件
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::ConstraintCritic, mppi::critics::CriticFunction)
