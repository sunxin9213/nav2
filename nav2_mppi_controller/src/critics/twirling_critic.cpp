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

#include "nav2_mppi_controller/critics/twirling_critic.hpp"

namespace mppi::critics
{

// 中文说明（文件级）：
// 旋转（Twirling）critic：仅用于全向（Omni）底盘，
// 惩罚轨迹中绕 z 轴的自旋角速度，防止机器人在平移过程中
// 不断原地旋转（“twirling”现象）。差速底盘因无 vy 自由度
// 通常不需要此 critic。

// 初始化：读取参数
void TwirlingCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);  // 代价幂次
  getParam(weight_, "cost_weight", 10.0);  // 权重（默认较大，强抑制自旋）

  RCLCPP_INFO(
    logger_, "TwirlingCritic instantiated with %d power and %f weight.", power_, weight_);
}

// 评分：惩罚轨迹的平均角速度绝对值
void TwirlingCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;
  // 未启用或已到达目标（按 goal_checker 判定）时跳过
  if (!enabled_ ||
    utils::withinPositionGoalTolerance(data.goal_checker, data.state.pose.pose, data.path))
  {
    return;
  }

  // 角速度绝对值：沿轨迹取平均，惩罚持续旋转的轨迹
  const auto wz = xt::abs(data.state.wz);
  data.costs += xt::pow(xt::mean(wz, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

// pluginlib 导出为 critic 插件
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::TwirlingCritic,
  mppi::critics::CriticFunction)
