// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#include "nav2_mppi_controller/critics/goal_critic.hpp"

namespace mppi::critics
{

using xt::evaluation_strategy::immediate;

// 中文说明（文件级）：
// 目标 critic：仅在机器人接近终点时生效，
// 惩罚轨迹终点离目标点的距离，引导机器人在终点附近收敛停靠。

// 初始化：读取权重与生效距离阈值
void GoalCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);  // 代价幂次
  getParam(weight_, "cost_weight", 5.0);  // 权重
  // 只有机器人距目标小于该阈值时本 critic 才参与评分
  getParam(threshold_to_consider_, "threshold_to_consider", 1.4);

  RCLCPP_INFO(
    logger_, "GoalCritic instantiated with %d power and %f weight.",
    power_, weight_);
}

// 评分：接近目标时惩罚轨迹点到目标的平均距离
void GoalCritic::score(CriticData & data)
{
  // 未启用或机器人距目标尚远时不评分（不影响中段跟踪行为）
  if (!enabled_ || !utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  // 路径最后一个点即目标点
  const auto goal_idx = data.path.x.shape(0) - 1;

  const auto goal_x = data.path.x(goal_idx);
  const auto goal_y = data.path.y(goal_idx);

  // 每条轨迹全部点的坐标视图
  const auto traj_x = xt::view(data.trajectories.x, xt::all(), xt::all());
  const auto traj_y = xt::view(data.trajectories.y, xt::all(), xt::all());

  // 每个轨迹点到目标的欧氏距离
  auto dists = xt::sqrt(
    xt::pow(traj_x - goal_x, 2) +
    xt::pow(traj_y - goal_y, 2));

  // 对每条轨迹取平均距离，加权并取幂后累加到总代价
  data.costs += xt::pow(xt::mean(dists, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

// pluginlib 导出为 critic 插件
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::GoalCritic, mppi::critics::CriticFunction)
