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

#include "nav2_mppi_controller/critics/goal_angle_critic.hpp"
#include "angles/angles.h"
namespace mppi::critics
{

// 中文说明（文件级）：
// 目标朝向（Goal Angle）critic：仅在机器人接近终点时生效，
// 惩罚候选轨迹朝向与目标朝向之间的角度差，使机器人最终
// 停在期望的目标朝向（对应对齐终点 yaw）。

// 初始化：读取参数
void GoalAngleCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);  // 代价幂次
  getParam(weight_, "cost_weight", 3.0);  // 权重

  // 距目标小于该距离时才参与评分
  getParam(threshold_to_consider_, "threshold_to_consider", 0.5);
  // 对称机器人（前后无差别）时启用：目标朝向 +180° 也可接受
  getParam(symmetric_yaw_tolerance_, "symmetric_yaw_tolerance", false);

  RCLCPP_INFO(
    logger_,
    "GoalAngleCritic instantiated with %d power, %f weight, %f "
    "angular threshold and symmetric_yaw_tolerance %s",
    power_, weight_, threshold_to_consider_, symmetric_yaw_tolerance_ ? "enabled" : "disabled");
}

// 评分：惩罚轨迹朝向与目标朝向的平均角差
void GoalAngleCritic::score(CriticData & data)
{
  // 未启用或未接近目标时不评分
  if (!enabled_ || !utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  // 路径最后一个点的朝向即目标朝向
  const auto goal_idx = data.path.x.shape(0) - 1;
  const float goal_yaw = data.path.yaws(goal_idx);

  // 每个轨迹点到目标朝向的最短角差（取绝对值）
  auto angular_distances =
    xt::eval(xt::fabs(utils::shortest_angular_distance(data.trajectories.yaws, goal_yaw)));

  if (symmetric_yaw_tolerance_) {
    // For symmetric robots: use minimum distance to either goal orientation or goal + 180°
    //（对称机器人：取“目标朝向”与“目标朝向+180°”两者中较小的角差）
    const float symmetric_goal_yaw = angles::normalize_angle(goal_yaw + M_PI);
    auto symmetric_distances =
      xt::eval(
      xt::fabs(
        utils::shortest_angular_distance(
          data.trajectories.yaws,
          symmetric_goal_yaw)));
    angular_distances = xt::eval(xt::minimum(angular_distances, symmetric_distances));
  }

  // 沿轨迹取平均角差，加权取幂后累加
  data.costs += xt::pow(
    xt::mean(angular_distances, {1}) * weight_, power_);
}

}  // namespace mppi::critics

// pluginlib 导出为 critic 插件
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::GoalAngleCritic,
  mppi::critics::CriticFunction)
