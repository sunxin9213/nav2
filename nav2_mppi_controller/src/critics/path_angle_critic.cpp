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

#include "nav2_mppi_controller/critics/path_angle_critic.hpp"

#include <math.h>

namespace mppi::critics
{

// 中文说明（文件级）：
// 路径角度（Path Angle）critic：当机器人朝向与路径前进方向夹角过大时，
// 惩罚轨迹朝向与前视点方向之间的角度差，驱动机器人转向对准路径，
// 避免横着漂移或大偏角跟踪。仅在夹角超过 max_angle_to_furthest 时生效。

// 初始化：读取参数并判断是否允许倒车
void PathAngleCritic::initialize()
{
  // 从父控制器读取 vx_min 以判断是否支持倒车
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  float vx_min;
  getParentParam(vx_min, "vx_min", -0.35);
  if (fabs(vx_min) < 1e-6) {  // zero
    reversing_allowed_ = false;  // 最小速度为 0：不支持倒车
  } else if (vx_min < 0.0) {   // reversing possible
    reversing_allowed_ = true;   // 允许负速度：支持倒车
  }

  auto getParam = parameters_handler_->getParamGetter(name_);
  // 前视点相对最远到达点的偏移量
  getParam(offset_from_furthest_, "offset_from_furthest", 4);
  getParam(power_, "cost_power", 1);   // 代价幂次
  getParam(weight_, "cost_weight", 2.0);  // 权重
  // 接近目标时停止评分
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 0.5);
  // 机器人朝向与路径方向夹角超过该值时本 critic 才参与
  getParam(
    max_angle_to_furthest_,
    "max_angle_to_furthest", 1.2);
  // 是否偏好前进（false 且允许倒车时，可接受倒车对准路径）
  getParam(
    forward_preference_,
    "forward_preference", true);

  if (!reversing_allowed_) {
    forward_preference_ = true;  // 不允许倒车时强制偏好前进
  }

  RCLCPP_INFO(
    logger_,
    "PathAngleCritic instantiated with %d power and %f weight. Reversing %s",
    power_, weight_, reversing_allowed_ ? "allowed." : "not allowed.");
}

// 评分：惩罚轨迹朝向与到前视点方向之间的角度差
void PathAngleCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;
  if (!enabled_) {
    return;
  }

  // 接近目标时不评分
  if (utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path)) {
    return;
  }

  // 计算最远到达路径点并取前视点
  utils::setPathFurthestPointIfNotSet(data);

  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, data.path.x.shape(0) - 1);

  // 前视点坐标（作为转向目标）
  const float goal_x = xt::view(data.path.x, offseted_idx);
  const float goal_y = xt::view(data.path.y, offseted_idx);

  // 当前机器人朝向与前视方向的夹角小于阈值时无需校正，跳过评分
  if (utils::posePointAngle(
      data.state.pose.pose, goal_x, goal_y, forward_preference_) < max_angle_to_furthest_)
  {
    return;
  }

  // 每个轨迹点到前视点的方向角（期望朝向）
  auto yaws_between_points = xt::atan2(
    goal_y - data.trajectories.y,
    goal_x - data.trajectories.x);

  // 轨迹朝向与期望朝向的最短角差（绝对值）
  auto yaws =
    xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points));

  if (reversing_allowed_ && !forward_preference_) {
    // 允许倒车且无前进偏好：轨迹可以倒着对准路径（朝向 + 180°）
    const auto yaws_between_points_corrected = xt::where(
      yaws < M_PI_2, yaws_between_points, utils::normalize_angles(yaws_between_points + M_PI));
    const auto corrected_yaws = xt::abs(
      utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points_corrected));
    // 取沿轨迹的平均角差作为代价
    data.costs += xt::pow(xt::mean(corrected_yaws, {1}, immediate) * weight_, power_);
  } else {
    // 前进模式：直接用平均角差
    data.costs += xt::pow(xt::mean(yaws, {1}, immediate) * weight_, power_);
  }
}

}  // namespace mppi::critics

// pluginlib 导出为 critic 插件
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAngleCritic,
  mppi::critics::CriticFunction)
