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

#include "nav2_mppi_controller/critics/path_follow_critic.hpp"

#include <xtensor/xmath.hpp>
#include <xtensor/xsort.hpp>

namespace mppi::critics
{

// 中文说明（文件级）：
// 路径跟随（Path Follow）critic：惩罚候选轨迹终点到参考路径上
// “前视目标点”的距离，驱动机器人沿路径向前推进。
// 前视目标点 = 机器人已到达的最远路径点 + offset_from_furthest，
// 若该点被动态障碍占据则继续向后寻找有效点（绕过障碍而非穿过）。

// 初始化：读取参数
void PathFollowCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  // 接近目标（距离小于阈值）时不再评分
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 1.4);
  // 前视偏移：最远到达点之后再走多少个路径点
  getParam(offset_from_furthest_, "offset_from_furthest", 6);
  getParam(power_, "cost_power", 1);  // 代价幂次
  getParam(weight_, "cost_weight", 5.0);  // 权重
}

// 评分：轨迹终点到前视路径点的距离
void PathFollowCritic::score(CriticData & data)
{
  // 未启用 / 路径过短 / 已接近目标：跳过
  if (!enabled_ || data.path.x.shape(0) < 2 ||
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  // 计算最远到达路径点索引与路径点的有效性（供跳过被占据点）
  utils::setPathFurthestPointIfNotSet(data);
  utils::setPathCostsIfNotSet(data, costmap_ros_);
  const size_t path_size = data.path.x.shape(0) - 1;

  // 前视目标点索引 = 最远到达点 + 偏移（不超过路径末尾）
  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, path_size);

  // Drive to the first valid path point, in case of dynamic obstacles on path
  // we want to drive past it, not through it
  //（目标点被占据时向后搜索首个有效点：绕过动态障碍而不是穿过它）
  bool valid = false;
  while (!valid && offseted_idx < path_size - 1) {
    valid = (*data.path_pts_valid)[offseted_idx];
    if (!valid) {
      offseted_idx++;
    }
  }

  // 前视路径点坐标
  const auto path_x = data.path.x(offseted_idx);
  const auto path_y = data.path.y(offseted_idx);

  // 所有候选轨迹的终点坐标（最后一列）
  const auto last_x = xt::view(data.trajectories.x, xt::all(), -1);
  const auto last_y = xt::view(data.trajectories.y, xt::all(), -1);

  // 轨迹终点到前视点的欧氏距离
  auto dists = xt::sqrt(
    xt::pow(last_x - path_x, 2) +
    xt::pow(last_y - path_y, 2));

  // 加权、取幂后累加到总代价
  data.costs += xt::pow(weight_ * std::move(dists), power_);
}

}  // namespace mppi::critics

// pluginlib 导出为 critic 插件
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathFollowCritic,
  mppi::critics::CriticFunction)
