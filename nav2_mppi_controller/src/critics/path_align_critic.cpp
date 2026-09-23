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

#include "nav2_mppi_controller/critics/path_align_critic.hpp"

#include <xtensor/xfixed.hpp>
#include <xtensor/xmath.hpp>

namespace mppi::critics
{

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

// 中文说明（文件级）：
// 路径对齐（Path Align）critic：惩罚候选轨迹与全局路径的横向偏差，
// 促使机器人在行进中贴合参考路径（与 PathFollowCritic 互补：
// Follow 看“终点推进”，Align 看“沿途贴合”）。
// 通过弧长积分将轨迹点映射到等距路径点，计算两者的欧氏距离
// （可选地叠加朝向差），并对无效（被障碍占据）的路径点跳过。

// 初始化：读取权重与若干生效条件参数
void PathAlignCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);          // 代价幂次
  getParam(weight_, "cost_weight", 10.0);     // 权重（默认较大，路径贴合优先）

  // 路径上无效点占比超过该比例（动态障碍挡路）时本 critic 不参与
  getParam(max_path_occupancy_ratio_, "max_path_occupancy_ratio", 0.07);
  // 机器人已到达的最远路径点之外再加该偏移才开始评分（刚上路时不评）
  getParam(offset_from_furthest_, "offset_from_furthest", 20);
  // 轨迹点采样步长（每 N 步取一个点，降低计算量）
  getParam(trajectory_point_step_, "trajectory_point_step", 4);
  // 接近目标时停止评分，交给 goal 类 critic
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 0.5);
  // 是否在距离中叠加朝向差（3 维距离）
  getParam(use_path_orientations_, "use_path_orientations", false);

  RCLCPP_INFO(
    logger_,
    "ReferenceTrajectoryCritic instantiated with %d power and %f weight",
    power_, weight_);
}

// 评分主函数：计算轨迹点到对应路径点的平均距离
void PathAlignCritic::score(CriticData & data)
{
  // Don't apply close to goal, let the goal critics take over
  //（接近目标时交由 goal 类 critic 接管）
  if (!enabled_ ||
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  // Don't apply when first getting bearing w.r.t. the path
  //（刚对准路径方向时不评分：计算机器人已到达的最远路径点索引）
  utils::setPathFurthestPointIfNotSet(data);
  const size_t path_segments_count = *data.furthest_reached_path_point;  // up to furthest only
  // 最远点偏移不足时（刚开始跟踪）跳过
  if (path_segments_count < offset_from_furthest_) {
    return;
  }

  // Don't apply when dynamic obstacles are blocking significant proportions of the local path
  //（动态障碍占据路径比例过高时不评分，交由避障 critic 处理）
  utils::setPathCostsIfNotSet(data, costmap_ros_);
  // 找到轨迹起点对应的最近路径点索引
  const size_t closest_initial_path_point = utils::findPathTrajectoryInitialPoint(data);
  unsigned int invalid_ctr = 0;
  const float range = *data.furthest_reached_path_point - closest_initial_path_point;
  // 统计有效区间内无效（障碍占据）路径点的比例
  for (size_t i = closest_initial_path_point; i < *data.furthest_reached_path_point; i++) {
    if (!(*data.path_pts_valid)[i]) {invalid_ctr++;}
    if (static_cast<float>(invalid_ctr) / range > max_path_occupancy_ratio_ && invalid_ctr > 2) {
      return;  // 占比超限：本周期跳过路径对齐
    }
  }

  // 路径点视图（去掉最后一个点，按线段处理）
  const auto P_x = xt::view(data.path.x, xt::range(_, -1));  // path points
  const auto P_y = xt::view(data.path.y, xt::range(_, -1));  // path points
  const auto P_yaw = xt::view(data.path.yaws, xt::range(_, -1));  // path points

  const size_t batch_size = data.trajectories.x.shape(0);
  const size_t time_steps = data.trajectories.x.shape(1);
  auto && cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});

  // Find integrated distance in the path
  //（预计算路径的累积弧长表：path_integrated_distances[i] = 路径起点到第 i 点的弧长）
  std::vector<float> path_integrated_distances(path_segments_count, 0.0f);
  float dx = 0.0f, dy = 0.0f;
  for (unsigned int i = 1; i != path_segments_count; i++) {
    dx = P_x(i) - P_x(i - 1);
    dy = P_y(i) - P_y(i - 1);
    float curr_dist = sqrtf(dx * dx + dy * dy);
    path_integrated_distances[i] = path_integrated_distances[i - 1] + curr_dist;
  }

  // 逐轨迹计算：按弧长匹配对应路径点并累计距离
  float traj_integrated_distance = 0.0f;
  float summed_path_dist = 0.0f, dyaw = 0.0f;
  float num_samples = 0.0f;
  float Tx = 0.0f, Ty = 0.0f;
  size_t path_pt = 0;
  for (size_t t = 0; t < batch_size; ++t) {
    // 每条轨迹重置累计量
    traj_integrated_distance = 0.0f;
    summed_path_dist = 0.0f;
    num_samples = 0.0f;
    path_pt = 0u;
    const auto T_x = xt::view(data.trajectories.x, t, xt::all());
    const auto T_y = xt::view(data.trajectories.y, t, xt::all());
    // 按步长采样轨迹点
    for (size_t p = trajectory_point_step_; p < time_steps; p += trajectory_point_step_) {
      Tx = T_x(p);
      Ty = T_y(p);
      // 轨迹累积弧长增量
      dx = Tx - T_x(p - trajectory_point_step_);
      dy = Ty - T_y(p - trajectory_point_step_);
      traj_integrated_distance += sqrtf(dx * dx + dy * dy);
      // 按弧长找到对应的路径点（单调递增，可增量搜索）
      path_pt = utils::findClosestPathPt(
        path_integrated_distances, traj_integrated_distance, path_pt);

      // The nearest path point to align to needs to be not in collision, else
      // let the obstacle critic take over in this region due to dynamic obstacles
      //（对应路径点被占据时跳过，让避障 critic 在该区域主导）
      if ((*data.path_pts_valid)[path_pt]) {
        dx = P_x(path_pt) - Tx;
        dy = P_y(path_pt) - Ty;
        num_samples += 1.0f;
        if (use_path_orientations_) {
          // 可选：叠加朝向差的 3 维距离
          const auto T_yaw = xt::view(data.trajectories.yaws, t, xt::all());
          dyaw = angles::shortest_angular_distance(P_yaw(path_pt), T_yaw(p));
          summed_path_dist += sqrtf(dx * dx + dy * dy + dyaw * dyaw);
        } else {
          // 默认：二维欧氏距离
          summed_path_dist += sqrtf(dx * dx + dy * dy);
        }
      }
    }
    // 平均距离作为该轨迹的对齐代价
    if (num_samples > 0) {
      cost[t] = summed_path_dist / num_samples;
    } else {
      cost[t] = 0.0f;
    }
  }

  // 加权、取幂后累加到总代价
  data.costs += xt::pow(std::move(cost) * weight_, power_);
}

}  // namespace mppi::critics

// pluginlib 导出为 critic 插件
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAlignCritic,
  mppi::critics::CriticFunction)
