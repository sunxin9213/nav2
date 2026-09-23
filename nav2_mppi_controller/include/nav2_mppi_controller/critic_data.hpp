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

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_

#include <memory>
#include <vector>
#include <xtensor/xtensor.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "nav2_mppi_controller/motion_models.hpp"


namespace mppi
{

/**
 * @struct mppi::CriticData
 * @brief Data to pass to critics for scoring, including state, trajectories, path, costs, and
 * important parameters to share
 *（critic 共享数据包：优化器每周期填充，供所有 critic 评分使用）
 *
 * 中文说明：
 * 该结构体由 Optimizer 持有并预填充引用，避免每周期重新构造，
 * 所有 critic 通过同一份只读数据并行评分，代价统一累加到 costs。
 */
struct CriticData
{
  const models::State & state;              // 批量状态（含噪声控制）
  const models::Trajectories & trajectories;  // 候选轨迹
  const models::Path & path;                // 全局路径（张量形式）

  xt::xtensor<float, 1> & costs;  // 每条轨迹的累计代价（各 critic 依次累加）
  float & model_dt;              // 轨迹推演时间步长

  bool fail_flag;               // 失败标志（如所有轨迹碰撞时置位，触发优化器回退）
  nav2_core::GoalChecker * goal_checker;        // 目标检查器（判断是否到达终点）
  std::shared_ptr<MotionModel> motion_model;     // 运动模型（部分 critic 需要）
  // 路径点是否有效（未被障碍物占据等），供 PathFollow/PathAlign 等 critic 复用
  std::optional<std::vector<bool>> path_pts_valid;
  // 机器人已到达的最远路径点索引（首个 critic 计算后供后续 critic 复用，避免重复搜索）
  std::optional<size_t> furthest_reached_path_point;
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_
