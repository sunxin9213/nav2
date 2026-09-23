// Copyright (c) 2021, Samsung Research America
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
// limitations under the License. Reserved.

#include "nav2_smac_planner/collision_checker.hpp"

namespace nav2_smac_planner
{

// 【中文注释】GridCollisionChecker：SMAC 规划器的代价地图碰撞检查器，
// 继承自 FootprintCollisionChecker（提供多边形足迹的 costmap 采样）。
// 核心思想：把机器人足迹按所有离散朝向角(angle bin)预先旋转并缓存，
// 碰撞检查时只需查表 + 平移，避免每次都做三角函数旋转，大幅加速搜索。
GridCollisionChecker::GridCollisionChecker(
  nav2_costmap_2d::Costmap2D * costmap,
  unsigned int num_quantizations,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: FootprintCollisionChecker(costmap)
{
  if (node) {
    clock_ = node->get_clock();
    logger_ = node->get_logger();
  }

  // Convert number of regular bins into angles
  // 【中文注释】把角度量化数（angle_quantization_bins，如 72）转换为
  // 均匀分布的朝向角表：第 i 个 bin 对应角度 i * (2*PI/N)。
  float bin_size = 2 * M_PI / static_cast<float>(num_quantizations);
  angles_.reserve(num_quantizations);
  for (unsigned int i = 0; i != num_quantizations; i++) {
    angles_.push_back(bin_size * i);
  }
}

// GridCollisionChecker::GridCollisionChecker(
//   nav2_costmap_2d::Costmap2D * costmap,
//   std::vector<float> & angles)
// : FootprintCollisionChecker(costmap),
//   angles_(angles)
// {
// }

void GridCollisionChecker::setFootprint(
  const nav2_costmap_2d::Footprint & footprint,
  const bool & radius,
  const double & possible_inscribed_cost)
{
  // 【中文注释】设置机器人足迹：
  // - radius=true：机器人用外接圆半径近似（配合膨胀层，检查中心点代价即可）；
  // - radius=false：用完整多边形足迹，为每个角度 bin 预计算旋转后的足迹
  //   （oriented_footprints_），搜索时只需查表平移。
  possible_inscribed_cost_ = possible_inscribed_cost;
  footprint_is_radius_ = radius;

  // Use radius, no caching required
  if (radius) {
    return;
  }

  // No change, no updates required
  // 【中文注释】足迹未变化时跳过重复的预计算
  if (footprint == unoriented_footprint_) {
    return;
  }

  oriented_footprints_.clear();
  oriented_footprints_.reserve(angles_.size());
  double sin_th, cos_th;
  geometry_msgs::msg::Point new_pt;
  const unsigned int footprint_size = footprint.size();

  // Precompute the orientation bins for checking to use
  // 【中文注释】对每个离散朝向角做 2D 旋转矩阵变换：
  // (x,y) -> (x*cos-y*sin, x*sin+y*cos)，得到该朝向下的足迹多边形
  for (unsigned int i = 0; i != angles_.size(); i++) {
    sin_th = sin(angles_[i]);
    cos_th = cos(angles_[i]);
    nav2_costmap_2d::Footprint oriented_footprint;
    oriented_footprint.reserve(footprint_size);

    for (unsigned int j = 0; j < footprint_size; j++) {
      new_pt.x = footprint[j].x * cos_th - footprint[j].y * sin_th;
      new_pt.y = footprint[j].x * sin_th + footprint[j].y * cos_th;
      oriented_footprint.push_back(new_pt);
    }

    oriented_footprints_.push_back(oriented_footprint);
  }

  unoriented_footprint_ = footprint;
}

bool GridCollisionChecker::inCollision(
  const float & x,
  const float & y,
  const float & angle_bin,
  const bool & traverse_unknown)
{
  // 【中文注释】带朝向的碰撞检查（Hybrid/Lattice 用）：
  // (x,y) 为栅格坐标，angle_bin 为角度量化索引。
  // 多边形足迹时采用两级检查（快速通道）：
  //   1) 先看中心格代价：若低于"可能内切代价"，足迹必然安全，直接返回不碰撞；
  //   2) 中心格为内切/占据时必碰撞；否则用预旋转足迹平移到 (x,y) 做完整检查。
  // 圆形（radius）足迹时只查中心格代价。
  // Check to make sure cell is inside the map
  if (outsideRange(costmap_->getSizeInCellsX(), x) ||
    outsideRange(costmap_->getSizeInCellsY(), y))
  {
    return true;
  }

  // Assumes setFootprint already set
  // 【中文注释】栅格坐标转世界坐标（足迹顶点以世界坐标表示）
  double wx, wy;
  costmap_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);

  if (!footprint_is_radius_) {
    // if footprint, then we check for the footprint's points, but first see
    // if the robot is even potentially in an inscribed collision
    // 【中文注释】第一级：检查中心格代价
    footprint_cost_ = costmap_->getCost(
      static_cast<unsigned int>(x), static_cast<unsigned int>(y));

    if (footprint_cost_ < possible_inscribed_cost_) {
      if (possible_inscribed_cost_ > 0) {
        // 【中文注释】快速通过：中心代价低于内切阈值，足迹任何朝向都安全
        return false;
      } else {
        // 【中文注释】膨胀层未配置/内切代价为 0：给出性能警告
        // （每次都要做完整足迹检查，速度会明显变慢）
        RCLCPP_ERROR_THROTTLE(
          logger_, *clock_, 1000,
          "Inflation layer either not found or inflation is not set sufficiently for "
          "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
          " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
          "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
          " for full instructions. This will substantially impact run-time performance.");
      }
    }

    // If its inscribed, in collision, or unknown in the middle,
    // no need to even check the footprint, its invalid
    // 【中文注释】中心格为未知且不允许穿越未知 -> 碰撞；
    // 中心格内切(INSCRIBED)或占据(OCCUPIED) -> 必然碰撞
    if (footprint_cost_ == UNKNOWN && !traverse_unknown) {
      return true;
    }

    if (footprint_cost_ == INSCRIBED || footprint_cost_ == OCCUPIED) {
      return true;
    }

    // if possible inscribed, need to check actual footprint pose.
    // Use precomputed oriented footprints are done on initialization,
    // offset by translation value to collision check
    // 【中文注释】第二级：中心格处于"可能内切"的灰色地带，
    // 取出该 angle_bin 的预旋转足迹，平移到当前位置做完整多边形代价检查
    geometry_msgs::msg::Point new_pt;
    const nav2_costmap_2d::Footprint & oriented_footprint = oriented_footprints_[angle_bin];
    nav2_costmap_2d::Footprint current_footprint;
    current_footprint.reserve(oriented_footprint.size());
    for (unsigned int i = 0; i < oriented_footprint.size(); ++i) {
      new_pt.x = wx + oriented_footprint[i].x;
      new_pt.y = wy + oriented_footprint[i].y;
      current_footprint.push_back(new_pt);
    }

    // 【中文注释】footprintCost 沿多边形边光栅化并取所有覆盖格的最大代价
    footprint_cost_ = footprintCost(current_footprint);

    if (footprint_cost_ == UNKNOWN && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    // 【中文注释】足迹覆盖到占据(或不可穿越的未知)格 -> 碰撞
    return footprint_cost_ >= OCCUPIED;
  } else {
    // if radius, then we can check the center of the cost assuming inflation is used
    // 【中文注释】圆形足迹：只需检查中心格代价（膨胀层保证半径外安全）
    footprint_cost_ = costmap_->getCost(
      static_cast<unsigned int>(x), static_cast<unsigned int>(y));

    if (footprint_cost_ == UNKNOWN && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    // 【中文注释】代价 >= INSCRIBED(253) 视为碰撞
    return static_cast<double>(footprint_cost_) >= INSCRIBED;
  }
}

bool GridCollisionChecker::inCollision(
  const unsigned int & i,
  const bool & traverse_unknown)
{
  // 【中文注释】按一维索引的碰撞检查（Node2D 用，不带朝向）：
  // 只查单格代价，UNKNOWN 且允许穿越未知时不视为碰撞。
  footprint_cost_ = costmap_->getCost(i);
  if (footprint_cost_ == UNKNOWN && traverse_unknown) {
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return footprint_cost_ >= INSCRIBED;
}

float GridCollisionChecker::getCost()
{
  // Assumes inCollision called prior
  // 【中文注释】返回最近一次 inCollision 缓存的代价值（供代价计算使用）
  return static_cast<float>(footprint_cost_);
}

bool GridCollisionChecker::outsideRange(const unsigned int & max, const float & value)
{
  // 【中文注释】越界判断：坐标为负或超出地图尺寸
  return value < 0.0f || value > max;
}

}  // namespace nav2_smac_planner
