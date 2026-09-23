// Copyright (c) 2019 Intel Corporation
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
//
// Modified by: Shivang Patel (shivaang14@gmail.com)
// ============================================================================
// 中文注释：基于话题的碰撞检测器（CostmapTopicCollisionChecker）实现文件
//
// 【文件功能】
// 本文件实现"以话题为数据源"的位姿碰撞检测器。与直接访问本地
// LayeredCostmap 的 FootprintCollisionChecker 不同，本类组合：
//   - CostmapSubscriber：订阅其他节点发布的代价地图话题；
//   - FootprintSubscriber：订阅动态足迹话题；
//   - FootprintCollisionChecker（组合成员 collision_checker_）：执行
//     实际的足迹栅格化碰撞计算。
// 提供 isCollisionFree(pose)：判断机器人在指定位姿是否碰撞。
//
// 【在 Nav2 中的位置】
// 被 CollisionMonitor（碰撞监控器）使用——碰撞监控器不拥有自己的
// 代价地图，而是复用 costmap 节点发布的地图做安全检查。
//
// 【异常体系】（见 exceptions.hpp）
// - IllegalPoseException：位姿落在地图外；
// - CollisionCheckerException：地图/足迹不可用等运行时错误。
// isCollisionFree 捕获所有异常并统一返回"碰撞"（安全优先）。
// ============================================================================

#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>

#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/exceptions.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/line_iterator.hpp"

using namespace std::chrono_literals;

namespace nav2_costmap_2d
{

// ============================================================================
// 构造函数：注入地图订阅器与足迹订阅器（碰撞检测器内部持有）
// ============================================================================

CostmapTopicCollisionChecker::CostmapTopicCollisionChecker(
  CostmapSubscriber & costmap_sub,
  FootprintSubscriber & footprint_sub,
  std::string name)
: name_(name),
  costmap_sub_(costmap_sub),
  footprint_sub_(footprint_sub),
  collision_checker_(nullptr)
{}

// ============================================================================
// isCollisionFree()：判断指定位姿是否无碰撞（安全优先）
// 代价 >= LETHAL_OBSTACLE(254) 视为碰撞；
// 任何异常（位姿出界/数据不可得）也一律视为碰撞（保守策略）
// ============================================================================

bool CostmapTopicCollisionChecker::isCollisionFree(
  const geometry_msgs::msg::Pose2D & pose,
  bool fetch_costmap_and_footprint)
{
  try {
    if (scorePose(pose, fetch_costmap_and_footprint) >= LETHAL_OBSTACLE) {
      return false;   // 代价达致命级：有碰撞
    }
    return true;
  } catch (const IllegalPoseException & e) {
    // 位姿在地图外：无法保证安全 → 视为碰撞
    RCLCPP_ERROR(rclcpp::get_logger(name_), "%s", e.what());
    return false;
  } catch (const CollisionCheckerException & e) {
    // 地图/足迹不可用：视为碰撞
    RCLCPP_ERROR(rclcpp::get_logger(name_), "%s", e.what());
    return false;
  } catch (...) {
    // 未知异常：保守处理，视为碰撞
    RCLCPP_ERROR(rclcpp::get_logger(name_), "Failed to check pose score!");
    return false;
  }
}

// ============================================================================
// scorePose()：计算位姿的足迹碰撞代价（数值越大越危险）
// fetch_costmap_and_footprint=true 时先从订阅器拉取最新地图与足迹
// ============================================================================

double CostmapTopicCollisionChecker::scorePose(
  const geometry_msgs::msg::Pose2D & pose,
  bool fetch_costmap_and_footprint)
{
  // 按需从话题订阅器拉取最新代价地图注入碰撞检测器
  if (fetch_costmap_and_footprint) {
    try {
      collision_checker_.setCostmap(costmap_sub_.getCostmap());
    } catch (const std::runtime_error & e) {
      // 地图尚不可用：包装为碰撞检测异常向上抛
      throw CollisionCheckerException(e.what());
    }
  }

  // 位姿中心：世界坐标 → 栅格坐标，出界抛 IllegalPoseException
  unsigned int cell_x, cell_y;
  if (!collision_checker_.worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    RCLCPP_DEBUG(rclcpp::get_logger(name_), "Map Cell: [%d, %d]", cell_x, cell_y);
    throw IllegalPoseException(name_, "Pose Goes Off Grid.");
  }

  // 以该位姿的完整足迹计算碰撞代价
  return collision_checker_.footprintCost(getFootprint(pose, fetch_costmap_and_footprint));
}

// ============================================================================
// getFootprint()：获取指定位姿下的世界坐标足迹
// fetch_latest_footprint=true 时先从足迹话题拉取最新足迹（机器人系），
// 再按位姿旋转平移到世界坐标
// ============================================================================

Footprint CostmapTopicCollisionChecker::getFootprint(
  const geometry_msgs::msg::Pose2D & pose,
  bool fetch_latest_footprint)
{
  // 按需更新缓存的机器人系足迹
  if (fetch_latest_footprint) {
    std_msgs::msg::Header header;
    if (!footprint_sub_.getFootprintInRobotFrame(footprint_, header)) {
      throw CollisionCheckerException("Current footprint not available.");
    }
  }
  Footprint footprint;
  // 足迹顶点按位姿 (x, y, theta) 旋转平移到世界坐标
  transformFootprint(pose.x, pose.y, pose.theta, footprint_, footprint);

  return footprint;
}

}  // namespace nav2_costmap_2d
