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

#ifndef NAV2_COSTMAP_2D__FOOTPRINT_COLLISION_CHECKER_HPP_
#define NAV2_COSTMAP_2D__FOOTPRINT_COLLISION_CHECKER_HPP_
// ============================================================================
// 中文注释：足迹碰撞检测器头文件（实现见 footprint_collision_checker.cpp）
//
// 【类功能】
// 模板类（CostmapT = Costmap2D* 或 shared_ptr<Costmap2D>）：
// 计算足迹多边形扫过区域的最大代价（轮廓 Bresenham 栅格化）。
//   - footprintCost(footprint)：足迹（世界坐标）碰撞代价；
//   - footprintCostAtPose(x,y,theta,footprint)：按位姿变换后检测——
//     控制器评估候选轨迹的标准入口；
//   - 返回 LETHAL_OBSTACLE(254)=碰撞、INSCRIBED_INFLATED_OBSTACLE(253)=
//     中心必碰、其他值越大越危险。
// ============================================================================

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_costmap_2d
{
typedef std::vector<geometry_msgs::msg::Point> Footprint;

/**
 * @class FootprintCollisionChecker
 * @brief Checker for collision with a footprint on a costmap
 */
template<typename CostmapT>
class FootprintCollisionChecker
{
public:
  /**
   * @brief A constructor.
   */
  FootprintCollisionChecker();
  /**
   * @brief A constructor.
   */
  explicit FootprintCollisionChecker(CostmapT costmap);
  /**
   * @brief Find the footprint cost in oriented footprint
   */
  double footprintCost(const Footprint footprint);
  /**
   * @brief Find the footprint cost a a post with an unoriented footprint
   */
  double footprintCostAtPose(double x, double y, double theta, const Footprint footprint);
  /**
   * @brief Get the cost for a line segment
   */
  double lineCost(int x0, int x1, int y0, int y1) const;
  /**
   * @brief Get the map coordinates from a world point
   */
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
  /**
   * @brief Get the cost of a point
   */
  double pointCost(int x, int y) const;
  /**
  * @brief Set the current costmap object to use for collision detection
  */
  void setCostmap(CostmapT costmap);
  /**
  * @brief Get the current costmap object
  */
  CostmapT getCostmap()
  {
    return costmap_;
  }

protected:
  CostmapT costmap_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__FOOTPRINT_COLLISION_CHECKER_HPP_
