// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#ifndef NAV2_GRACEFUL_CONTROLLER__EGO_POLAR_COORDS_HPP_
#define NAV2_GRACEFUL_CONTROLLER__EGO_POLAR_COORDS_HPP_

#include <math.h>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/utils.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_graceful_controller
{

/**
 * @brief Egocentric polar coordinates defined as the difference between the
 * robot pose and the target pose relative to the robot position and orientation.
 *（自极坐标系：平滑控制律所使用的坐标系统）
 *
 * 中文说明：
 * 平滑控制律的理论推导基于“自极坐标”：以机器人当前位置为极点、
 * 机器人到目标的视线为基准方向，定义三个状态量：
 *   r     —— 机器人到目标的径向距离；
 *   φ(phi) —— 目标朝向相对视线的角度（视线系下的目标朝向偏差）；
 *   δ(delta) —— 机器人朝向相对视线的角度（视线系下的机器人朝向偏差）。
 * 在该坐标系下控制律可被证明收敛（Lyapunov 分析）。
 */
struct EgocentricPolarCoordinates
{
  float r;       // Radial distance between the robot pose and the target pose.
                 //（径向距离：机器人到目标的欧氏距离）
  float phi;     // Orientation of target with respect to the line of sight
                 // from the robot to the target.
                 //（目标朝向相对视线的角度）
  float delta;   // Steering angle of the robot with respect to the line of sight.
                 //（机器人朝向相对视线的角度）

  EgocentricPolarCoordinates(
    const float & r_in = 0.0,
    const float & phi_in = 0.0,
    const float & delta_in = 0.0)
  : r(r_in), phi(phi_in), delta(delta_in) {}

  /**
   * @brief Construct a new egocentric polar coordinates as the difference between the robot pose
   * and the target pose relative to the robot position and orientation, both referenced to the same frame.
   *
   * Thus, r, phi and delta are always at the origin of the frame.
   *（由目标位姿与当前位姿计算自极坐标）
   *
   * Thus, r, phi and delta are always at the origin of the frame.
   *（要求两个位姿在同一坐标系下；计算结果相当于原点处的自极坐标）
   *
   * @param target Target pose.（目标位姿）
   * @param current Current pose. Defaults to the origin.（当前位姿，默认为原点）
   * @param backward If true, the robot is moving backwards. Defaults to false.（是否倒车）
   */
  explicit EgocentricPolarCoordinates(
    const geometry_msgs::msg::Pose & target,
    const geometry_msgs::msg::Pose & current = geometry_msgs::msg::Pose(), bool backward = false)
  {
    // Compute the difference between the target and the current pose
    //（目标相对当前位置的位移）
    float dX = target.position.x - current.position.x;
    float dY = target.position.y - current.position.y;
    // Compute the line of sight from the robot to the target
    // Flip it if the robot is moving backwards
    //（视线角：目标方向的负角；倒车时翻转 180° 使“前方”指向运动方向）
    float line_of_sight = backward ? (std::atan2(-dY, dX) + M_PI) : std::atan2(-dY, dX);
    // Compute the ego polar coordinates
    //（三个状态量：距离 r、目标朝向偏差 φ、机器人朝向偏差 δ）
    r = sqrt(dX * dX + dY * dY);
    phi = angles::normalize_angle(tf2::getYaw(target.orientation) + line_of_sight);
    delta = angles::normalize_angle(tf2::getYaw(current.orientation) + line_of_sight);
  }
};

}  // namespace nav2_graceful_controller

#endif  // NAV2_GRACEFUL_CONTROLLER__EGO_POLAR_COORDS_HPP_
