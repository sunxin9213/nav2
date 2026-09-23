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

#ifndef NAV2_UTIL__GEOMETRY_UTILS_HPP_
#define NAV2_UTIL__GEOMETRY_UTILS_HPP_

#include <cmath>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_util
{
namespace geometry_utils
{
// ==================== 几何工具（内联函数/模板） ====================
// 提供位姿、点、路径相关的常用几何计算：四元数生成、欧氏距离、
// 路径长度、以及两个泛型迭代器算法 min_by / first_after_integrated_distance。

/**
 * @brief Get a geometry_msgs Quaternion from a yaw angle
 * @param angle Yaw angle to generate a quaternion from
 * @return geometry_msgs Quaternion
 */
// 偏航角 -> 四元数：平面机器人只需要绕 Z 轴的旋转（roll=pitch=0）
inline geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);  // void returning function
  // tf2 数学类型转换为 ROS 消息类型
  return tf2::toMsg(q);
}

/**
 * @brief Get the euclidean distance between 2 geometry_msgs::Points
 * @param pos1 First point
 * @param pos1 Second point
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::Point & pos1,
  const geometry_msgs::msg::Point & pos2,
  const bool is_3d = false)
{
  // std::hypot 计算 sqrt(dx^2+dy^2[+dz^2])，
  // 相比手写 sqrt 可避免中间平方运算的溢出/下溢
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;

  if (is_3d) {
    double dz = pos1.z - pos2.z;
    return std::hypot(dx, dy, dz);
  }

  // 默认 2D 距离（平面导航最常用）
  return std::hypot(dx, dy);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Poses
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double euclidean distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::Pose & pos1,
  const geometry_msgs::msg::Pose & pos2,
  const bool is_3d = false)
{
  double dx = pos1.position.x - pos2.position.x;
  double dy = pos1.position.y - pos2.position.y;

  if (is_3d) {
    double dz = pos1.position.z - pos2.position.z;
    return std::hypot(dx, dy, dz);
  }

  return std::hypot(dx, dy);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::PoseStamped
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::PoseStamped & pos1,
  const geometry_msgs::msg::PoseStamped & pos2,
  const bool is_3d = false)
{
  // PoseStamped 版本：剥掉 header 后复用 Pose 版本的实现
  return euclidean_distance(pos1.pose, pos2.pose, is_3d);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Pose2D
 * @param pos1 First pose
 * @param pos1 Second pose
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::Pose2D & pos1,
  const geometry_msgs::msg::Pose2D & pos2)
{
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;

  return std::hypot(dx, dy);
}

/**
 * Find element in iterator with the minimum calculated value
 */
template<typename Iter, typename Getter>
inline Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  // 空区间直接返回 end（约定俗成的"未找到"返回值）
  if (begin == end) {
    return end;
  }
  // 线性扫描：Getter 是可调用对象（lambda/函数指针），为每个元素计算比较值
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

/**
 * Find first element in iterator that is greater integrated distance than comparevalue
 */
template<typename Iter, typename Getter>
inline Iter first_after_integrated_distance(Iter begin, Iter end, Getter getCompareVal)
{
  // 沿序列累加相邻元素之间的欧氏距离（弧长积分），
  // 返回累计弧长首次超过给定阈值的那个迭代器
  if (begin == end) {
    return end;
  }
  Getter dist = 0.0;
  for (Iter it = begin; it != end - 1; it++) {
    dist += euclidean_distance(*it, *(it + 1));
    if (dist > getCompareVal) {
      return it + 1;
    }
  }
  return end;
}

/**
 * @brief Calculate the length of the provided path, starting at the provided index
 * @param path Path containing the poses that are planned
 * @param start_index Optional argument specifying the starting index for
 * the calculation of path length. Provide this if you want to calculate length of a
 * subset of the path.
 * @return double Path length
 */
inline double calculate_path_length(const nav_msgs::msg::Path & path, size_t start_index = 0)
{
  // 有效路径段不足两点时长度为 0
  if (start_index + 1 >= path.poses.size()) {
    return 0.0;
  }
  // 逐段累加相邻位姿的欧氏距离得到路径总长
  double path_length = 0.0;
  for (size_t idx = start_index; idx < path.poses.size() - 1; ++idx) {
    path_length += euclidean_distance(path.poses[idx].pose, path.poses[idx + 1].pose);
  }
  return path_length;
}

}  // namespace geometry_utils
}  // namespace nav2_util

#endif  // NAV2_UTIL__GEOMETRY_UTILS_HPP_
