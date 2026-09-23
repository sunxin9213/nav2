// Copyright (c) 2022, Samsung Research America
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

// ============================================================================
// 文件说明（中文）：
// 本文件为 nav2_smoother 模块提供公共工具函数（头文件实现，全部 inline）：
//   1. findDirectionalPathSegments：将路径按"行驶方向"切分为若干段，
//      识别尖点（cusp，前进/倒车切换点）与原地旋转点，避免平滑算法
//      把不同方向的两段路径"拉直"到一起。
//   2. updateApproximatePathOrientations：平滑后路径点的朝向会失真，
//      此函数根据相邻路径点的位置向量重新计算每个点的近似朝向角，
//      并可识别倒车段（朝向与运动方向相反，需要加 PI 翻转）。
// 这两个工具被 SimpleSmoother、SavitzkyGolaySmoother 等平滑插件共用。
// ============================================================================

#ifndef NAV2_SMOOTHER__SMOOTHER_UTILS_HPP_
#define NAV2_SMOOTHER__SMOOTHER_UTILS_HPP_

#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "angles/angles.h"
#include "tf2/utils.h"

namespace smoother_utils
{

/**
 * @class nav2_smoother::PathSegment
 * @brief A segment of a path in start/end indices
 * 路径分段：用起始/结束下标 [start, end]（闭区间）描述路径中的一段
 */
struct PathSegment
{
  unsigned int start;
  unsigned int end;
};

// 路径正向迭代器与反向迭代器的类型别名
typedef std::vector<geometry_msgs::msg::PoseStamped>::iterator PathIterator;
typedef std::vector<geometry_msgs::msg::PoseStamped>::reverse_iterator ReversePathIterator;

/**
 * @brief 将路径切分为若干"同方向行驶"的段。
 *
 * 算法核心：遍历路径中每个内部点 idx，构造两个向量：
 *   OA = P[idx] - P[idx-1]（上一段运动方向）
 *   AB = P[idx+1] - P[idx]（下一段运动方向）
 * 若两向量点积 < 0，说明运动方向发生反转（夹角大于 90 度），
 * 即出现"尖点"（cusp，例如差速机器人前进转倒车的切换点），
 * 在此处切断路径形成新的分段。
 *
 * 另外若相邻点位置几乎重合（|AB|≈0）但朝向角变化明显，
 * 则认为是"原地旋转"点，同样在此切断分段。
 *
 * @param path 待切分的原始路径
 * @return 分段列表（至少返回一个覆盖全路径的段）
 */
inline std::vector<PathSegment> findDirectionalPathSegments(
  const nav_msgs::msg::Path & path)
{
  std::vector<PathSegment> segments;
  PathSegment curr_segment;
  curr_segment.start = 0;

  // Iterating through the path to determine the position of the cusp
  // 遍历路径内部点（首尾点除外），逐点判断是否为方向切换点
  for (unsigned int idx = 1; idx < path.poses.size() - 1; ++idx) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    // 计算 OA 向量（前一点指向当前点）
    double oa_x = path.poses[idx].pose.position.x -
      path.poses[idx - 1].pose.position.x;
    double oa_y = path.poses[idx].pose.position.y -
      path.poses[idx - 1].pose.position.y;
    // 计算 AB 向量（当前点指向后一点）
    double ab_x = path.poses[idx + 1].pose.position.x -
      path.poses[idx].pose.position.x;
    double ab_y = path.poses[idx + 1].pose.position.y -
      path.poses[idx].pose.position.y;

    // Checking for the existance of cusp, in the path, using the dot product.
    // 点积为负 => 前后运动方向夹角大于 90 度 => 出现尖点（方向反转），
    // 在当前点处结束当前段并开启新段
    double dot_product = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_product < 0.0) {
      curr_segment.end = idx;
      segments.push_back(curr_segment);
      curr_segment.start = idx;
    }

    // Checking for the existance of a differential rotation in place.
    // 检测原地旋转：位置几乎不变但航向角发生明显变化
    double cur_theta = tf2::getYaw(path.poses[idx].pose.orientation);
    double next_theta = tf2::getYaw(path.poses[idx + 1].pose.orientation);
    // shortest_angular_distance 计算两角度间最短（归一化到 [-pi, pi]）角差
    double dtheta = angles::shortest_angular_distance(cur_theta, next_theta);
    if (fabs(ab_x) < 1e-4 && fabs(ab_y) < 1e-4 && fabs(dtheta) > 1e-4) {
      // 位置不变、角度变化 => 原地旋转点，同样切段
      curr_segment.end = idx;
      segments.push_back(curr_segment);
      curr_segment.start = idx;
    }
  }

  // 收尾：最后一段以路径末点结束并加入结果
  curr_segment.end = path.poses.size() - 1;
  segments.push_back(curr_segment);
  return segments;
}

/**
 * @brief 平滑后根据路径点位置向量重新计算各点近似朝向。
 *
 * 平滑算法只调整位置 (x, y)，会破坏原始朝向信息，因此需要重建：
 *   - 每个点的朝向取其指向下一点的方向角 atan2(dy, dx)；
 *   - 先用第 1~2 个点判断本段是否为倒车段（机器人朝向与运动方向相反，
 *     即朝向与运动方向夹角大于 90 度）；
 *   - 倒车段中所有朝向在运动方向基础上加 PI 翻转，符合倒车运动学；
 *   - 最后一个点保持不变（循环只处理到倒数第二个点）。
 *
 * @param path 输入输出路径（朝向被原地更新）
 * @param reversing_segment 输出：本段是否为倒车行驶段
 */
inline void updateApproximatePathOrientations(
  nav_msgs::msg::Path & path,
  bool & reversing_segment)
{
  double dx, dy, theta, pt_yaw;
  reversing_segment = false;

  // Find if this path segment is in reverse
  // 用第 1 个点到第 2 个点的运动方向与第 1 个点的实际朝向比较，
  // 判断该段是否是倒车段（角差超过 PI/2 视为倒车）
  dx = path.poses[2].pose.position.x - path.poses[1].pose.position.x;
  dy = path.poses[2].pose.position.y - path.poses[1].pose.position.y;
  theta = atan2(dy, dx);
  pt_yaw = tf2::getYaw(path.poses[1].pose.orientation);
  if (fabs(angles::shortest_angular_distance(pt_yaw, theta)) > M_PI_2) {
    reversing_segment = true;
  }

  // Find the angle relative the path position vectors
  // 遍历除末点外的所有点，按位置向量方向重设朝向
  for (unsigned int i = 0; i != path.poses.size() - 1; i++) {
    dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    theta = atan2(dy, dx);

    // If points are overlapping, pass
    // 相邻两点重合时无法确定方向角，跳过该点（保留原朝向）
    if (fabs(dx) < 1e-4 && fabs(dy) < 1e-4) {
      continue;
    }

    // Flip the angle if this path segment is in reverse
    // 倒车段：朝向 = 运动方向 + PI（orientationAroundZAxis 内部会做角度归一化）
    if (reversing_segment) {
      theta += M_PI;  // orientationAroundZAxis will normalize
    }

    // 由平面偏航角 theta 生成四元数并写入该点的朝向
    path.poses[i].pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
  }
}

}  // namespace smoother_utils

#endif  // NAV2_SMOOTHER__SMOOTHER_UTILS_HPP_
