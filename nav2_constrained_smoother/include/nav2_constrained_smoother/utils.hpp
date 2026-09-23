// Copyright (c) 2021 RoboTech Vision
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

// ============================================================================
// 文件说明（中文）：
// ConstrainedSmoother 的数学工具函数（模板 + inline，供自动微分使用）：
//   arcCenter  —— 求过三点的圆弧圆心（曲率计算的基础）；
//   tangentDir —— 求三点圆弧在中点的切线方向（朝向重建的基础）。
// 两者均为模板函数（类型 T 在自动微分时为 Ceres 的 Jet 类型），
// 是"DSRB 数学"（Differential/几何计算）的核心。
// ============================================================================

#ifndef NAV2_CONSTRAINED_SMOOTHER__UTILS_HPP_
#define NAV2_CONSTRAINED_SMOOTHER__UTILS_HPP_

#include <limits>
#include "Eigen/Core"

// 数值下限常量：避免除零/零长度段
#define EPSILON 0.0001

/**
 * Compatibility with different ceres::isinf() and ceres::IsInfinite() API
 * used in Ceres Solver 2.1.0+ and 2.0.0- versions respectively
 */
#if defined(USE_OLD_CERES_API)
  #define CERES_ISINF(x) ceres::IsInfinite(x)
#else
  #define CERES_ISINF(x) ceres::isinf(x)
#endif

namespace nav2_constrained_smoother
{

/**
 * @brief Center of an arc between three points
 * @param pt_prev Starting point of the arc
 * @param pt Mid point of the arc
 * @param pt_next Last point of the arc
 * @param is_cusp True if pt is a cusp point
 * @result position of the center or Vector2(inf, inf) for straight lines and 180 deg turns
 */
template<typename T>
inline Eigen::Matrix<T, 2, 1> arcCenter(
  Eigen::Matrix<T, 2, 1> pt_prev,
  Eigen::Matrix<T, 2, 1> pt,
  Eigen::Matrix<T, 2, 1> pt_next,
  bool is_cusp)
{
  // 前段向量 d1 = pt - pt_prev，后段向量 d2 = pt_next - pt
  Eigen::Matrix<T, 2, 1> d1 = pt - pt_prev;
  Eigen::Matrix<T, 2, 1> d2 = pt_next - pt;

  if (is_cusp) {
    // 尖点处运动方向反转：将后段向量取反，
    // 使"前进->倒车"的路径在几何上等效为同方向的折线，
    // 这样才能求出正确的等效转弯圆弧
    d2 = -d2;
    pt_next = pt + d2;
  }

  // d1、d2 叉积（行列式）。接近 0 说明三点共线（直线或 180 度折返）
  T det = d1[0] * d2[1] - d1[1] * d2[0];
  if (ceres::abs(det) < (T)1e-4) {  // straight line
    // 共线：圆心在无穷远（曲率为 0），返回 (inf, inf) 哨兵值
    return Eigen::Matrix<T, 2, 1>(
      (T)std::numeric_limits<double>::infinity(), (T)std::numeric_limits<double>::infinity());
  }

  // circle center is at the intersection of mirror axes of the segments:
  // 圆心位于两段弦的中垂线的交点：
  // http://paulbourke.net/geometry/circlesphere/
  // line intersection:
  // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Intersection%20of%20two%20lines
  // mid1/mid2 为两段弦的中点；n1/n2 为各自的中垂线方向（弦向量旋转 90 度）
  Eigen::Matrix<T, 2, 1> mid1 = (pt_prev + pt) / (T)2;
  Eigen::Matrix<T, 2, 1> mid2 = (pt + pt_next) / (T)2;
  Eigen::Matrix<T, 2, 1> n1(-d1[1], d1[0]);
  Eigen::Matrix<T, 2, 1> n2(-d2[1], d2[0]);
  // 用两点式直线方程的克莱姆法则求两中垂线的交点（即圆心）
  T det1 = (mid1[0] + n1[0]) * mid1[1] - (mid1[1] + n1[1]) * mid1[0];
  T det2 = (mid2[0] + n2[0]) * mid2[1] - (mid2[1] + n2[1]) * mid2[0];
  Eigen::Matrix<T, 2, 1> center((det1 * n2[0] - det2 * n1[0]) / det,
    (det1 * n2[1] - det2 * n1[1]) / det);
  return center;
}

/**
 * @brief Direction of a line which contains pt and is tangential to arc
 * between pt_prev, pt, pt_next
 * @param pt_prev Starting point of the arc
 * @param pt Mid point of the arc, lying on the tangential line
 * @param pt_next Last point of the arc
 * @param is_cusp True if pt is a cusp point
 * @result Tangential line direction.
 * Note: the sign of tangentDir is undefined here, should be assigned in post-process
 * depending on movement direction. Also, for speed reasons, direction vector is not normalized.
 */
template<typename T>
inline Eigen::Matrix<T, 2, 1> tangentDir(
  Eigen::Matrix<T, 2, 1> pt_prev,
  Eigen::Matrix<T, 2, 1> pt,
  Eigen::Matrix<T, 2, 1> pt_next,
  bool is_cusp)
{
  // 先求过三点的圆弧圆心
  Eigen::Matrix<T, 2, 1> center = arcCenter(pt_prev, pt, pt_next, is_cusp);
  if (CERES_ISINF(center[0])) {  // straight line
    // 共线（圆心在无穷远）：切线方向即直线方向
    Eigen::Matrix<T, 2, 1> d1 = pt - pt_prev;
    Eigen::Matrix<T, 2, 1> d2 = pt_next - pt;

    if (is_cusp) {
      // 尖点处后段向量取反，与 arcCenter 中的处理保持一致
      d2 = -d2;
      pt_next = pt + d2;
    }

    // 切线取（等效）首末点连线方向
    Eigen::Matrix<T, 2, 1> result(pt_next[0] - pt_prev[0], pt_next[1] - pt_prev[1]);
    if (result[0] == 0.0 && result[1] == 0.0) {  // a very rare edge situation
      // 极端情况：首末点重合（180 度原地折返），用前段的垂直方向
      return Eigen::Matrix<T, 2, 1>(d1[1], -d1[0]);
    }
    return result;
  }

  // tangent is prependicular to (pt - center)
  // Note: not determining + or - direction here, this should be handled at the caller side
  // 一般情况：圆弧上 pt 点的切线垂直于半径向量 (pt - center)，
  // 返回其垂直向量（未归一化以提升速度）；
  // 正负方向（前进还是倒车朝向）由调用方根据运动方向决定
  return Eigen::Matrix<T, 2, 1>(center[1] - pt[1], pt[0] - center[0]);
}

}  // namespace nav2_constrained_smoother

#endif  // NAV2_CONSTRAINED_SMOOTHER__UTILS_HPP_
