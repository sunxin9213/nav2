// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2020, Samsung Research America
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
// 平滑器代价函数（SmootherCostFunction）的实现。
// 模块定位：改进 elastic band 平滑的核心数学，被 Ceres 求解器调用。
// 每个残差块作用于相邻三个路径点 (pt_prev, pt, pt_next)，
// 输出 4 维残差，对应四个优化目标（加权后构成总代价）：
//   residual[0] 平滑项（addSmoothingResidual）：
//     w_smooth * |ratio*d_next - d_prev|^2 —— 让相邻两段方向与长度
//     相互匹配（等价于最小化转角/曲率变化），d_next、d_prev 为前后段向量。
//   residual[1] 曲率约束项（addCurvatureResidual）：
//     三点确定外接圆弧，若曲率 k=1/R 超过最大曲率 max_curvature
//     （由最小转弯半径换算），则惩罚 (k - k_max)^2；不超限时不惩罚，
//     属于"软约束的铰链（hinge）"形式。
//   residual[2] 数据保真项（addDistanceResidual）：
//     w_data * |xi - xi_original|^2 —— 把优化点拉向原始路径点，
//     防止平滑后路径偏离规划意图（w_dist 默认 0，即默认不启用）。
//   residual[3] 代价地图/避障项（addCostResidual）：
//     用双三次插值在代价地图上采样当前点（或按 cost_check_points
//     给出的机器人轮廓检查点）的代价值并平方惩罚，把路径推离障碍
//     与高代价区。尖点附近使用更大的 cusp_costmap_weight。
// 整个 operator() 是模板函数，配合 ceres::AutoDiffCostFunction
// 实现自动微分求雅可比，无需手写导数。
// ============================================================================

#ifndef NAV2_CONSTRAINED_SMOOTHER__SMOOTHER_COST_FUNCTION_HPP_
#define NAV2_CONSTRAINED_SMOOTHER__SMOOTHER_COST_FUNCTION_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <memory>
#include <queue>
#include <utility>

#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"
#include "Eigen/Core"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_constrained_smoother/options.hpp"
#include "nav2_constrained_smoother/utils.hpp"

namespace nav2_constrained_smoother
{

/**
 * @struct nav2_constrained_smoother::SmootherCostFunction
 * @brief Cost function for path smoothing with multiple terms
 * including curvature, smoothness, distance from original and obstacle avoidance.
 * 多目标路径平滑代价函数（平滑/曲率/保真/避障四项）。
 */
class SmootherCostFunction
{
public:
  /**
   * @brief A constructor for nav2_constrained_smoother::SmootherCostFunction
   * @param original_path Original position of the path node
   * @param next_to_last_length_ratio Ratio of next path segment compared to previous.
   *  Negative if one of them represents reversing motion.
   * @param reversing Whether the path segment after this node represents reversing motion.
   * @param costmap A costmap to get values for collision and obstacle avoidance
   * @param params Optimization weights and parameters
   * @param costmap_weight Costmap cost weight. Can be params.costmap_weight or params.cusp_costmap_weight
   */
  SmootherCostFunction(
    const Eigen::Vector2d & original_pos,
    double next_to_last_length_ratio,
    bool reversing,
    const nav2_costmap_2d::Costmap2D * costmap,
    const std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> & costmap_interpolator,
    const SmootherParams & params,
    double costmap_weight)
  : original_pos_(original_pos),                        // 原始路径点位置（保真项参考）
    next_to_last_length_ratio_(next_to_last_length_ratio),  // 后段/前段长度比（负值表示经过尖点）
    reversing_(reversing),                              // 该点之后是否倒车
    params_(params),                                    // 平滑参数（各项权重等）
    costmap_weight_(costmap_weight),                    // 代价地图项权重（尖点区可被动态调大）
    costmap_origin_(costmap->getOriginX(), costmap->getOriginY()),  // 代价地图原点（世界坐标）
    costmap_resolution_(costmap->getResolution()),      // 代价地图分辨率（米/格）
    costmap_interpolator_(costmap_interpolator)         // 代价地图双三次插值器
  {
  }

  // 包装为 Ceres 自动微分代价函数：
  // 残差维度 4，三个参数块各为 2 维（前一点、当前点、前前一点的 x/y）
  ceres::CostFunction * AutoDiff()
  {
    return new ceres::AutoDiffCostFunction<SmootherCostFunction, 4, 2, 2, 2>(this);
  }

  // 动态设置代价地图项权重（尖点区域回溯调整时使用）
  void setCostmapWeight(double costmap_weight)
  {
    costmap_weight_ = costmap_weight;
  }

  // 读取当前代价地图项权重
  double getCostmapWeight()
  {
    return costmap_weight_;
  }

  /**
   * @brief Smoother cost function evaluation
   * @param pt X, Y coords of current point
   * @param pt_next X, Y coords of next point
   * @param pt_prev X, Y coords of previous point
   * @param pt_residual array of output residuals (smoothing, curvature, distance, cost)
   * @return if successful in computing values
   *
   * 代价函数求值：依次累加四个残差项（模板类型 T 支持 Ceres 自动微分）。
   */
  template<typename T>
  bool operator()(
    const T * const pt, const T * const pt_next, const T * const pt_prev,
    T * pt_residual) const
  {
    // 将裸指针包装为 Eigen 向量，便于向量运算
    Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi(pt);
    Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi_next(pt_next);
    Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi_prev(pt_prev);
    Eigen::Map<Eigen::Matrix<T, 4, 1>> residual(pt_residual);
    residual.setZero();

    // compute cost
    // 依次计算四项残差：平滑、曲率、数据保真、代价地图
    addSmoothingResidual<T>(params_.smooth_weight, xi, xi_next, xi_prev, residual[0]);
    addCurvatureResidual<T>(params_.curvature_weight, xi, xi_next, xi_prev, residual[1]);
    addDistanceResidual<T>(
      params_.distance_weight, xi,
      original_pos_.template cast<T>(), residual[2]);
    addCostResidual<T>(costmap_weight_, xi, xi_next, xi_prev, residual[3]);

    return true;
  }

protected:
  /**
   * @brief Cost function term for smooth paths
   * @param weight Weight to apply to function
   * @param pt Point Xi for evaluation
   * @param pt_next Point Xi+1 for calculating Xi's cost
   * @param pt_prev Point Xi-1 for calculating Xi's cost
   * @param r Residual (cost) of term
   *
   * 平滑项：最小化 |ratio * d_next - d_prev|^2。
   * 若前后段长度相等且比例修正后方向一致，则残差为 0；
   * 即要求路径"匀速直行"或平滑转弯，惩罚方向突变与长度跳变。
   * （ratio 为负表示经过尖点，前后段方向相反时的等效处理。）
   */
  template<typename T>
  inline void addSmoothingResidual(
    const double & weight,
    const Eigen::Matrix<T, 2, 1> & pt,
    const Eigen::Matrix<T, 2, 1> & pt_next,
    const Eigen::Matrix<T, 2, 1> & pt_prev,
    T & r) const
  {
    Eigen::Matrix<T, 2, 1> d_next = pt_next - pt;  // 后段向量
    Eigen::Matrix<T, 2, 1> d_prev = pt - pt_prev;  // 前段向量
    // 长度比修正后的差向量：衡量前后段"不连续"程度
    Eigen::Matrix<T, 2, 1> d_diff = next_to_last_length_ratio_ * d_next - d_prev;
    r += (T)weight * d_diff.dot(d_diff);    // objective function value
  }

  /**
   * @brief Cost function term for maximum curved paths
   * @param weight Weight to apply to function
   * @param pt Point Xi for evaluation
   * @param pt_next Point Xi+1 for calculating Xi's cost
   * @param pt_prev Point Xi-1 for calculating Xi's cost
   * @param curvature_params A struct to cache computations for the jacobian to use
   * @param r Residual (cost) of term
   *
   * 曲率约束项：三点确定外接圆弧（见 utils.hpp 的 arcCenter），
   * 转弯半径 R -> 曲率 k = 1/R。仅当 k > max_curvature（超过
   * 最小转弯半径允许的最大曲率）时施加二次惩罚（软约束/铰链形式），
   * 满足约束时不产生任何代价。
   */
  template<typename T>
  inline void addCurvatureResidual(
    const double & weight,
    const Eigen::Matrix<T, 2, 1> & pt,
    const Eigen::Matrix<T, 2, 1> & pt_next,
    const Eigen::Matrix<T, 2, 1> & pt_prev,
    T & r) const
  {
    // 计算过三点的圆弧圆心（尖点时反转后段向量，等价处理方向切换）
    Eigen::Matrix<T, 2, 1> center = arcCenter(
      pt_prev, pt, pt_next,
      next_to_last_length_ratio_ < 0);
    if (CERES_ISINF(center[0])) {
      // 三点共线（直线）：曲率为 0，必然满足约束，直接返回
      return;
    }
    T turning_rad = (pt - center).norm();              // 转弯半径 R
    T ki_minus_kmax = (T)1.0 / turning_rad - params_.max_curvature;  // k - k_max

    if (ki_minus_kmax <= (T)EPSILON) {
      // 曲率未超限：不惩罚（铰链的"松弛"一侧）
      return;
    }

    // 超过最大曲率：二次惩罚，迫使路径点回到可转弯范围内
    r += (T)weight * ki_minus_kmax * ki_minus_kmax;  // objective function value
  }

  /**
   * @brief Cost function derivative term for steering away changes in pose
   * @param weight Weight to apply to function
   * @param xi Point Xi for evaluation
   * @param xi_original original point Xi for evaluation
   * @param r Residual (cost) of term
   *
   * 数据保真项：|xi - xi_original|^2。
   * 把优化中的点拉回原始规划点位置，防止平滑后路径大幅偏离原路径
   * （例如抄近道穿过障碍膨胀区）。权重 w_dist 默认为 0（不启用）。
   */
  template<typename T>
  inline void addDistanceResidual(
    const double & weight,
    const Eigen::Matrix<T, 2, 1> & xi,
    const Eigen::Matrix<T, 2, 1> & xi_original,
    T & r) const
  {
    r += (T)weight * (xi - xi_original).squaredNorm();  // objective function value
  }

  /**
   * @brief Cost function term for steering away from costs
   * @param weight Weight to apply to function
   * @param value Point Xi's cost'
   * @param params computed values to reduce overhead
   * @param r Residual (cost) of term
   *
   * 代价地图（避障）项，两种模式：
   * 1) cost_check_points 为空：只检查当前点本身——
   *    将世界坐标转为网格坐标（减原点、除分辨率，-0.5 对齐栅格中心），
   *    用双三次插值取代价值并平方累加。平滑的插值使该代价可导。
   * 2) cost_check_points 非空：按机器人轮廓检查点列表
   *    [x1,y1,w1, x2,y2,w2, ...]（机器人本体坐标系）逐个检查——
   *    根据当前点的切线方向构造旋转平移矩阵，把各检查点变换到
   *    世界系后采样代价值，加权（w 已归一化）累加平方惩罚，
   *    相当于对整个机器人足迹做避障。
   */
  template<typename T>
  inline void addCostResidual(
    const double & weight,
    const Eigen::Matrix<T, 2, 1> & pt,
    const Eigen::Matrix<T, 2, 1> & pt_next,
    const Eigen::Matrix<T, 2, 1> & pt_prev,
    T & r) const
  {
    if (params_.cost_check_points.empty()) {
      // 模式 1：单点检查。世界坐标 -> 地图网格坐标
      Eigen::Matrix<T, 2, 1> interp_pos =
        (pt - costmap_origin_.template cast<T>()) / (T)costmap_resolution_;
      T value;
      // 注意行列顺序：Evaluate(行=y, 列=x)，-0.5 补偿栅格中心偏移
      costmap_interpolator_->Evaluate(interp_pos[1] - (T)0.5, interp_pos[0] - (T)0.5, &value);
      r += (T)weight * value * value;  // objective function value
    } else {
      // 模式 2：多点足迹检查
      // 求当前点处路径的切线方向（尖点时按反转处理）
      Eigen::Matrix<T, 2, 1> dir = tangentDir(
        pt_prev, pt, pt_next,
        next_to_last_length_ratio_ < 0);
      dir.normalize();
      // 若切线方向与运动方向不符（考虑倒车），翻转切线
      if (((pt_next - pt).dot(dir) < (T)0) != reversing_) {
        dir = -dir;
      }
      // 构造本体坐标系 -> 世界坐标系的刚体变换矩阵
      // [cos -sin x; sin cos y; 0 0 1]（dir 为单位向量即 cos/sin）
      Eigen::Matrix<T, 3, 3> transform;
      transform << dir[0], -dir[1], pt[0],
        dir[1], dir[0], pt[1],
        (T)0, (T)0, (T)1;
      // 遍历所有检查点（每 3 个数为一组：x, y, weight）
      for (size_t i = 0; i < params_.cost_check_points.size(); i += 3) {
        Eigen::Matrix<T, 3, 1> ccpt((T)params_.cost_check_points[i],
          (T)params_.cost_check_points[i + 1], (T)1);
        // 检查点变换到世界坐标
        auto ccpt_world = (transform * ccpt).template block<2, 1>(0, 0);
        // 世界坐标 -> 地图网格坐标
        Eigen::Matrix<T, 2,
          1> interp_pos = (ccpt_world - costmap_origin_.template cast<T>()) /
          (T)costmap_resolution_;
        T value;
        costmap_interpolator_->Evaluate(interp_pos[1] - (T)0.5, interp_pos[0] - (T)0.5, &value);

        // 加权平方累加：权重为该检查点的归一化权重
        r += (T)weight * (T)params_.cost_check_points[i + 2] * value * value;
      }
    }
  }

  const Eigen::Vector2d original_pos_;      // 原始路径点位置（保真项参考）
  double next_to_last_length_ratio_;        // 后/前段长度比（负值表示尖点）
  bool reversing_;                          // 该点之后是否为倒车段
  SmootherParams params_;                   // 平滑参数与各项权重
  double costmap_weight_;                   // 代价地图项权重（尖点区可调大）
  Eigen::Vector2d costmap_origin_;          // 代价地图世界坐标原点
  double costmap_resolution_;               // 代价地图分辨率（米/格）
  std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> costmap_interpolator_;  // 双三次插值器
};

}  // namespace nav2_constrained_smoother

#endif  // NAV2_CONSTRAINED_SMOOTHER__SMOOTHER_COST_FUNCTION_HPP_
