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
// ConstrainedSmoother 的核心算法实现（头文件内实现）：
// 基于 Ceres 非线性最小二乘求解器的"改进弹性带（elastic band）"路径平滑。
//
// 算法总体框架：
//   1. buildProblem：把路径点作为优化变量构造 Ceres 问题。
//      - 对代价地图构建 ceres::Grid2D + BiCubicInterpolator，
//        使障碍代价对 (x,y) 可微（双三次插值）；
//      - 按 path_downsampling_factor 对路径降采样以减少优化变量数量，
//        但尖点（前进/倒车切换点）与首末点附近绝不跳过；
//      - 对每个参与优化的点添加残差块 SmootherCostFunction，
//        包含平滑项、曲率项、数据保真项与代价地图项（详见
//        smoother_cost_function.hpp）；
//      - 尖点两侧 cusp_zone_length/2 范围内增大代价地图权重
//        （cusp_costmap_weight），防止尖点附近过度平滑而撞上障碍——
//        即"keep in line（保持在直线上）"处理；
//      - 首末两个参数块（及可选的首末朝向保持点）设为常量，
//        保证平滑前后端点位置与朝向约束。
//   2. ceres::Solve：迭代求解非线性最小二乘问题（可配置求解器类型、
//      迭代上限与三类收敛容差），受 max_time 时间预算限制。
//   3. upsampleAndPopulate：把优化后的稀疏点列按 path_upsampling_factor
//      用分段三次贝塞尔曲线上采样插值，重算各点朝向（切线方向，
//      尖点处按倒车方向翻转），生成最终稠密平滑路径。
// ============================================================================

#ifndef NAV2_CONSTRAINED_SMOOTHER__SMOOTHER_HPP_
#define NAV2_CONSTRAINED_SMOOTHER__SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>
#include <deque>
#include <limits>
#include <algorithm>

#include "nav2_constrained_smoother/smoother_cost_function.hpp"
#include "nav2_constrained_smoother/utils.hpp"

#include "ceres/ceres.h"
#include "Eigen/Core"

namespace nav2_constrained_smoother
{

/**
 * @class nav2_smac_planner::Smoother
 * @brief A Conjugate Gradient 2D path smoother implementation
 * 基于非线性最小二乘（Ceres）的 2D 路径平滑器：构造并求解
 * 平滑/曲率/保真/避障多目标优化问题，最后贝塞尔上采样输出。
 */
class Smoother
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::Smoother
   */
  Smoother() {}

  /**
   * @brief A destructor for nav2_smac_planner::Smoother
   */
  ~Smoother() {}

  /**
   * @brief Initialization of the smoother
   * @param params OptimizerParam struct
   * 用优化器参数初始化 Ceres 求解选项：
   *   线性求解器类型、最大迭代次数、函数/梯度/参数三类收敛容差，
   *   以及调试模式下向标准输出打印每次迭代的收敛过程。
   */
  void initialize(const OptimizerParams params)
  {
    debug_ = params.debug;

    // 线性求解器类型（如 SPARSE_NORMAL_CHOLESKY / DENSE_QR）
    options_.linear_solver_type = params.solver_types.at(params.linear_solver_type);

    // 非线性优化的最大迭代次数
    options_.max_num_iterations = params.max_iterations;

    // 三类收敛判据：目标函数变化量 / 梯度范数 / 参数变化量
    options_.function_tolerance = params.fn_tol;
    options_.gradient_tolerance = params.gradient_tol;
    options_.parameter_tolerance = params.param_tol;

    // 调试模式：每次迭代输出报告；否则静默求解
    if (debug_) {
      options_.minimizer_progress_to_stdout = true;
      options_.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
    } else {
      options_.logging_type = ceres::SILENT;
    }
  }

  /**
   * @brief Smoother method
   * @param path Reference to path
   * @param start_dir Orientation of the first pose
   * @param end_dir Orientation of the last pose
   * @param costmap Pointer to minimal costmap
   * @param params parameters weights
   * @return If smoothing was successful
   *
   * 平滑主流程：构建问题 -> Ceres 求解 -> 上采样回填路径。
   */
  bool smooth(
    std::vector<Eigen::Vector3d> & path,
    const Eigen::Vector2d & start_dir,
    const Eigen::Vector2d & end_dir,
    const nav2_costmap_2d::Costmap2D * costmap,
    const SmootherParams & params)
  {
    // Path has always at least 2 points
    // 路径至少要有 2 个点，否则抛异常
    if (path.size() < 2) {
      throw std::runtime_error("Constrained smoother: Path must have at least 2 points");
    }

    // 本次求解的时间预算（来自动作目标的 max_smoothing_duration）
    options_.max_solver_time_in_seconds = params.max_time;

    ceres::Problem problem;
    std::vector<Eigen::Vector3d> path_optim;  // 参与优化的（可能降采样后的）路径点
    std::vector<bool> optimized;              // 标记每个原始点是否参与优化
    if (buildProblem(path, costmap, params, problem, path_optim, optimized)) {
      // solve the problem
      // 求解非线性最小二乘问题
      ceres::Solver::Summary summary;
      ceres::Solve(options_, &problem, &summary);
      if (debug_) {
        RCLCPP_INFO(rclcpp::get_logger("smoother_server"), "%s", summary.FullReport().c_str());
      }
      // 解不可用（数值失败）或最终代价比初始还大（越优化越差）都视为失败
      if (!summary.IsSolutionUsable() || summary.initial_cost - summary.final_cost < 0.0) {
        return false;
      }
    } else {
      // 可优化点数不足（路径太短），无需优化，直接走插值回填
      RCLCPP_INFO(rclcpp::get_logger("smoother_server"), "Path too short to optimize");
    }

    // 将优化结果（含被跳过点的贝塞尔插值与朝向重建）写回输出路径
    upsampleAndPopulate(path_optim, optimized, start_dir, end_dir, params, path);

    return true;
  }

private:
  /**
   * @brief Build problem method
   * @param path Reference to path
   * @param costmap Pointer to costmap
   * @param params Smoother parameters
   * @param problem Output problem to solve
   * @param path_optim Output path on which the problem will be solved
   * @param optimized False for points skipped by downsampling
   * @return If there is a problem to solve
   *
   * 构造 Ceres 优化问题：遍历（可能降采样的）路径点，
   * 为每个相邻三点组 (prelast, last, current) 添加残差块。
   */
  bool buildProblem(
    const std::vector<Eigen::Vector3d> & path,
    const nav2_costmap_2d::Costmap2D * costmap,
    const SmootherParams & params,
    ceres::Problem & problem,
    std::vector<Eigen::Vector3d> & path_optim,
    std::vector<bool> & optimized)
  {
    // Create costmap grid
    // 将代价地图封装为 Ceres 网格并构造双三次插值器：
    // 使障碍代价成为连续可微函数，供自动求导使用
    costmap_grid_ = std::make_shared<ceres::Grid2D<u_char>>(
      costmap->getCharMap(), 0, costmap->getSizeInCellsY(), 0, costmap->getSizeInCellsX());
    auto costmap_interpolator = std::make_shared<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>(
      *costmap_grid_);

    // Create residual blocks
    // 尖点影响区域的半长
    const double cusp_half_length = params.cusp_zone_length / 2;
    ceres::LossFunction * loss_function = NULL;  // 不使用鲁棒核函数
    path_optim = path;
    optimized = std::vector<bool>(path.size());
    optimized[0] = true;
    int prelast_i = -1;   // 上上个被保留点的下标
    int last_i = 0;       // 上一个被保留点的下标
    double last_direction = path_optim[0][2];  // 上一段的方向符号（+1/-1）
    bool last_was_cusp = false;       // 上一个保留点是否为尖点
    bool last_is_reversing = false;   // 上一段是否倒车
    // 最近若干残差块队列（段长, 代价函数指针）：用于尖点出现时回溯调整权重
    std::deque<std::pair<double, SmootherCostFunction *>> potential_cusp_funcs;
    double last_segment_len = EPSILON;     // 上一段长度（防 0 加 epsilon）
    double potential_cusp_funcs_len = 0;   // 队列覆盖的累计长度
    double len_since_cusp = std::numeric_limits<double>::infinity();  // 距上个尖点的弧长

    for (size_t i = 1; i < path_optim.size(); i++) {
      auto & pt = path_optim[i];
      bool is_cusp = false;
      if (i != path_optim.size() - 1) {
        // 方向符号变号（正负相乘 < 0）说明此处是前进/倒车切换点（尖点）
        is_cusp = pt[2] * last_direction < 0;
        last_direction = pt[2];

        // skip to downsample if can be skipped (no forward/reverse direction change)
        // 降采样：若不是尖点，且不在需要保留的首末朝向区间内，
        // 且距离上一个保留点不足 downsampling_factor，则跳过该点
        if (!is_cusp &&
          i > (params.keep_start_orientation ? 1 : 0) &&
          i < path_optim.size() - (params.keep_goal_orientation ? 2 : 1) &&
          static_cast<int>(i - last_i) < params.path_downsampling_factor)
        {
          continue;
        }
      }

      // keep distance inequalities between poses
      // (some might have been downsampled while others might not)
      // 当前保留点与上一保留点之间的实际段长（降采样导致段长不均，
      // 通过长度比例传入代价函数以校正平滑项/曲率项的计算）
      double current_segment_len = (path_optim[i] - path_optim[last_i]).block<2, 1>(0, 0).norm();

      // forget cost functions which don't have chance to be part of a cusp zone
      // 滑窗维护：淘汰已超出尖点半长范围的历史残差块
      potential_cusp_funcs_len += current_segment_len;
      while (!potential_cusp_funcs.empty() && potential_cusp_funcs_len > cusp_half_length) {
        potential_cusp_funcs_len -= potential_cusp_funcs.front().first;
        potential_cusp_funcs.pop_front();
      }

      // update cusp zone costmap weights
      // 遇到尖点：向前回溯调整滑窗内各残差块的代价地图权重——
      // 越靠近尖点权重越接近 cusp_costmap_weight（按线性插值过渡），
      // 使尖点两侧路径段更"贴合原路径"，防止过度上冲切入障碍
      if (is_cusp) {
        double len_to_cusp = current_segment_len;
        for (int i = potential_cusp_funcs.size() - 1; i >= 0; i--) {
          auto & f = potential_cusp_funcs[i];
          // 线性插值：新权重在 cusp_costmap_weight 与 costmap_weight 之间
          double new_weight =
            params.cusp_costmap_weight * (1.0 - len_to_cusp / cusp_half_length) +
            params.costmap_weight * len_to_cusp / cusp_half_length;
          // 只有当新权重比现有权重更接近尖点权重时才更新
          if (std::abs(new_weight - params.cusp_costmap_weight) <
            std::abs(f.second->getCostmapWeight() - params.cusp_costmap_weight))
          {
            f.second->setCostmapWeight(new_weight);
          }
          len_to_cusp += f.first;
        }
        // 清空滑窗并重置尖点距离
        potential_cusp_funcs_len = 0;
        potential_cusp_funcs.clear();
        len_since_cusp = 0;
      }

      // add cost function
      // 为当前保留点添加残差块（至少已有 prelast/last 两个前置点）
      optimized[i] = true;
      if (prelast_i != -1) {
        double costmap_weight = params.costmap_weight;
        // 尖点后方半长范围内：代价权重按距尖点距离线性过渡（同上）
        if (len_since_cusp <= cusp_half_length) {
          costmap_weight =
            params.cusp_costmap_weight * (1.0 - len_since_cusp / cusp_half_length) +
            params.costmap_weight * len_since_cusp / cusp_half_length;
        }
        SmootherCostFunction * cost_function = new SmootherCostFunction(
          path[last_i].template block<2, 1>(   // 上一保留点的原始位置（数据保真项参考）
            0,
            0),
          // 前后段长度比；若上一保留点是尖点则取负号（编码方向反转）
          (last_was_cusp ? -1 : 1) * last_segment_len / current_segment_len,
          last_is_reversing,          // 上一段是否倒车
          costmap,
          costmap_interpolator,
          params,
          costmap_weight
        );
        // 残差块参数：last（前一点）、i（当前点）、prelast（前前一点）
        // 三个点的 x/y 均为优化变量（每个 Vector3d 的前 2 个分量）
        problem.AddResidualBlock(
          cost_function->AutoDiff(), loss_function,
          path_optim[last_i].data(), pt.data(), path_optim[prelast_i].data());

        // 当前残差块进入滑窗，供后续尖点回溯调整权重
        potential_cusp_funcs.emplace_back(current_segment_len, cost_function);
      }

      // shift current to last and last to pre-last
      // 滚动更新下标与状态：当前点变为 last，原 last 变为 prelast
      last_was_cusp = is_cusp;
      last_is_reversing = last_direction < 0;
      prelast_i = last_i;
      last_i = i;
      len_since_cusp += current_segment_len;
      last_segment_len = std::max(EPSILON, current_segment_len);
    }

    // 统计真正可优化的点数：总参数块 - 首末两点（必为常量），
    // 再减去被要求保持朝向的首/末相邻点
    int posesToOptimize = problem.NumParameterBlocks() - 2;  // minus start and goal
    if (params.keep_goal_orientation) {
      posesToOptimize -= 1;  // minus goal orientation holder
    }
    if (params.keep_start_orientation) {
      posesToOptimize -= 1;  // minus start orientation holder
    }
    if (posesToOptimize <= 0) {
      return false;  // nothing to optimize
    }
    // first two and last two points are constant (to keep start and end direction)
    // 端点固定：起点、终点（以及可选的起末朝向保持点）设为常量参数块，
    // 保证平滑后路径的起点/终点位置（和朝向）与原路径一致
    problem.SetParameterBlockConstant(path_optim.front().data());
    if (params.keep_start_orientation) {
      problem.SetParameterBlockConstant(path_optim[1].data());
    }
    if (params.keep_goal_orientation) {
      problem.SetParameterBlockConstant(path_optim[path_optim.size() - 2].data());
    }
    problem.SetParameterBlockConstant(path_optim.back().data());
    return true;
  }

  /**
   * @brief Populate optimized points to path, assigning orientations and upsampling poses using cubic bezier
   * @param path_optim Path with optimized points
   * @param optimized False for points skipped by downsampling
   * @param start_dir Orientation of the first pose
   * @param end_dir Orientation of the last pose
   * @param params Smoother parameters
   * @param path Output path with upsampled optimized points
   *
   * 输出阶段：遍历所有参与优化的点，在相邻保留点之间用三次贝塞尔曲线
   * 插值（还原被降采样跳过的点并可选上采样加密），并基于切线方向
   * 重算每个输出点的朝向角（倒车段朝向取反向）。
   */
  void upsampleAndPopulate(
    const std::vector<Eigen::Vector3d> & path_optim,
    const std::vector<bool> & optimized,
    const Eigen::Vector2d & start_dir,
    const Eigen::Vector2d & end_dir,
    const SmootherParams & params,
    std::vector<Eigen::Vector3d> & path)
  {
    // Populate path, assign orientations, interpolate skipped/upsampled poses
    path.clear();
    if (params.path_upsampling_factor > 1) {
      // 预留空间：约 (n-1)*上采样倍率 + 1 个点
      path.reserve(params.path_upsampling_factor * (path_optim.size() - 1) + 1);
    }
    int last_i = 0;
    int prelast_i = -1;
    Eigen::Vector2d prelast_dir;  // prelast 点的切线方向（贝塞尔控制点用）
    for (int i = 1; i <= static_cast<int>(path_optim.size()); i++) {
      // 遇到参与优化的点（或遍历到末尾哨兵）时，结算 [prelast, last] 区间
      if (i == static_cast<int>(path_optim.size()) || optimized[i]) {
        if (prelast_i != -1) {
          Eigen::Vector2d last_dir;
          auto & prelast = path_optim[prelast_i];
          auto & last = path_optim[last_i];

          // Compute orientation of last
          // 计算 last 点的朝向（切线方向）：
          if (i < static_cast<int>(path_optim.size())) {
            auto & current = path_optim[i];
            // 三点圆弧的切线方向（尖点标志：prelast 与 last 方向符号相反）
            Eigen::Vector2d tangent_dir = tangentDir<double>(
              prelast.block<2, 1>(0, 0),
              last.block<2, 1>(0, 0),
              current.block<2, 1>(0, 0),
              prelast[2] * last[2] < 0);

            // 切线方向符号不定：与"运动方向"点积为正者为正确朝向
            // （last[2] 编码了倒车符号，保证倒车段朝向相反）
            last_dir =
              tangent_dir.dot((current - last).block<2, 1>(0, 0) * last[2]) >= 0 ?
              tangent_dir :
              -tangent_dir;
            last_dir.normalize();
          } else if (params.keep_goal_orientation) {
            last_dir = end_dir;  // 末点：保留原始终点朝向
          } else {
            // 不保留终点朝向：沿最后一段方向（乘方向符号处理倒车）
            last_dir = (last - prelast).block<2, 1>(0, 0) * last[2];
            last_dir.normalize();
          }
          double last_angle = atan2(last_dir[1], last_dir[0]);

          // Interpolate poses between prelast and last
          // 在 prelast 与 last 之间插值出被跳过/上采样的点
          // 插值点数 = 跳过点数 * 上采样倍率 - 1
          int interp_cnt = (last_i - prelast_i) * params.path_upsampling_factor - 1;
          if (interp_cnt > 0) {
            Eigen::Vector2d last_pt = last.block<2, 1>(0, 0);
            Eigen::Vector2d prelast_pt = prelast.block<2, 1>(0, 0);
            double dist = (last_pt - prelast_pt).norm();
            // 贝塞尔控制点：沿两端切线方向、距离为段长的 0.4 倍
            // （乘 prelast[2] 处理倒车段的方向翻转）
            Eigen::Vector2d pt1 = prelast_pt + prelast_dir * dist * 0.4 * prelast[2];
            Eigen::Vector2d pt2 = last_pt - last_dir * dist * 0.4 * prelast[2];
            for (int j = 1; j <= interp_cnt; j++) {
              // 参数 interp 取 (0,1) 开区间，端点由外部输出
              double interp = j / static_cast<double>(interp_cnt + 1);
              Eigen::Vector2d pt = cubicBezier(prelast_pt, pt1, pt2, last_pt, interp);
              path.emplace_back(pt[0], pt[1], 0.0);  // 朝向稍后统一计算
            }
          }
          // 输出 last 点（含朝向角）
          path.emplace_back(last[0], last[1], last_angle);

          // Assign orientations to interpolated points
          // 为刚插值出的点赋朝向：取其前后相邻点确定的圆弧切线方向，
          // 符号与运动方向一致（乘 prelast[2] 编码倒车）
          for (size_t j = path.size() - 1 - interp_cnt; j < path.size() - 1; j++) {
            Eigen::Vector2d tangent_dir = tangentDir<double>(
              path[j - 1].block<2, 1>(0, 0),
              path[j].block<2, 1>(0, 0),
              path[j + 1].block<2, 1>(0, 0),
              false);
            tangent_dir =
              tangent_dir.dot((path[j + 1] - path[j]).block<2, 1>(0, 0) * prelast[2]) >= 0 ?
              tangent_dir :
              -tangent_dir;
            path[j][2] = atan2(tangent_dir[1], tangent_dir[0]);
          }

          prelast_dir = last_dir;  // 当前 last 的方向作为下一区间的起点切线
        } else {  // start pose
          // 处理起点：朝向取原始起点朝向，或（不保留时）沿首段方向
          auto & start = path_optim[0];
          Eigen::Vector2d dir = params.keep_start_orientation ?
            start_dir :
            ((path_optim[i] - start).block<2, 1>(0, 0) * start[2]).normalized();
          path.emplace_back(start[0], start[1], atan2(dir[1], dir[0]));
          prelast_dir = dir;
        }
        prelast_i = last_i;
        last_i = i;
      }
    }
  }

  /*
    Piecewise cubic bezier curve as defined by Adobe in Postscript
    The two end points are pt0 and pt3
    Their associated control points are pt1 and pt2
    分段三次贝塞尔曲线（PostScript 定义形式）：
    pt0/pt3 为端点，pt1/pt2 为对应控制点；
    用多项式系数形式 a*u^3 + b*u^2 + c*u + pt0 直接求值。
  */
  static Eigen::Vector2d cubicBezier(
    Eigen::Vector2d & pt0, Eigen::Vector2d & pt1,
    Eigen::Vector2d & pt2, Eigen::Vector2d & pt3, double mu)
  {
    Eigen::Vector2d a, b, c, pt;

    // 多项式展开系数（等价于标准 3 次伯恩斯坦基）
    c[0] = 3 * (pt1[0] - pt0[0]);
    c[1] = 3 * (pt1[1] - pt0[1]);
    b[0] = 3 * (pt2[0] - pt1[0]) - c[0];
    b[1] = 3 * (pt2[1] - pt1[1]) - c[1];
    a[0] = pt3[0] - pt0[0] - c[0] - b[0];
    a[1] = pt3[1] - pt0[1] - c[1] - b[1];

    // 按 mu ∈ [0,1] 求曲线上的点
    pt[0] = a[0] * mu * mu * mu + b[0] * mu * mu + c[0] * mu + pt0[0];
    pt[1] = a[1] * mu * mu * mu + b[1] * mu * mu + c[1] * mu + pt0[1];

    return pt;
  }

  bool debug_;  // 是否打印 Ceres 求解过程
  ceres::Solver::Options options_;  // Ceres 求解器配置
  std::shared_ptr<ceres::Grid2D<u_char>> costmap_grid_;  // 代价地图的 Ceres 网格封装
};

}  // namespace nav2_constrained_smoother

#endif  // NAV2_CONSTRAINED_SMOOTHER__SMOOTHER_HPP_
