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
// ConstrainedSmoother 的参数结构定义：
//   SmootherParams  —— 平滑算法参数（各项代价权重、曲率限制、
//                      尖点区域长度、降/上采样倍率、倒车与首末朝向开关、
//                      机器人足迹检查点列表等），从 ROS2 参数服务器读取；
//   OptimizerParams —— Ceres 求解器参数（线性求解器类型、迭代上限、
//                      函数/梯度/参数收敛容差、调试开关）。
// 两者均提供 get(node, name) 方法：以 "插件名." 为前缀声明并读取参数，
// 未配置时使用默认值。
// ============================================================================

#ifndef NAV2_CONSTRAINED_SMOOTHER__OPTIONS_HPP_
#define NAV2_CONSTRAINED_SMOOTHER__OPTIONS_HPP_

#include <map>
#include <string>
#include <vector>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "ceres/ceres.h"

namespace nav2_constrained_smoother
{

/**
 * @struct nav2_smac_planner::SmootherParams
 * @brief Parameters for the smoother cost function
 */
struct SmootherParams
{
  /**
   * @brief A constructor for nav2_smac_planner::SmootherParams
   */
  SmootherParams()
  {
  }

  /**
   * @brief Get params from ROS parameter
   * @param node_ Ptr to node
   * @param name Name of plugin
   */
  void get(rclcpp_lifecycle::LifecycleNode * node, const std::string & name)
  {
    std::string local_name = name + std::string(".");

    // Smoother params
    // 最小转弯半径 -> 换算为最大曲率约束 max_curvature = 1/R
    double minimum_turning_radius;
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".minimum_turning_radius", rclcpp::ParameterValue(0.4));
    node->get_parameter(name + ".minimum_turning_radius", minimum_turning_radius);
    max_curvature = 1.0f / minimum_turning_radius;
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_curve", rclcpp::ParameterValue(30.0));
    node->get_parameter(local_name + "w_curve", curvature_weight);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_cost", rclcpp::ParameterValue(0.015));
    node->get_parameter(local_name + "w_cost", costmap_weight);
    double cost_cusp_multiplier;
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_cost_cusp_multiplier", rclcpp::ParameterValue(3.0));
    node->get_parameter(local_name + "w_cost_cusp_multiplier", cost_cusp_multiplier);
    cusp_costmap_weight = costmap_weight * cost_cusp_multiplier;
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "cusp_zone_length", rclcpp::ParameterValue(2.5));
    node->get_parameter(local_name + "cusp_zone_length", cusp_zone_length);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_dist", rclcpp::ParameterValue(0.0));
    node->get_parameter(local_name + "w_dist", distance_weight);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_smooth", rclcpp::ParameterValue(2000000.0));
    node->get_parameter(local_name + "w_smooth", smooth_weight);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "cost_check_points", rclcpp::ParameterValue(std::vector<double>()));
    node->get_parameter(local_name + "cost_check_points", cost_check_points);
    // cost_check_points 必须是 [x1, y1, weight1, x2, y2, weight2, ...] 形式
    // （每 3 个数一组：机器人本体坐标系下的检查点位置及其权重），
    // 数量不是 3 的倍数则视为非法参数并抛异常
    if (cost_check_points.size() % 3 != 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "constrained_smoother"),
        "cost_check_points parameter must contain values as follows: "
        "[x1, y1, weight1, x2, y2, weight2, ...]");
      throw std::runtime_error("Invalid parameter: cost_check_points");
    }
    // normalize check point weights so that their sum == 1.0
    // 归一化所有检查点权重，使其总和为 1（保证足迹避障代价与点数无关）
    double check_point_weights_sum = 0.0;
    for (size_t i = 2u; i < cost_check_points.size(); i += 3) {
      check_point_weights_sum += cost_check_points[i];
    }
    for (size_t i = 2u; i < cost_check_points.size(); i += 3) {
      cost_check_points[i] /= check_point_weights_sum;
    }
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "path_downsampling_factor", rclcpp::ParameterValue(1));
    node->get_parameter(local_name + "path_downsampling_factor", path_downsampling_factor);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "path_upsampling_factor", rclcpp::ParameterValue(1));
    node->get_parameter(local_name + "path_upsampling_factor", path_upsampling_factor);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "reversing_enabled", rclcpp::ParameterValue(true));
    node->get_parameter(local_name + "reversing_enabled", reversing_enabled);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "keep_goal_orientation", rclcpp::ParameterValue(true));
    node->get_parameter(local_name + "keep_goal_orientation", keep_goal_orientation);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "keep_start_orientation", rclcpp::ParameterValue(true));
    node->get_parameter(local_name + "keep_start_orientation", keep_start_orientation);
  }

  double smooth_weight{0.0};        // w_smooth：平滑项权重
  double costmap_weight{0.0};       // w_cost：代价地图（避障）项权重
  double cusp_costmap_weight{0.0};  // 尖点区域的代价地图权重（w_cost * 乘数）
  double cusp_zone_length{0.0};     // 尖点影响区长度（米）
  double distance_weight{0.0};      // w_dist：数据保真项权重（默认 0 不启用）
  double curvature_weight{0.0};     // w_curve：曲率约束项权重
  double max_curvature{0.0};        // 最大曲率 = 1 / 最小转弯半径
  double max_time{10.0};  // adjusted by action goal, not by parameters
                          // 求解时间预算（来自动作目标，而非参数文件）
  int path_downsampling_factor{1};  // 路径降采样倍率（优化前抽稀）
  int path_upsampling_factor{1};    // 路径上采样倍率（优化后贝塞尔插值加密）
  bool reversing_enabled{true};     // 是否允许倒车（检测尖点/倒车段）
  bool keep_goal_orientation{true}; // 是否保持终点朝向
  bool keep_start_orientation{true};// 是否保持起点朝向
  std::vector<double> cost_check_points{};  // 机器人足迹检查点 [x,y,w,...]（本体坐标系）
};

/**
 * @struct nav2_smac_planner::OptimizerParams
 * @brief Parameters for the ceres optimizer
 */
struct OptimizerParams
{
  OptimizerParams()
  : debug(false),
    max_iterations(50),
    param_tol(1e-8),
    fn_tol(1e-6),
    gradient_tol(1e-10)
  {
  }

  /**
   * @brief Get params from ROS parameter
   * @param node_ Ptr to node
   * @param name Name of plugin
   */
  void get(rclcpp_lifecycle::LifecycleNode * node, const std::string & name)
  {
    std::string local_name = name + std::string(".optimizer.");

    // Optimizer params
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "linear_solver_type", rclcpp::ParameterValue("SPARSE_NORMAL_CHOLESKY"));
    node->get_parameter(local_name + "linear_solver_type", linear_solver_type);
    if (solver_types.find(linear_solver_type) == solver_types.end()) {
      std::stringstream valid_types_str;
      for (auto type = solver_types.begin(); type != solver_types.end(); type++) {
        if (type != solver_types.begin()) {
          valid_types_str << ", ";
        }
        valid_types_str << type->first;
      }
      RCLCPP_ERROR(
        rclcpp::get_logger("constrained_smoother"),
        "Invalid linear_solver_type. Valid values are %s", valid_types_str.str().c_str());
      throw std::runtime_error("Invalid parameter: linear_solver_type");
    }
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "param_tol", rclcpp::ParameterValue(1e-15));
    node->get_parameter(local_name + "param_tol", param_tol);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "fn_tol", rclcpp::ParameterValue(1e-7));
    node->get_parameter(local_name + "fn_tol", fn_tol);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "gradient_tol", rclcpp::ParameterValue(1e-10));
    node->get_parameter(local_name + "gradient_tol", gradient_tol);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "max_iterations", rclcpp::ParameterValue(100));
    node->get_parameter(local_name + "max_iterations", max_iterations);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "debug_optimizer", rclcpp::ParameterValue(false));
    node->get_parameter(local_name + "debug_optimizer", debug);
  }

  // 可选的 Ceres 线性求解器类型映射（名字字符串 -> 枚举值）
  const std::map<std::string, ceres::LinearSolverType> solver_types = {
    {"DENSE_QR", ceres::DENSE_QR},
    {"SPARSE_NORMAL_CHOLESKY", ceres::SPARSE_NORMAL_CHOLESKY}};

  bool debug;                     // 是否输出求解器调试信息
  std::string linear_solver_type; // 线性求解器类型名
  int max_iterations;  // Ceres default: 50
                       // 最大迭代次数（Ceres 默认 50）

  double param_tol;  // Ceres default: 1e-8   参数变化收敛容差
  double fn_tol;  // Ceres default: 1e-6      目标函数变化收敛容差
  double gradient_tol;  // Ceres default: 1e-10 梯度范数收敛容差
};

}  // namespace nav2_constrained_smoother

#endif  // NAV2_CONSTRAINED_SMOOTHER__OPTIONS_HPP_
