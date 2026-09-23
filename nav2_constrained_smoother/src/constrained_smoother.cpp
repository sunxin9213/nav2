// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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
// ConstrainedSmoother（带约束平滑器）插件的实现。
// 模块定位：nav2_constrained_smoother 是 Nav2 平滑服务器的算法插件之一，
// 实现改进的 elastic band（弹性带）路径平滑，底层使用 Google Ceres
// 非线性最小二乘求解器优化路径点。与 SimpleSmoother 的朴素梯度法相比，
// 它显式加入了曲率约束（最小转弯半径）、代价地图避障项、尖点（cusp，
// 前进/倒车切换点）区域处理、路径降采样/三次贝塞尔上采样等机制，
// 适合差速/阿克曼等有运动学约束的机器人。
//
// 本文件职责（插件适配层）：
//   configure：读取参数并初始化内部 Smoother 优化器；
//   smooth：将 nav_msgs/Path 转换为内部格式 (x, y, 方向符号)，
//           调用 Ceres 优化，再把优化结果转回 ROS 路径消息。
// 核心算法在 include/nav2_constrained_smoother/smoother.hpp 中实现。
// ============================================================================

#include <algorithm>
#include <string>
#include <memory>
#include <utility>
#include <vector>

#include "nav2_constrained_smoother/constrained_smoother.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "tf2/utils.h"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT

namespace nav2_constrained_smoother
{

// ---------------------------------------------------------------------------
// 插件配置回调（SmootherServer 在 on_configure 时调用）：
//   1. 锁定父节点 WeakPtr，保存代价地图订阅器、TF 缓冲、插件名与日志器；
//   2. 创建内部优化器 Smoother 实例；
//   3. 分别读取优化器参数（Ceres 求解器配置）与平滑参数（各项权重等）；
//   4. 用优化器参数初始化 Ceres 求解选项。
// ---------------------------------------------------------------------------
void ConstrainedSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  costmap_sub_ = costmap_sub;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  smoother_ = std::make_unique<nav2_constrained_smoother::Smoother>();
  optimizer_params_.get(node.get(), name);      // 读取 "插件名.optimizer.*" 参数
  smoother_params_.get(node.get(), name);       // 读取 "插件名.*" 平滑参数
  smoother_->initialize(optimizer_params_);     // 初始化 Ceres 求解选项
}

// 清理回调：本插件无持久资源，仅打印日志
void ConstrainedSmoother::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up smoother: %s of type"
    " nav2_constrained_smoother::ConstrainedSmoother",
    plugin_name_.c_str());
}

// 激活回调：仅打印日志（优化器为无状态对象，无需激活操作）
void ConstrainedSmoother::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating smoother: %s of type "
    "nav2_constrained_smoother::ConstrainedSmoother",
    plugin_name_.c_str());
}

// 停用回调：仅打印日志
void ConstrainedSmoother::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating smoother: %s of type "
    "nav2_constrained_smoother::ConstrainedSmoother",
    plugin_name_.c_str());
}

// ---------------------------------------------------------------------------
// 平滑主入口：把 ROS 路径消息 <-> 内部优化格式进行双向转换并调用优化器。
// 内部路径点格式为 Eigen::Vector3d：(x, y, dir)，其中 dir 取 +1（前进）
// 或 -1（倒车），即用符号编码该点之后一段的运动方向。
// ---------------------------------------------------------------------------
bool ConstrainedSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time)
{
  // 少于 2 个点无法平滑，直接视为成功返回
  if (path.poses.size() < 2) {
    return true;
  }

  // populate smoother input with (x, y, forward/reverse dir)
  // 将 ROS 路径转为内部格式 (x, y, 前进/倒车方向符号)
  std::vector<Eigen::Vector3d> path_world;
  path_world.reserve(path.poses.size());
  // smoother keeps record of start/end orientations so that it
  // can use them in the final path, preventing degradation of these (often important) values
  // 记录路径首/末点的原始朝向单位向量，优化后回填，防止起终点朝向被平滑破坏
  Eigen::Vector2d start_dir;
  Eigen::Vector2d end_dir;
  for (size_t i = 0; i < path.poses.size(); i++) {
    auto & pose = path.poses[i].pose;
    // 从四元数中提取偏航角，并转为单位方向向量 (cos, sin)
    double angle = tf2::getYaw(pose.orientation);
    Eigen::Vector2d orientation(cos(angle), sin(angle));
    if (i == path.poses.size() - 1) {
      // Note: `reversing` indicates the direction of the segment after the point and
      // there is no segment after the last point. Most probably the value is irrelevant, but
      // copying it from the last but one point, just to make it defined...
      // 末点之后没有路径段，方向符号沿用倒数第二个点的值（仅为定义完整）
      path_world.emplace_back(pose.position.x, pose.position.y, path_world.back()[2]);
      end_dir = orientation;
    } else {
      // 当前点到下一点的运动向量
      auto & pos_next = path.poses[i + 1].pose.position;
      Eigen::Vector2d mvmt(pos_next.x - pose.position.x, pos_next.y - pose.position.y);
      // robot is considered reversing when angle between its orientation and movement direction
      // is more than 90 degrees (i.e. dot product is less than 0)
      // 机器人朝向与运动方向点积 < 0（夹角超过 90 度）即判定为倒车；
      // reversing_enabled=false 时禁用倒车检测（全部按前进处理）
      bool reversing = smoother_params_.reversing_enabled && orientation.dot(mvmt) < 0;
      // we transform boolean value of "reversing" into sign of movement direction (+1 or -1)
      // to simplify further computations
      // 把布尔"倒车"编码为方向符号 +1/-1，便于后续计算
      path_world.emplace_back(pose.position.x, pose.position.y, reversing ? -1 : 1);
      if (i == 0) {
        start_dir = orientation;  // 记录起点朝向
      } else if (i == 1 && !smoother_params_.keep_start_orientation) {
        // overwrite start forward/reverse when orientation was set to be ignored
        // note: start_dir is overwritten inside Smoother::upsampleAndPopulate() method
        // 若不保留起点朝向，用第 1 个点的方向符号覆盖起点的符号，
        // 使起点方向由后续路径走势决定（真正覆盖发生在 upsampleAndPopulate 中）
        path_world[0][2] = path_world.back()[2];
      }
    }
  }

  // 动作目标中的最大平滑时长写入参数结构（不来自参数文件）
  smoother_params_.max_time = max_time.seconds();

  // Smooth plan
  // 获取全局代价地图并执行 Ceres 优化平滑
  auto costmap = costmap_sub_->getCostmap();
  if (!smoother_->smooth(path_world, start_dir, end_dir, costmap.get(), smoother_params_)) {
    // 优化失败（Ceres 未找到可用解）：告警并抛出规划器异常，
    // 上层 SmootherServer 会捕获并终止该动作
    RCLCPP_WARN(
      logger_,
      "%s: failed to smooth plan, Ceres could not find a usable solution to optimize.",
      plugin_name_.c_str());
    throw new nav2_core::PlannerException(
            "Failed to smooth plan, Ceres could not find a usable solution.");
  }

  // populate final path
  // 将优化后的内部格式 (x, y, theta) 转回 ROS PoseStamped 路径
  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.poses.front().header;
  path.poses.clear();
  path.poses.reserve(path_world.size());
  for (auto & pw : path_world) {
    pose.pose.position.x = pw[0];
    pose.pose.position.y = pw[1];
    // 由平面偏航角构造四元数（仅绕 Z 轴旋转）：
    // z = sin(theta/2), w = cos(theta/2)
    pose.pose.orientation.z = sin(pw[2] / 2);
    pose.pose.orientation.w = cos(pw[2] / 2);

    path.poses.push_back(pose);
  }

  return true;
}

}  // namespace nav2_constrained_smoother

// Register this smoother as a nav2_core plugin
// 通过 pluginlib 宏导出为 nav2_core::Smoother 插件，供平滑服务器动态加载
PLUGINLIB_EXPORT_CLASS(
  nav2_constrained_smoother::ConstrainedSmoother,
  nav2_core::Smoother)
