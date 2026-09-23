// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_

#include <algorithm>
#include <chrono>
#include <string>
#include <limits>
#include <memory>
#include <vector>

#include <xtensor/xarray.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "angles/angles.h"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_core/goal_checker.hpp"

#include "nav2_mppi_controller/models/optimizer_settings.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav2_mppi_controller/critic_data.hpp"

namespace mppi::utils
{
// 工具集：包含消息构造、路径转张量、角度处理、路径点搜索、
// 代价检查、Savitzky-Golay 平滑与路径掉头检测等 MPPI 各部件共用的辅助函数
using xt::evaluation_strategy::immediate;  // 令 xtensor 表达式立即求值（非惰性）

/**
 * @brief Convert data into pose
 * @param x X position
 * @param y Y position
 * @param z Z position
 * @return Pose object
 */
// 构造单位朝向（w=1 无旋转）的位姿消息，用于可视化 Marker
inline geometry_msgs::msg::Pose createPose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1;   // 四元数 (0,0,0,1) 表示无旋转
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  return pose;
}

/**
 * @brief Convert data into scale
 * @param x X scale
 * @param y Y scale
 * @param z Z scale
 * @return Scale object
 */
// 构造可视化 Marker 的尺寸向量（米）
inline geometry_msgs::msg::Vector3 createScale(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 scale;
  scale.x = x;
  scale.y = y;
  scale.z = z;
  return scale;
}

/**
 * @brief Convert data into color
 * @param r Red component
 * @param g Green component
 * @param b Blue component
 * @param a Alpha component (transparency)
 * @return Color object
 */
// 构造 RGBA 颜色（a=0 全透明，a=1 不透明）
inline std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

/**
 * @brief Convert data into a Maarker
 * @param id Marker ID
 * @param pose Marker pose
 * @param scale Marker scale
 * @param color Marker color
 * @param frame Reference frame to use
 * @return Visualization Marker
 */
// 组装一个球形（SPHERE）可视化 Marker
inline visualization_msgs::msg::Marker createMarker(
  int id, const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color, const std::string & frame_id, const std::string & ns)
{
  using visualization_msgs::msg::Marker;
  Marker marker;
  marker.header.frame_id = frame_id;   // 显示坐标系
  marker.header.stamp = rclcpp::Time(0, 0);  // 0 表示使用最新可用变换
  marker.ns = ns;                      // 命名空间：同 ns 的 marker 可整体删除
  marker.id = id;                      // 同 ns 内唯一标识
  marker.type = Marker::SPHERE;        // 球体形状
  marker.action = Marker::ADD;         // 添加/更新

  marker.pose = pose;
  marker.scale = scale;
  marker.color = color;
  return marker;
}

/**
 * @brief Convert data into TwistStamped
 * @param vx X velocity
 * @param wz Angular velocity
 * @param stamp Timestamp
 * @param frame Reference frame to use
 */
// 将 (vx, wz) 打包为带时间戳/坐标系的速度指令消息（差速模型）
inline geometry_msgs::msg::TwistStamped toTwistStamped(
  float vx, float wz, const builtin_interfaces::msg::Time & stamp, const std::string & frame)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame;
  twist.header.stamp = stamp;
  twist.twist.linear.x = vx;
  twist.twist.angular.z = wz;

  return twist;
}

/**
 * @brief Convert data into TwistStamped
 * @param vx X velocity
 * @param vy Y velocity
 * @param wz Angular velocity
 * @param stamp Timestamp
 * @param frame Reference frame to use
 */
// 全向模型重载：额外填入侧向速度 vy
inline geometry_msgs::msg::TwistStamped toTwistStamped(
  float vx, float vy, float wz, const builtin_interfaces::msg::Time & stamp,
  const std::string & frame)
{
  auto twist = toTwistStamped(vx, wz, stamp, frame);  // 复用二参版本
  twist.twist.linear.y = vy;

  return twist;
}

/**
 * @brief Convert path to a tensor
 * @param path Path to convert
 * @return Path tensor
 */
// 将 ROS Path 消息转换为 models::Path 张量（x/y/yaws 三个一维张量），
// 便于评价器做向量化距离计算
inline models::Path toTensor(const nav_msgs::msg::Path & path)
{
  auto result = models::Path{};
  result.reset(path.poses.size());  // 按路径点数分配张量内存

  for (size_t i = 0; i < path.poses.size(); ++i) {
    result.x(i) = path.poses[i].pose.position.x;
    result.y(i) = path.poses[i].pose.position.y;
    result.yaws(i) = tf2::getYaw(path.poses[i].pose.orientation);  // 四元数转偏航角
  }

  return result;
}

/**
 * @brief Check if the robot pose is within the Goal Checker's tolerances to goal
 * @param global_checker Pointer to the goal checker
 * @param robot Pose of robot
 * @param path Path to retreive goal pose from
 * @return bool If robot is within goal checker tolerances to the goal
 */
// 判断机器人是否已进入目标检查器的位置容差内（用于接近终点时的降速等逻辑）
inline bool withinPositionGoalTolerance(
  nav2_core::GoalChecker * goal_checker,
  const geometry_msgs::msg::Pose & robot,
  const models::Path & path)
{
  const auto goal_idx = path.x.shape(0) - 1;  // 路径末点即目标点
  const auto goal_x = path.x(goal_idx);
  const auto goal_y = path.y(goal_idx);

  if (goal_checker) {
    // 从目标检查器取得位置/速度容差
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    goal_checker->getTolerances(pose_tolerance, velocity_tolerance);

    // 用距离平方比较，避免开方
    const auto pose_tolerance_sq = pose_tolerance.position.x * pose_tolerance.position.x;

    auto dx = robot.position.x - goal_x;
    auto dy = robot.position.y - goal_y;

    auto dist_sq = dx * dx + dy * dy;

    if (dist_sq < pose_tolerance_sq) {
      return true;
    }
  }

  return false;
}

/**
 * @brief Check if the robot pose is within tolerance to the goal
 * @param pose_tolerance Pose tolerance to use
 * @param robot Pose of robot
 * @param path Path to retreive goal pose from
 * @return bool If robot is within tolerance to the goal
 */
// 上一个函数的标量容差版本（不依赖 goal checker 对象）
inline bool withinPositionGoalTolerance(
  float pose_tolerance,
  const geometry_msgs::msg::Pose & robot,
  const models::Path & path)
{
  const auto goal_idx = path.x.shape(0) - 1;
  const auto goal_x = path.x(goal_idx);
  const auto goal_y = path.y(goal_idx);

  const auto pose_tolerance_sq = pose_tolerance * pose_tolerance;

  auto dx = robot.position.x - goal_x;
  auto dy = robot.position.y - goal_y;

  auto dist_sq = dx * dx + dy * dy;

  if (dist_sq < pose_tolerance_sq) {
    return true;
  }

  return false;
}

/**
  * @brief normalize
  * Normalizes the angle to be -M_PI circle to +M_PI circle
  * It takes and returns radians.
  * @param angles Angles to normalize
  * @return normalized angles
  */
// 张量化角度归一化：把任意角折算到 (-pi, pi] 区间
template<typename T>
auto normalize_angles(const T & angles)
{
  // 第一步 fmod 折到 [0, 2pi) 或 (-2pi, 0]；第二步平移到 (-pi, pi]
  auto && theta = xt::eval(xt::fmod(angles + M_PI, 2.0 * M_PI));
  return xt::eval(xt::where(theta <= 0.0, theta + M_PI, theta - M_PI));
}

/**
  * @brief shortest_angular_distance
  *
  * Given 2 angles, this returns the shortest angular
  * difference.  The inputs and ouputs are of course radians.
  *
  * The result
  * would always be -pi <= result <= pi.  Adding the result
  * to "from" will always get you an equivelent angle to "to".
  * @param from Start angle
  * @param to End angle
  * @return Shortest distance between angles
  */
// 最短角差：返回从 from 转到 to 的最小旋转量（张量化）
template<typename F, typename T>
auto shortest_angular_distance(
  const F & from,
  const T & to)
{
  return normalize_angles(to - from);
}

/**
 * @brief Evaluate furthest point idx of data.path which is
 * nearset to some trajectory in data.trajectories
 * @param data Data to use
 * @return Idx of furthest path point reached by a set of trajectories
 */
// 计算整批采样轨迹能"到达"的路径最远点索引：
// 对每条轨迹取其终点，找路径上距终点最近的点索引，
// 再在所有轨迹中取最大索引。该索引被多个评价器（如 path_align、
// path_follow）用作参考基准，避免拿机器人身后/远处的路径点打分
inline size_t findPathFurthestReachedPoint(const CriticData & data)
{
  // 提取每条轨迹的终点 (batch, 1) 形状视图
  const auto traj_x = xt::view(data.trajectories.x, xt::all(), -1, xt::newaxis());
  const auto traj_y = xt::view(data.trajectories.y, xt::all(), -1, xt::newaxis());

  // 广播计算：路径点到每条轨迹终点的 dx/dy 及距离平方 (batch × path_len)
  const auto dx = data.path.x - traj_x;
  const auto dy = data.path.y - traj_y;

  const auto dists = dx * dx + dy * dy;

  size_t max_id_by_trajectories = 0, min_id_by_path = 0;
  float min_distance_by_path = std::numeric_limits<float>::max();
  float cur_dist = 0.0f;

  // 逐轨迹：先找该轨迹终点最近的路径点，再取所有轨迹的最大索引
  for (size_t i = 0; i < dists.shape(0); i++) {
    min_id_by_path = 0;
    min_distance_by_path = std::numeric_limits<float>::max();
    for (size_t j = 0; j < dists.shape(1); j++) {
      cur_dist = dists(i, j);
      if (cur_dist < min_distance_by_path) {
        min_distance_by_path = cur_dist;
        min_id_by_path = j;
      }
    }
    max_id_by_trajectories = std::max(max_id_by_trajectories, min_id_by_path);
  }
  return max_id_by_trajectories;
}

/**
 * @brief Evaluate closest point idx of data.path which is
 * nearset to the start of the trajectory in data.trajectories
 * @param data Data to use
 * @return Idx of closest path point at start of the trajectories
 */
// 计算路径上距轨迹起点最近的点索引（即机器人当前对应的路径位置）
inline size_t findPathTrajectoryInitialPoint(const CriticData & data)
{
  // First point should be the same for all trajectories from initial conditions
  // 所有轨迹共享同一初始条件，取第 0 条轨迹的第 0 个点即可
  const auto dx = data.path.x - data.trajectories.x(0, 0);
  const auto dy = data.path.y - data.trajectories.y(0, 0);
  const auto dists = dx * dx + dy * dy;

  // 线性扫描找最近路径点
  float min_distance_by_path = std::numeric_limits<float>::max();
  size_t min_id = 0;
  for (size_t j = 0; j < dists.shape(0); j++) {
    if (dists(j) < min_distance_by_path) {
      min_distance_by_path = dists(j);
      min_id = j;
    }
  }

  return min_id;
}

/**
 * @brief evaluate path furthest point if it is not set
 * @param data Data to use
 */
// 惰性计算：首个需要该数据的评价器触发一次性填充，其余评价器复用
inline void setPathFurthestPointIfNotSet(CriticData & data)
{
  if (!data.furthest_reached_path_point) {
    data.furthest_reached_path_point = findPathFurthestReachedPoint(data);
  }
}

/**
 * @brief evaluate path costs
 * @param data Data to use
 */
// 逐路径点查取代价地图代价值，判定该点是否"可通行"：
// LETHAL/INSCRIBED 视为无效；NO_INFORMATION 依是否允许未知区域而定
inline void findPathCosts(
  CriticData & data,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto * costmap = costmap_ros->getCostmap();
  unsigned int map_x, map_y;
  const size_t path_segments_count = data.path.x.shape(0) - 1;  // 段数 = 点数-1
  data.path_pts_valid = std::vector<bool>(path_segments_count, false);
  for (unsigned int idx = 0; idx < path_segments_count; idx++) {
    const auto path_x = data.path.x(idx);
    const auto path_y = data.path.y(idx);
    // 世界坐标 -> 地图像素；越界视为无效
    if (!costmap->worldToMap(path_x, path_y, map_x, map_y)) {
      (*data.path_pts_valid)[idx] = false;
      continue;
    }

    // 按代价值分级判定
    switch (costmap->getCost(map_x, map_y)) {
      using namespace nav2_costmap_2d; // NOLINT
      case (LETHAL_OBSTACLE):                 // 致命障碍：碰撞
        (*data.path_pts_valid)[idx] = false;
        continue;
      case (INSCRIBED_INFLATED_OBSTACLE):     // 内切膨胀：必定碰撞
        (*data.path_pts_valid)[idx] = false;
        continue;
      case (NO_INFORMATION):                  // 未知区域：看是否允许穿越
        const bool is_tracking_unknown =
          costmap_ros->getLayeredCostmap()->isTrackingUnknown();
        (*data.path_pts_valid)[idx] = is_tracking_unknown ? true : false;
        continue;
    }

    (*data.path_pts_valid)[idx] = true;  // 其余情况均有效
  }
}

/**
 * @brief evaluate path costs if it is not set
 * @param data Data to use
 */
// 惰性计算：路径点有效性只算一次，各评价器共享
inline void setPathCostsIfNotSet(
  CriticData & data,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  if (!data.path_pts_valid) {
    findPathCosts(data, costmap_ros);
  }
}

/**
 * @brief evaluate angle from pose (have angle) to point (no angle)
 * @param pose pose
 * @param point_x Point to find angle relative to X axis
 * @param point_y Point to find angle relative to Y axis
 * @param forward_preference If reversing direction is valid
 * @return Angle between two points
 */
// 计算位姿朝向与"指向目标点方向"的夹角（弧度，绝对值）。
// forward_preference=false 时允许倒车：也考虑反向 180° 的情况
inline float posePointAngle(
  const geometry_msgs::msg::Pose & pose, double point_x, double point_y, bool forward_preference)
{
  float pose_x = pose.position.x;
  float pose_y = pose.position.y;
  float pose_yaw = tf2::getYaw(pose.orientation);

  // 目标方向的方位角
  float yaw = atan2f(point_y - pose_y, point_x - pose_x);

  // If no forward preference, return smallest angle either in heading or 180 of heading
  // 无前进偏好：取"正对目标"与"倒车对准目标"两个夹角中的较小者
  if (!forward_preference) {
    return std::min(
      fabs(angles::shortest_angular_distance(yaw, pose_yaw)),
      fabs(angles::shortest_angular_distance(yaw, angles::normalize_angle(pose_yaw + M_PI))));
  }

  // 有前进偏好：只考虑正向对准
  return fabs(angles::shortest_angular_distance(yaw, pose_yaw));
}

/**
 * @brief Apply Savisky-Golay filter to optimal trajectory
 * @param control_sequence Sequence to apply filter to
 * @param control_history Recent set of controls for edge-case handling
 * @param Settings Settings to use
 */
// Savitzky-Golay 平滑滤波：对最优控制序列做 9 点二次多项式平滑，
// 抑制 MPPI 加权平均后残余的高频抖动（改善乘坐舒适性、减少电机磨损）。
// 序列头部不足 9 点的窗口用历史控制量补齐，尾部用重复值补齐
inline void savitskyGolayFilter(
  models::ControlSequence & control_sequence,
  std::array<mppi::models::Control, 4> & control_history,
  const models::OptimizerSettings & settings)
{
  // Savitzky-Golay Quadratic, 9-point Coefficients
  // 二次多项式 9 点滤波系数（归一化到 1/231，保持信号幅度不失真）
  xt::xtensor<float, 1> filter = {-21.0, 14.0, 39.0, 54.0, 59.0, 54.0, 39.0, 14.0, -21.0};
  filter /= 231.0;

  const unsigned int num_sequences = control_sequence.vx.shape(0) - 1;  // 有效时间步数

  // Too short to smooth meaningfully
  // 序列过短，平滑无意义（窗口需要前后各 4 点）
  if (num_sequences < 20) {
    return;
  }

  // 内积：9 点窗口与滤波系数的加权和
  auto applyFilter = [&](const xt::xtensor<float, 1> & data) -> float {
      return xt::sum(data * filter, {0}, immediate)();
    };

  // 对单个控制序列（vx/vy/wz 之一）沿时间轴应用滤波
  auto applyFilterOverAxis =
    [&](xt::xtensor<float, 1> & sequence,
      const float hist_0, const float hist_1, const float hist_2, const float hist_3) -> void
    {
      unsigned int idx = 0;
      // 头部 4 个点：窗口左端不足，用上周期历史控制量 hist_0..hist_3 补齐
      sequence(idx) = applyFilter(
      {
        hist_0,
        hist_1,
        hist_2,
        hist_3,
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 4)});

      idx++;
      sequence(idx) = applyFilter(
      {
        hist_1,
        hist_2,
        hist_3,
        sequence(idx - 1),  // 已平滑的前一点参与后续窗口
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 4)});

      idx++;
      sequence(idx) = applyFilter(
      {
        hist_2,
        hist_3,
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 4)});

      idx++;
      sequence(idx) = applyFilter(
      {
        hist_3,
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 4)});

      // 主体：完整 9 点滑动窗口（对称，无边界问题）
      for (idx = 4; idx != num_sequences - 4; idx++) {
        sequence(idx) = applyFilter(
        {
          sequence(idx - 4),
          sequence(idx - 3),
          sequence(idx - 2),
          sequence(idx - 1),
          sequence(idx),
          sequence(idx + 1),
          sequence(idx + 2),
          sequence(idx + 3),
          sequence(idx + 4)});
      }

      // 尾部 4 个点：右端不足，用最近的有效点重复补齐（边缘复制）
      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 4),
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 3)});

      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 4),
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 2),
        sequence(idx + 2)});

      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 4),
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 1),
        sequence(idx + 1),
        sequence(idx + 1)});

      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 4),
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx),
        sequence(idx),
        sequence(idx),
        sequence(idx)});
    };

  // Filter trajectories
  // 对 vx / vy / wz 三个控制通道分别平滑（历史量按通道对应）
  applyFilterOverAxis(
    control_sequence.vx, control_history[0].vx,
    control_history[1].vx, control_history[2].vx, control_history[3].vx);
  applyFilterOverAxis(
    control_sequence.vy, control_history[0].vy,
    control_history[1].vy, control_history[2].vy, control_history[3].vy);
  applyFilterOverAxis(
    control_sequence.wz, control_history[0].wz,
    control_history[1].wz, control_history[2].wz, control_history[3].wz);

  // Update control history
  // 滚动更新历史：保留最近 4 个控制量供下一周期滤波补边界。
  // 若启用了控制序列时移（shift），序列起点为第 1 个元素
  unsigned int offset = settings.shift_control_sequence ? 1 : 0;
  control_history[0] = control_history[1];
  control_history[1] = control_history[2];
  control_history[2] = control_history[3];
  control_history[3] = {
    control_sequence.vx(offset),
    control_sequence.vy(offset),
    control_sequence.wz(offset)};
}

/**
 * @brief Find the iterator of the first pose at which there is an inversion on the path,
 * @param path to check for inversion
 * @return the first point after the inversion found in the path
 */
// 检测路径上首次"方向反转"（cusp/掉头点）：
// 相邻两段方向向量的点积 < 0 说明发生折返。
// 返回掉头点之后的第一个位姿索引
inline unsigned int findFirstPathInversion(nav_msgs::msg::Path & path)
{
  // At least 3 poses for a possible inversion
  // 至少 3 个点才能构成两个方向向量
  if (path.poses.size() < 3) {
    return path.poses.size();  // 无掉头：返回越界值表示不存在
  }

  // Iterating through the path to determine the position of the path inversion
  for (unsigned int idx = 1; idx < path.poses.size() - 1; ++idx) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    // 前段向量 OA = P[idx] - P[idx-1]，后段向量 AB = P[idx+1] - P[idx]
    float oa_x = path.poses[idx].pose.position.x -
      path.poses[idx - 1].pose.position.x;
    float oa_y = path.poses[idx].pose.position.y -
      path.poses[idx - 1].pose.position.y;
    float ab_x = path.poses[idx + 1].pose.position.x -
      path.poses[idx].pose.position.x;
    float ab_y = path.poses[idx + 1].pose.position.y -
      path.poses[idx].pose.position.y;

    // Checking for the existance of cusp, in the path, using the dot product.
    // 点积为负 => 夹角大于 90° => 发生折返
    float dot_product = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_product < 0.0) {
      return idx + 1;  // 返回掉头点之后的位姿
    }
  }

  return path.poses.size();  // 整条路径无折返
}

/**
 * @brief Find and remove poses after the first inversion in the path
 * @param path to check for inversion
 * @return The location of the inversion, return 0 if none exist
 */
// 截断路径：删除首次掉头点之后的所有位姿（就地修改）。
// 返回掉头位置索引；0 表示无掉头（此时路径不变）
inline unsigned int removePosesAfterFirstInversion(nav_msgs::msg::Path & path)
{
  nav_msgs::msg::Path cropped_path = path;  // 先复制，避免在查找中修改原路径
  const unsigned int first_after_inversion = findFirstPathInversion(cropped_path);
  if (first_after_inversion == path.poses.size()) {
    return 0u;  // 无掉头，无需截断
  }

  // 删除 [first_after_inversion, end) 的位姿，写回原路径
  cropped_path.poses.erase(
    cropped_path.poses.begin() + first_after_inversion, cropped_path.poses.end());
  path = cropped_path;
  return first_after_inversion;
}

/**
 * @brief Compare to trajectory points to find closest path point along integrated distances
 * @param vec Vect to check
 * @return dist Distance to look for
 */
// 在弧长前缀和数组中二分查找最接近 dist 的索引：
// 用 lower_bound 定位下界，再比较左右邻居取更近者。
// init 允许从中间开始（已知下界的单调性）
inline size_t findClosestPathPt(const std::vector<float> & vec, float dist, size_t init = 0)
{
  // 第一个 >= dist 的位置
  auto iter = std::lower_bound(vec.begin() + init, vec.end(), dist);
  if (iter == vec.begin() + init) {
    return 0;  // dist 比所有元素都小 -> 取首元素
  }
  // 比较左侧（< dist）与右侧（>= dist）哪个更接近 dist
  if (dist - *(iter - 1) < *iter - dist) {
    return iter - 1 - vec.begin();
  }
  return iter - vec.begin();
}

}  // namespace mppi::utils

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_
