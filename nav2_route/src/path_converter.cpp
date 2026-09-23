// Copyright (c) 2025, Open Navigation LLC
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
// PathConverter（路径转换器）的实现。
// 模块定位：把 RoutePlanner 输出的稀疏路线（节点-边序列）转换为
// 稠密的 nav_msgs/Path，供下游控制器使用（与 planner_server 的
// 输出格式一致，发布到 "plan" 话题）。
// 核心功能：
//   1. densify 加密：沿每条边按 path_density 间距线性插值；
//   2. 拐角平滑（可选，smooth_corners）：相邻边转角处用 CornerArc
//      圆弧替代尖角（半径 smoothing_radius），先插值到圆弧起点、
//      再插值圆弧本身；
//   3. 重路由处理：若从当前边的最近点重路由，则从该点到边终点
//      也做插值，避免不必要的自由空间规划；
//   4. 朝向重建：每个路径点朝向取指向下一点的方向（末点沿最后一条边方向）。
// ============================================================================

#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <mutex>
#include <algorithm>

#include "nav2_route/path_converter.hpp"

namespace nav2_route
{

// ---------------------------------------------------------------------------
// 配置：读取三个参数并创建路径发布器：
//   path_density     路径点间距（米，默认 0.05）
//   smoothing_radius 拐角圆弧平滑半径（米，默认 1.0）
//   smooth_corners   是否启用拐角平滑（默认 false）
// ---------------------------------------------------------------------------
void PathConverter::configure(nav2_util::LifecycleNode::SharedPtr node)
{
  // Density to make path points
  nav2_util::declare_parameter_if_not_declared(
    node, "path_density", rclcpp::ParameterValue(0.05));
  density_ = static_cast<float>(node->get_parameter("path_density").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "smoothing_radius", rclcpp::ParameterValue(1.0));
  smoothing_radius_ = static_cast<float>(node->get_parameter("smoothing_radius").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "smooth_corners", rclcpp::ParameterValue(false));
  smooth_corners_ = node->get_parameter("smooth_corners").as_bool();

  // 路径发布器（与 planner_server 相同的 "plan" 话题）
  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("plan", 1);
  path_pub_->on_activate();
  logger_ = node->get_logger();
}

// ---------------------------------------------------------------------------
// 主入口：路线 -> 稠密路径。
// 逐边处理：正常边直接线性插值；启用平滑且拐角有效时，
// 用圆弧替代拐角（插值到弧起点 -> 插值弧 -> 下一边从弧终点开始）。
// 最后统一重建朝向并发布路径。
// ---------------------------------------------------------------------------
nav_msgs::msg::Path PathConverter::densify(
  const Route & route,
  const ReroutingState & rerouting_info,
  const std::string & frame,
  const rclcpp::Time & now)
{
  nav_msgs::msg::Path path;
  path.header.stamp = now;
  path.header.frame_id = frame;

  // If we're rerouting and covering the same previous edge to start,
  // the path should contain the relevant partial information along edge
  // to avoid unnecessary free-space planning where state is retained
  // 重路由：若起点在当前边内部，则从"边上的最近点"插值到边终点，
  // 保留该边的部分信息，避免从机器人位置做自由空间规划
  if (rerouting_info.curr_edge) {
    const Coordinates & start = rerouting_info.closest_pt_on_edge;
    const Coordinates & end = rerouting_info.curr_edge->end->coords;
    interpolateEdge(start.x, start.y, end.x, end.y, path.poses);
  }

  Coordinates start;
  Coordinates end;

  if (!route.edges.empty()) {
    start = route.edges[0]->start->coords;

    // Fill in path via route edges
    // 逐边插值（处理到倒数第二条边，最后一条边在循环外处理）
    for (unsigned int i = 0; i < route.edges.size() - 1; i++) {
      const EdgePtr edge = route.edges[i];
      const EdgePtr & next_edge = route.edges[i + 1];
      end = edge->end->coords;

      // 构造当前边与下一条边之间的拐角圆弧（半径 smoothing_radius）
      CornerArc corner_arc(start, end, next_edge->end->coords, smoothing_radius_);
      if (corner_arc.isCornerValid() && smooth_corners_) {
        // if an arc exists, end of the first edge is the start of the arc
        // 圆弧有效且启用平滑：第一条边的实际终点提前到圆弧起点
        end = corner_arc.getCornerStart();

        // interpolate to start of arc
        // 沿直线插值到圆弧起点
        interpolateEdge(start.x, start.y, end.x, end.y, path.poses);

        // interpolate arc
        // 插值圆弧本身（角步长 = density / radius，保证弧上点间距约等于 density）
        corner_arc.interpolateArc(density_ / smoothing_radius_, path.poses);

        // new start of next edge is end of smoothing arc
        // 下一条边从圆弧终点开始
        start = corner_arc.getCornerEnd();
      } else {
        // 不平滑（或圆弧无效）：直接直线插值整条边
        if (smooth_corners_) {
          // 启用了平滑但该拐角无法构造圆弧（如转角过大）：告警
          RCLCPP_WARN(
            logger_, "Unable to smooth corner between edge %i and edge %i", edge->edgeid,
            next_edge->edgeid);
        }
        interpolateEdge(start.x, start.y, end.x, end.y, path.poses);
        start = end;
      }
    }
  }

  if (route.edges.empty()) {
    // 空路线（起终点同节点）：路径只含单个点
    path.poses.push_back(utils::toMsg(route.start_node->coords.x, route.start_node->coords.y));
  } else {
    // 处理最后一条边：从当前 start 插值到路线终点
    interpolateEdge(
      start.x, start.y, route.edges.back()->end->coords.x,
      route.edges.back()->end->coords.y, path.poses);

    // 确保终点本身在路径中（interpolateEdge 不含终点）
    path.poses.push_back(
      utils::toMsg(route.edges.back()->end->coords.x, route.edges.back()->end->coords.y));
  }

  // Set path poses orientations for each point
  // 重建朝向：每个点朝向指向下一点的方向角
  for (size_t i = 0; i < path.poses.size() - 1; ++i) {
    const auto & pose = path.poses[i];
    const auto & next_pose = path.poses[i + 1];
    const double dx = next_pose.pose.position.x - pose.pose.position.x;
    const double dy = next_pose.pose.position.y - pose.pose.position.y;
    const double yaw = atan2(dy, dx);
    path.poses[i].pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  }

  // Set the last pose orientation to the last edge
  // 末点朝向：沿最后一条边的方向（保持进入终点的姿态）
  if (!route.edges.empty()) {
    const auto & last_edge = route.edges.back();
    const double dx = last_edge->end->coords.x - last_edge->start->coords.x;
    const double dy = last_edge->end->coords.y - last_edge->start->coords.y;
    path.poses.back().pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(atan2(dy, dx));
  }

  // publish path similar to planner server
  // 发布路径（与 planner_server 行为一致）
  path_pub_->publish(std::make_unique<nav_msgs::msg::Path>(path));

  return path;
}

// ---------------------------------------------------------------------------
// 线性插值一条边：从 (x0,y0) 到 (x1,y1)，按 density 间距采样。
// 点数 = ceil(长度/间距)；沿单位方向向量逐步推进。
// 注意：包含起点、不包含终点（终点由调用方或下一条边的起点覆盖）。
// ---------------------------------------------------------------------------
void PathConverter::interpolateEdge(
  float x0, float y0, float x1, float y1,
  std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  // Find number of points to populate by given density
  // 边长度与需要插值的点数
  const float mag = hypotf(x1 - x0, y1 - y0);
  const unsigned int num_pts = ceil(mag / density_);
  const float iterpolated_dist = mag / num_pts;  // 实际步长（均匀分布）

  // Find unit vector direction
  // 单位方向向量
  float ux = (x1 - x0) / mag;
  float uy = (y1 - y0) / mag;

  // March along it until dist
  // 从起点沿方向逐步推进采样
  float x = x0;
  float y = y0;
  poses.push_back(utils::toMsg(x, y));

  unsigned int pt_ctr = 0;
  while (pt_ctr < num_pts - 1) {
    x += ux * iterpolated_dist;
    y += uy * iterpolated_dist;
    pt_ctr++;
    poses.push_back(utils::toMsg(x, y));
  }
}

}  // namespace nav2_route
