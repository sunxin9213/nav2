/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// ============================================================================
// 中文注释：机器人足迹（footprint）工具函数实现文件
//
// 【文件功能】
// 本文件提供机器人"足迹"（机器人本体在地面上的投影多边形）相关工具：
//   - calculateMinAndMaxDistances：计算足迹的内切/外接半径；
//   - transformFootprint：把足迹按位姿 (x,y,theta) 旋转平移到世界坐标；
//   - padFootprint：把足迹向外"加衬垫"（安全冗余）；
//   - makeFootprintFromRadius：用圆半径生成 16 边形足迹；
//   - makeFootprintFromString：解析 YAML 中的足迹字符串 "[[x1,y1],[x2,y2],...]"；
//   - 若干 Point/Point32/Polygon 类型转换工具。
//
// 【在 Nav2 中的位置】
// 被 Costmap2DROS（读取 footprint 参数）、InflationLayer（内切半径）、
// 各层足迹清除、碰撞检测器等广泛使用。
//
// 【核心概念】
// - 足迹有两种配置方式：footprint 多边形字符串 或 robot_radius 圆形；
// - 内切半径 inscribed radius：中心到边的最短距离——中心距障碍小于
//   该值必然碰撞（对应 INSCRIBED_INFLATED_OBSTACLE）；
// - 外接半径 circumscribed radius：中心到顶点的最大距离。
// ============================================================================

#include "nav2_costmap_2d/footprint.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "nav2_costmap_2d/array_parser.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

namespace nav2_costmap_2d
{

// ============================================================================
// calculateMinAndMaxDistances()：计算机器人中心到足迹的最小/最大距离
// min_dist = 内切半径（中心到边的最短距离）
// max_dist = 外接半径（中心到顶点的最远距离）
// 两者都是膨胀层与碰撞检测的关键参数
// ============================================================================

void calculateMinAndMaxDistances(
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double & min_dist, double & max_dist)
{
  min_dist = std::numeric_limits<double>::max();
  max_dist = 0.0;

  // 少于 3 个点构不成多边形
  if (footprint.size() <= 2) {
    return;
  }

  // 遍历每条边 (i → i+1)
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    // check the distance from the robot center point to the first vertex
    // 中心到顶点的距离（候选外接半径）
    double vertex_dist = distance(0.0, 0.0, footprint[i].x, footprint[i].y);
    // 中心到边（线段）的距离（候选内切半径）
    double edge_dist = distanceToLine(
      0.0, 0.0, footprint[i].x, footprint[i].y,
      footprint[i + 1].x, footprint[i + 1].y);
    min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
  }

  // we also need to do the last vertex and the first vertex
  // 补上闭合边（最后一个顶点 → 第一个顶点）
  double vertex_dist = distance(0.0, 0.0, footprint.back().x, footprint.back().y);
  double edge_dist = distanceToLine(
    0.0, 0.0, footprint.back().x, footprint.back().y,
    footprint.front().x, footprint.front().y);
  min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
  max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
}

// ---- 类型转换工具：Point(64位) ↔ Point32(32位) ----
geometry_msgs::msg::Point32 toPoint32(geometry_msgs::msg::Point pt)
{
  geometry_msgs::msg::Point32 point32;
  point32.x = pt.x;
  point32.y = pt.y;
  point32.z = pt.z;
  return point32;
}

geometry_msgs::msg::Point toPoint(geometry_msgs::msg::Point32 pt)
{
  geometry_msgs::msg::Point point;
  point.x = pt.x;
  point.y = pt.y;
  point.z = pt.z;
  return point;
}

// 点向量 → Polygon 消息（Point32 数组）
geometry_msgs::msg::Polygon toPolygon(std::vector<geometry_msgs::msg::Point> pts)
{
  geometry_msgs::msg::Polygon polygon;
  for (unsigned int i = 0; i < pts.size(); i++) {
    polygon.points.push_back(toPoint32(pts[i]));
  }
  return polygon;
}

// Polygon 消息 → 点向量
std::vector<geometry_msgs::msg::Point> toPointVector(geometry_msgs::msg::Polygon::SharedPtr polygon)
{
  std::vector<geometry_msgs::msg::Point> pts;
  for (unsigned int i = 0; i < polygon->points.size(); i++) {
    pts.push_back(toPoint(polygon->points[i]));
  }
  return pts;
}

// ============================================================================
// transformFootprint()（vector 版）：把车体系足迹按位姿变换到世界系
// 二维旋转矩阵：new_x = x + (px*cos - py*sin)，new_y = y + (px*sin + py*cos)
// 用于各层的"足迹清除"（先变换再栅格化）
// ============================================================================

void transformFootprint(
  double x, double y, double theta,
  const std::vector<geometry_msgs::msg::Point> & footprint_spec,
  std::vector<geometry_msgs::msg::Point> & oriented_footprint)
{
  // build the oriented footprint at a given location
  oriented_footprint.resize(footprint_spec.size());
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
    // 平移 (x,y) + 旋转 theta 的复合变换
    double new_x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    double new_y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    geometry_msgs::msg::Point & new_pt = oriented_footprint[i];
    new_pt.x = new_x;
    new_pt.y = new_y;
  }
}

// ============================================================================
// transformFootprint()（PolygonStamped 版）：同上，但输出 ROS 消息类型
// （常用于发布/可视化）
// ============================================================================

void transformFootprint(
  double x, double y, double theta,
  const std::vector<geometry_msgs::msg::Point> & footprint_spec,
  geometry_msgs::msg::PolygonStamped & oriented_footprint)
{
  // build the oriented footprint at a given location
  oriented_footprint.polygon.points.clear();
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
    geometry_msgs::msg::Point32 new_pt;
    new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    oriented_footprint.polygon.points.push_back(new_pt);
  }
}

// ============================================================================
// padFootprint()：给足迹加安全衬垫（原地修改）
// 每个顶点沿其符号方向（sign0：x/y 各自的符号）外移 padding
// ============================================================================

void padFootprint(std::vector<geometry_msgs::msg::Point> & footprint, double padding)
{
  // pad footprint in place
  for (unsigned int i = 0; i < footprint.size(); i++) {
    geometry_msgs::msg::Point & pt = footprint[i];
    pt.x += sign0(pt.x) * padding;   // x 按符号外扩
    pt.y += sign0(pt.y) * padding;   // y 按符号外扩
  }
}


// ============================================================================
// makeFootprintFromRadius()：用圆半径生成正 16 边形足迹
// （robot_radius 圆形配置的实现：以多边形近似圆）
// ============================================================================

std::vector<geometry_msgs::msg::Point> makeFootprintFromRadius(double radius)
{
  std::vector<geometry_msgs::msg::Point> points;

  // Loop over 16 angles around a circle making a point each time
  // 均匀取 16 个角度，在圆周上生成顶点
  int N = 16;
  geometry_msgs::msg::Point pt;
  for (int i = 0; i < N; ++i) {
    double angle = i * 2 * M_PI / N;
    pt.x = cos(angle) * radius;
    pt.y = sin(angle) * radius;

    points.push_back(pt);
  }

  return points;
}


// ============================================================================
// makeFootprintFromString()：解析 YAML 中的足迹字符串
// 格式："[[x1,y1],[x2,y2],[x3,y3],...]"（parseVVF 做实际的字符串解析）
// 至少需要 3 个点，每个点必须是 (x,y) 数值对
// ============================================================================

bool makeFootprintFromString(
  const std::string & footprint_string,
  std::vector<geometry_msgs::msg::Point> & footprint)
{
  std::string error;
  // 解析为 vector<vector<float>>
  std::vector<std::vector<float>> vvf = parseVVF(footprint_string, error);

  if (error != "") {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "nav2_costmap_2d"), "Error parsing footprint parameter: '%s'", error.c_str());
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "nav2_costmap_2d"), "  Footprint string was '%s'.", footprint_string.c_str());
    return false;
  }

  // convert vvf into points.
  // 多边形至少 3 个顶点
  if (vvf.size() < 3) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "nav2_costmap_2d"),
      "You must specify at least three points for the robot footprint, reverting to previous footprint."); //NOLINT
    return false;
  }
  footprint.reserve(vvf.size());
  // 每个点必须是恰好 2 个数字 (x,y)
  for (unsigned int i = 0; i < vvf.size(); i++) {
    if (vvf[i].size() == 2) {
      geometry_msgs::msg::Point point;
      point.x = vvf[i][0];
      point.y = vvf[i][1];
      point.z = 0;
      footprint.push_back(point);
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger(
            "nav2_costmap_2d"),
        "Points in the footprint specification must be pairs of numbers. Found a point with %d numbers.", //NOLINT
        static_cast<int>(vvf[i].size()));
      return false;
    }
  }

  return true;
}

}  // end namespace nav2_costmap_2d
