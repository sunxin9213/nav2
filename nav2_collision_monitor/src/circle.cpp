// Copyright (c) 2022 Samsung R&D Institute Russia
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
// Circle（圆形安全区）类的实现。
// 模块定位：nav2_collision_monitor 中 Polygon（多边形安全区）的特化子类，
// 用"圆形"描述机器人周围的减速/停障区域。与一般多边形相比：
//   - 参数只需一个 radius（半径），配置更简单；
//   - getPolygon 生成 16 边正多边形近似圆（用于可视化发布）；
//   - getPointsInside 用 x^2+y^2 < r^2 快速判定点是否在圆内
//     （比多边形逐边相交检测更快、更稳定）。
// 适用于全向轮等无方向性的机器人。
// ============================================================================

#include "nav2_collision_monitor/circle.hpp"

#include <math.h>
#include <cmath>
#include <exception>

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

// 构造：委托给基类 Polygon 完成公共初始化
Circle::Circle(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & polygon_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
: Polygon::Polygon(node, polygon_name, tf_buffer, base_frame_id, transform_tolerance)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Circle", polygon_name_.c_str());
}

// 析构：打印日志
Circle::~Circle()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Circle", polygon_name_.c_str());
}

// ---------------------------------------------------------------------------
// 生成圆形安全区对应的（近似）多边形顶点：
// 以机器人为圆心，用 16 个等角度间隔的顶点构成正十六边形逼近圆。
// 该结果用于可视化发布（polygon 话题）。
// ---------------------------------------------------------------------------
void Circle::getPolygon(std::vector<Point> & poly) const
{
  // Number of polygon points. More edges means better approximation.
  // 多边形边数：边越多对圆的近似越好
  const double polygon_edges = 16;
  // Increment of angle during points position calculation
  // 相邻顶点间的角度增量
  double angle_increment = 2 * M_PI / polygon_edges;

  // Clear polygon before filling
  poly.clear();

  // Making new polygon looks like a circle
  // 在圆周上等角度采样生成顶点（半径 radius_，圆心为机器人本体原点）
  Point p;
  for (double angle = 0.0; angle < 2 * M_PI; angle += angle_increment) {
    p.x = radius_ * std::cos(angle);
    p.y = radius_ * std::sin(angle);
    poly.push_back(p);
  }
}

// ---------------------------------------------------------------------------
// 统计落在圆形安全区内的观测点数量。
// 圆的判定：x^2 + y^2 < r^2（相对机器人本体坐标），比多边形检测高效。
// ---------------------------------------------------------------------------
int Circle::getPointsInside(const std::vector<Point> & points) const
{
  int num = 0;
  for (Point point : points) {
    if (point.x * point.x + point.y * point.y < radius_squared_) {
      num++;
    }
  }

  return num;
}

// ---------------------------------------------------------------------------
// 读取 Circle 专属参数：
//   调用基类 getCommonParameters 读取公共参数（动作类型、订阅话题等）；
//   圆形没有足迹订阅，footprint_topic 置空；
//   读取 <polygon_name>.radius 参数（无默认值：未配置时视为错误）。
// ---------------------------------------------------------------------------
bool Circle::getParameters(std::string & polygon_pub_topic, std::string & footprint_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!getCommonParameters(polygon_pub_topic)) {
    return false;
  }

  // There is no footprint subscription for the Circle. Thus, set string as empty.
  // 圆形安全区不需要订阅机器人足迹
  footprint_topic.clear();

  try {
    // Leave it not initialized: the will cause an error if it will not set
    // 声明 radius 参数但不给默认值：未配置时读取会抛异常
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".radius", rclcpp::PARAMETER_DOUBLE);
    radius_ = node->get_parameter(polygon_name_ + ".radius").as_double();
    radius_squared_ = radius_ * radius_;  // 缓存半径平方，加速内点判定
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Error while getting circle parameters: %s",
      polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

}  // namespace nav2_collision_monitor
