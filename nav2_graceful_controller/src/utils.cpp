// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#include "nav2_graceful_controller/utils.hpp"

namespace nav2_graceful_controller
{

// 中文说明（文件级）：
// 工具函数：构造 RViz 调试消息（运动目标点 PointStamped 与减速半径球体 Marker）。

// 将运动目标位姿转为 PointStamped 点消息（略抬高 z 便于观察）
geometry_msgs::msg::PointStamped createMotionTargetMsg(
  const geometry_msgs::msg::PoseStamped & motion_target)
{
  geometry_msgs::msg::PointStamped motion_target_point;
  motion_target_point.header = motion_target.header;
  motion_target_point.point = motion_target.pose.position;
  motion_target_point.point.z = 0.01;
  return motion_target_point;
}

// 在运动目标处生成半透明绿色球体标记，可视化减速半径范围
visualization_msgs::msg::Marker createSlowdownMarker(
  const geometry_msgs::msg::PoseStamped & motion_target, const double & slowdown_radius)
{
  visualization_msgs::msg::Marker slowdown_marker;
  slowdown_marker.header = motion_target.header;
  slowdown_marker.ns = "slowdown";  // 命名空间：同 ns 的旧标记会被覆盖
  slowdown_marker.id = 0;
  slowdown_marker.type = visualization_msgs::msg::Marker::SPHERE;  // 球体
  slowdown_marker.action = visualization_msgs::msg::Marker::ADD;
  slowdown_marker.pose = motion_target.pose;
  slowdown_marker.pose.position.z = 0.01;
  // 球体直径 = 2 * 减速半径，z 压扁成薄盘便于在平面地图上观察
  slowdown_marker.scale.x = slowdown_radius * 2.0;
  slowdown_marker.scale.y = slowdown_radius * 2.0;
  slowdown_marker.scale.z = 0.02;
  // 半透明绿色
  slowdown_marker.color.a = 0.2;
  slowdown_marker.color.r = 0.0;
  slowdown_marker.color.g = 1.0;
  slowdown_marker.color.b = 0.0;
  return slowdown_marker;
}

}  // namespace nav2_graceful_controller
