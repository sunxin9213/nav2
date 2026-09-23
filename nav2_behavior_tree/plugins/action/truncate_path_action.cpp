// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

// =============================================================================
// 【文件说明】truncate_path_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 TruncatePath 行为树节点：把输入路径从末端截短——保留路径终点，
// 删除终点前 distance 米以内的中间点。用于配合 RPP（纯追踪）等控制器：
// 纯追踪沿切线跟踪，若终点前有密集点会导致机器人冲过目标，
// 截短后末端朝向被修正为指向真正的终点。
//
// 【在 Nav2 导航流程中的位置】
// 位于 ComputePathToPose 与 FollowPath 之间的可选数据处理节点
// （本地计算，不涉及任何 ROS2 通信）。
//
// 【行为树核心概念】
// - 输入端口：input_path（原始路径）、distance（截短距离，米）；
//   输出端口：output_path（截短后的路径）。
// - 虽然分类为动作节点（ActionNodeBase），但 tick 是纯本地同步计算，
//   一次完成，立刻返回 SUCCESS。
// - XML 使用示例：
//   <TruncatePath distance="1.0" input_path="{path}" output_path="{truncated_path}"/>
// =============================================================================

#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/action/truncate_path_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：读取截短距离端口（默认 1.0 米）。
TruncatePath::TruncatePath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  distance_(1.0)
{
  // distance：从路径末端起要截掉的大致长度（米）。
  getInput("distance", distance_);
}

// tick 主函数：截短路径并修正末端朝向（本地同步计算）。
inline BT::NodeStatus TruncatePath::tick()
{
  // 标记 RUNNING（BT 约定，虽然本节点单次完成）。
  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::msg::Path input_path;

  // 读取输入路径（通常来自黑板 "path" 键）。
  getInput("input_path", input_path);

  // 空路径：无需处理，原样输出并成功返回。
  if (input_path.poses.empty()) {
    setOutput("output_path", input_path);
    return BT::NodeStatus::SUCCESS;
  }

  // 记住原始终点（截短后末端要指向它）。
  geometry_msgs::msg::PoseStamped final_pose = input_path.poses.back();

  // 从末端逐点回退删除：只要"当前末端点到原终点"的距离还小于
  // distance_ 且路径还剩多于 2 个点，就弹出末端点。
  // （第一行计算结果恒为 0，属于原有实现风格，实际回退发生在循环内。）
  double distance_to_goal = nav2_util::geometry_utils::euclidean_distance(
    input_path.poses.back(), final_pose);

  while (distance_to_goal < distance_ && input_path.poses.size() > 2) {
    input_path.poses.pop_back();
    distance_to_goal = nav2_util::geometry_utils::euclidean_distance(
      input_path.poses.back(), final_pose);
  }

  // 计算新末端点指向原终点的方向角（atan2(dy, dx)）。
  double dx = final_pose.pose.position.x - input_path.poses.back().pose.position.x;
  double dy = final_pose.pose.position.y - input_path.poses.back().pose.position.y;

  double final_angle = atan2(dy, dx);

  // 数学异常保护：方向角 NaN/Inf（如两点重合）时置 0 并告警。
  if (std::isnan(final_angle) || std::isinf(final_angle)) {
    RCLCPP_WARN(
      config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
      "Final angle is not valid while truncating path. Setting to 0.0");
    final_angle = 0.0;
  }

  // 把新末端的朝向设置为指向原终点的方向
  // （orientationAroundZAxis 生成绕 Z 轴的四元数）。
  input_path.poses.back().pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
    final_angle);

  // 输出截短后的路径到黑板。
  setOutput("output_path", input_path);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "TruncatePath" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TruncatePath>("TruncatePath");
}
