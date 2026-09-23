// Copyright (c) 2021 RoboTech Vision
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
// 【文件说明】truncate_path_local_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 TruncatePathLocal 行为树节点：以"机器人当前位姿在路径上的投影点"
// 为中心，截取路径的一个局部片段——向前 distance_forward 米、向后
// distance_backward 米。用于把全局路径裁剪成局部路径交给控制器，
// 或配合局部规划器限定搜索范围。
//
// 【在 Nav2 导航流程中的位置】
// 可选数据处理节点，位于规划与跟踪之间；与 TruncatePath（从末端截短）
// 不同，本节点按"机器人当前位置"截取。
//
// 【行为树核心概念与 ROS2 API】
// - 输入端口：input_path、distance_forward / distance_backward（米）、
//   angular_distance_weight（最近点匹配时的角度距离权重）、
//   max_robot_pose_search_dist（机器人位姿搜索上限，inf = 不裁剪）、
//   pose（可选，直接给定位姿）或 robot_frame + transform_tolerance
//   （用 tf 查询当前位姿）。
// - 输出端口：output_path。
// - 使用 tf2 缓冲区（黑板 "tf_buffer"）+ nav2_util::getCurrentPose 查询
//   机器人位姿；使用 nav2_util::geometry_utils 的迭代器工具
//   （first_after_integrated_distance / min_by）做几何计算。
// - XML 使用示例：
//   <TruncatePathLocal input_path="{path}" output_path="{local_path}"
//                      distance_forward="5.0" distance_backward="2.0"
//                      robot_frame="base_link"/>
// =============================================================================

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/create_timer_ros.h"

#include "nav2_behavior_tree/plugins/action/truncate_path_local_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：从黑板获取共享的 tf2 变换缓冲区。
TruncatePathLocal::TruncatePathLocal(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  tf_buffer_ =
    config().blackboard->template get<std::shared_ptr<tf2_ros::Buffer>>(
    "tf_buffer");
}

// tick 主函数：找到机器人投影点，截取局部路径并输出。
inline BT::NodeStatus TruncatePathLocal::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  // 读取各输入端口：前向/后向截取长度、角度权重、搜索上限。
  double distance_forward, distance_backward;
  geometry_msgs::msg::PoseStamped pose;
  double angular_distance_weight;
  double max_robot_pose_search_dist;

  getInput("distance_forward", distance_forward);
  getInput("distance_backward", distance_backward);
  getInput("angular_distance_weight", angular_distance_weight);
  getInput("max_robot_pose_search_dist", max_robot_pose_search_dist);

  // 是否启用"最近点搜索窗口裁剪"：上限为有限值时启用——
  // 只在机器人附近一段窗口内找投影点，效率更高。
  bool path_pruning = std::isfinite(max_robot_pose_search_dist);
  // 读取输入路径。
  nav_msgs::msg::Path new_path;
  getInput("input_path", new_path);
  // 若不裁剪，或路径发生了变化（重新规划），重置搜索起点到路径开头。
  if (!path_pruning || new_path != path_) {
    path_ = new_path;
    closest_pose_detection_begin_ = path_.poses.begin();
  }

  // 获取机器人当前位姿（tf 查询或端口直供）；失败则节点失败。
  if (!getRobotPose(path_.header.frame_id, pose)) {
    return BT::NodeStatus::FAILURE;
  }

  // 空路径：原样输出并成功。
  if (path_.poses.empty()) {
    setOutput("output_path", path_);
    return BT::NodeStatus::SUCCESS;
  }

  // 计算最近点搜索窗口的结束迭代器：
  // 从上次搜索起点沿路径累计 max_robot_pose_search_dist 米处。
  auto closest_pose_detection_end = path_.poses.end();
  if (path_pruning) {
    closest_pose_detection_end = nav2_util::geometry_utils::first_after_integrated_distance(
      closest_pose_detection_begin_, path_.poses.end(), max_robot_pose_search_dist);
  }

  // find the closest pose on the path
  // 在窗口内找与机器人位姿"最接近"的路径点（距离度量含角度项）。
  auto current_pose = nav2_util::geometry_utils::min_by(
    closest_pose_detection_begin_, closest_pose_detection_end,
    [&pose, angular_distance_weight](const geometry_msgs::msg::PoseStamped & ps) {
      return poseDistance(pose, ps, angular_distance_weight);
    });

  // 裁剪模式：把下次搜索的起点推进到本次找到的最近点
  // （机器人沿路径前进，窗口也随之滑动）。
  if (path_pruning) {
    closest_pose_detection_begin_ = current_pose;
  }

  // expand forwards to extract desired length
  // 从最近点向前累计 distance_forward 米，得到片段的前边界。
  auto forward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
    current_pose, path_.poses.end(), distance_forward);

  // expand backwards to extract desired length
  // Note: current_pose + 1 is used because reverse iterator points to a cell before it
  // 从最近点向后累计 distance_backward 米，得到片段的后边界
  // （反向迭代器语义：指向元素的前一格，故 +1 修正）。
  auto backward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
    std::reverse_iterator(current_pose + 1), path_.poses.rend(), distance_backward);

  // 组装输出路径：[后边界, 前边界) 区间的位姿，沿用原路径头部信息。
  nav_msgs::msg::Path output_path;
  output_path.header = path_.header;
  output_path.poses = std::vector<geometry_msgs::msg::PoseStamped>(
    backward_pose_it.base(), forward_pose_it);
  setOutput("output_path", output_path);

  return BT::NodeStatus::SUCCESS;
}

// 获取机器人位姿：优先用端口直供的 pose；否则用 tf 查询。
inline bool TruncatePathLocal::getRobotPose(
  std::string path_frame_id, geometry_msgs::msg::PoseStamped & pose)
{
  if (!getInput("pose", pose)) {
    // 端口未提供 pose：需要 robot_frame 端口来查 tf。
    std::string robot_frame;
    if (!getInput("robot_frame", robot_frame)) {
      // 两者都没有：配置错误，记日志并失败。
      RCLCPP_ERROR(
        config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
        "Neither pose nor robot_frame specified for %s", name().c_str());
      return false;
    }
    // 读取 tf 查询时间容差。
    double transform_tolerance;
    getInput("transform_tolerance", transform_tolerance);
    // 用 tf2 缓冲区查询：robot_frame -> 路径坐标系 的当前位姿。
    if (!nav2_util::getCurrentPose(
        pose, *tf_buffer_, path_frame_id, robot_frame, transform_tolerance))
    {
      RCLCPP_WARN(
        config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
        "Failed to lookup current robot pose for %s", name().c_str());
      return false;
    }
  }
  return true;
}

// 带角度项的距离度量：sqrt(dx^2 + dy^2 + (w*|角度差|)^2)。
// 权重越大，最近点匹配越看重朝向一致性（机器人在路径上的投影
// 不仅要位置近，还要方向接近）。
double
TruncatePathLocal::poseDistance(
  const geometry_msgs::msg::PoseStamped & pose1,
  const geometry_msgs::msg::PoseStamped & pose2,
  const double angular_distance_weight)
{
  double dx = pose1.pose.position.x - pose2.pose.position.x;
  double dy = pose1.pose.position.y - pose2.pose.position.y;
  // taking angular distance into account in addition to spatial distance
  // 把四元数转成 tf2::Quaternion 再算最短夹角。
  tf2::Quaternion q1;
  tf2::convert(pose1.pose.orientation, q1);
  tf2::Quaternion q2;
  tf2::convert(pose2.pose.orientation, q2);
  double da = angular_distance_weight * std::abs(q1.angleShortestPath(q2));
  return std::sqrt(dx * dx + dy * dy + da * da);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "TruncatePathLocal" 标签注册进 factory。
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::TruncatePathLocal>(
    "TruncatePathLocal");
}
