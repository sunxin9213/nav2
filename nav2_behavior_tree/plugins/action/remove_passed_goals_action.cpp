// Copyright (c) 2021 Samsung Research America
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
// 【文件说明】remove_passed_goals_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 RemovePassedGoals 行为树节点：多点导航（NavigateThroughPoses）中，
// 从目标队列头部移除机器人已经经过（到达半径内）的途经点，
// 输出剩余的目标序列。这样下游的规划/跟踪始终以"下一个未到达的途经点"
// 为目标。
//
// 【在 Nav2 导航流程中的位置】
// 位于多点导航树中 GoalUpdated/ComputePathThroughPoses 之前的数据处理节点。
//
// 【行为树核心概念与 ROS2 API】
// - 输入端口：input_goals（当前目标序列）、radius（途经点到达半径，米）、
//   global_frame（目标所在坐标系，如 "map"）、robot_base_frame
//   （机器人基座坐标系，如 "base_link"）；
//   输出端口：output_goals（剔除已过点后的序列）。
// - 使用 tf2（从黑板 "tf_buffer" 取变换缓冲区）查询机器人当前位姿：
//   nav2_util::getCurrentPose 内部通过 lookupTransform 把 base_link 位姿
//   变换到 global_frame。这演示了 BT 节点直接消费 ROS2 生态服务
//   （tf、参数）的典型方式。
// - 注意最后一个目标永远不会被移除（size > 1 条件）——
//   它是导航的最终目的地，到达与否由 GoalReached 等节点判定。
// - XML 使用示例：
//   <RemovePassedGoals input_goals="{goals}" output_goals="{goals}"
//                      radius="0.5" global_frame="map"/>
// =============================================================================

#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/action/remove_passed_goals_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：读取端口并获取 tf 与参数依赖。
RemovePassedGoals::RemovePassedGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  viapoint_achieved_radius_(0.5)
{
  // 途经点"已到达"的判定半径（米）。
  getInput("radius", viapoint_achieved_radius_);

  // 目标坐标系与机器人基座坐标系：用于 tf 查询当前位姿。
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  // 从黑板取 tf2 缓冲区（由 bt_navigator 预先放入，
  // 各 BT 节点共享同一个 tf 监听器）。
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  // 读取 tf 查询的时间容差参数（超龄的变换视为不可用）。
  node->get_parameter("transform_tolerance", transform_tolerance_);
}

// tick 主函数：剔除已路过的途经点（本地计算 + tf 查询，单次完成）。
inline BT::NodeStatus RemovePassedGoals::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  Goals goal_poses;
  // 读取当前目标序列。
  getInput("input_goals", goal_poses);

  // 空序列：原样输出并成功。
  if (goal_poses.empty()) {
    setOutput("output_goals", goal_poses);
    return BT::NodeStatus::SUCCESS;
  }

  using namespace nav2_util::geometry_utils;  // NOLINT

  // 通过 tf 查询机器人当前位姿（base_link -> global_frame 变换）；
  // 查询失败（tf 不可用）时节点失败。
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    return BT::NodeStatus::FAILURE;
  }

  // 从队头依次检查：只要机器人离队头目标在半径内且后面还有其他目标，
  // 就把该目标从队列移除；遇到第一个未到达的目标即停止。
  double dist_to_goal;
  while (goal_poses.size() > 1) {
    dist_to_goal = euclidean_distance(goal_poses[0].pose, current_pose.pose);

    if (dist_to_goal > viapoint_achieved_radius_) {
      break;
    }

    goal_poses.erase(goal_poses.begin());
  }

  // 输出剩余目标序列（最终目的地一定保留）。
  setOutput("output_goals", goal_poses);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "RemovePassedGoals" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemovePassedGoals>("RemovePassedGoals");
}
