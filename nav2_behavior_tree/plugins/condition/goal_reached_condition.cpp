// Copyright (c) 2019 Intel Corporation
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
// 【文件说明】goal_reached_condition.cpp
// 【所属模块】nav2_behavior_tree/plugins/condition —— Nav2 行为树的条件节点插件
//
// 实现 GoalReached 行为树节点：判断机器人是否已到达目标点
// （当前位置与目标的欧氏距离 <= goal_reached_tol 参数，默认 0.25 米）。
//
// 【在 Nav2 导航流程中的位置】
// 导航树的"终止判定"：通常放在导航主分支的末尾或 RecoveryNode 的
// SUCCESS 出口前——GoalReached 为真时整棵导航树返回成功。
//
// 【BehaviorTree.CPP 核心概念：条件节点（ConditionNode）】
// - 条件节点没有子节点，tick 一次立即返回：
//   条件满足 -> SUCCESS；不满足 -> FAILURE（不会返回 RUNNING）。
// - 输入端口：goal（目标位姿，来自黑板）、global_frame、
//   robot_base_frame（用于 tf 查询当前位姿）。
// - ROS2 API：用 tf2（黑板 "tf_buffer"）查询机器人当前位姿；
//   到达容差从参数 goal_reached_tol 读取。
// - XML 使用示例：
//   <GoalReached goal="{goal}" global_frame="map" robot_base_frame="base_link"/>
// =============================================================================

#include <string>
#include <memory>

#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"

#include "nav2_behavior_tree/plugins/condition/goal_reached_condition.hpp"

namespace nav2_behavior_tree
{

// 构造函数：读取坐标系端口（真正的初始化推迟到首次 tick）。
GoalReachedCondition::GoalReachedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  initialized_(false),
  global_frame_("map"),
  robot_base_frame_("base_link")
{
  // 目标坐标系（默认 map）与机器人基座坐标系（默认 base_link）。
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
}

// 析构函数：确保清理（当前 cleanup() 无需释放资源）。
GoalReachedCondition::~GoalReachedCondition()
{
  cleanup();
}

// tick 主函数：条件判定入口。
BT::NodeStatus GoalReachedCondition::tick()
{
  // 惰性初始化：首次 tick 才从黑板取节点/tf，读取参数
  // （构造阶段参数可能尚未声明完毕）。
  if (!initialized_) {
    initialize();
  }

  // 距离判定：到达 -> SUCCESS；未到 -> FAILURE。
  if (isGoalReached()) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

// 惰性初始化：获取 ROS2 节点、tf 缓冲区并读取参数。
void GoalReachedCondition::initialize()
{
  // 从黑板取共享 ROS2 节点。
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // 声明并读取到达容差参数（默认 0.25 米）。
  nav2_util::declare_parameter_if_not_declared(
    node_, "goal_reached_tol",
    rclcpp::ParameterValue(0.25));
  node_->get_parameter_or<double>("goal_reached_tol", goal_reached_tol_, 0.25);
  // 从黑板取共享的 tf2 变换缓冲区。
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  // tf 查询时间容差。
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  initialized_ = true;
}

// 距离判定：机器人当前位置到目标的距离是否在容差内。
bool GoalReachedCondition::isGoalReached()
{
  geometry_msgs::msg::PoseStamped current_pose;

  // 用 tf 查询机器人当前位姿（robot_base_frame -> global_frame）；
  // 查询失败视为未到达。
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }

  // 读取目标位姿并计算 xy 平面距离平方
  // （比较平方避免开方，微小性能优化）。
  geometry_msgs::msg::PoseStamped goal;
  getInput("goal", goal);
  double dx = goal.pose.position.x - current_pose.pose.position.x;
  double dy = goal.pose.position.y - current_pose.pose.position.y;

  return (dx * dx + dy * dy) <= (goal_reached_tol_ * goal_reached_tol_);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "GoalReached" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalReachedCondition>("GoalReached");
}
