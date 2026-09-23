// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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
// 【文件说明】distance_traveled_condition.cpp
// 【所属模块】nav2_behavior_tree/plugins/condition —— Nav2 行为树的条件节点插件
//
// 实现 DistanceTraveled 行为树节点：判断机器人自上次判定以来是否已
// 行驶了指定距离（distance 米）。是"按里程触发"的条件版
// （对应装饰器 DistanceController 的条件形式）。
//
// 【在 Nav2 导航流程中的位置】
// 用于里程驱动的逻辑：如每前进 X 米做一次校验/重规划触发。
//
// 【BehaviorTree.CPP 与 ROS2 API 要点】
// - 条件节点：行驶距离达到阈值 -> SUCCESS（并重置起点）；
//   未达到 -> FAILURE。
// - 通过 tf2（黑板 "tf_buffer"）查询当前位姿，与记录的起点位姿做
//   欧氏距离比较。
// - 输入端口：distance（阈值，米）、global_frame、robot_base_frame。
// - XML 使用示例：
//   <DistanceTraveled distance="1.0"/>
// =============================================================================

#include <string>
#include <memory>

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/condition/distance_traveled_condition.hpp"

namespace nav2_behavior_tree
{

// 构造函数：读取端口，获取节点与 tf 缓冲区。
DistanceTraveledCondition::DistanceTraveledCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  distance_(1.0),
  transform_tolerance_(0.1),
  global_frame_("map"),
  robot_base_frame_("base_link")
{
  // 行驶距离阈值（米）、目标坐标系、机器人基座坐标系。
  getInput("distance", distance_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  // 从黑板取共享 ROS2 节点与 tf2 缓冲区，读取 tf 时间容差。
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

// tick 主函数：行驶距离判定。
BT::NodeStatus DistanceTraveledCondition::tick()
{
  // 节点从 IDLE 启动（新一轮执行）：记录起始位姿，
  // 本次返回 FAILURE（先开始累计距离）。
  if (status() == BT::NodeStatus::IDLE) {
    if (!nav2_util::getCurrentPose(
        start_pose_, *tf_, global_frame_, robot_base_frame_,
        transform_tolerance_))
    {
      RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    }
    return BT::NodeStatus::FAILURE;
  }

  // 查询机器人当前位姿；失败则 FAILURE。
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  // Get euclidean distance
  // 计算自起点以来的行驶距离（欧氏距离）。
  auto travelled = nav2_util::geometry_utils::euclidean_distance(
    start_pose_.pose, current_pose.pose);

  // 距离未达到阈值：FAILURE。
  if (travelled < distance_) {
    return BT::NodeStatus::FAILURE;
  }

  // Update start pose
  // 达到阈值：把起点重置为当前位姿（重新累计下一段距离），
  // 返回 SUCCESS。
  start_pose_ = current_pose;

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "DistanceTraveled" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::DistanceTraveledCondition>("DistanceTraveled");
}
