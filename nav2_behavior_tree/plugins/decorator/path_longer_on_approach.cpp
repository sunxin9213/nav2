// Copyright (c) 2022 Neobotix GmbH
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
// 【文件说明】path_longer_on_approach.cpp
// 【所属模块】nav2_behavior_tree/plugins/decorator —— Nav2 行为树的装饰节点插件
//
// 实现 PathLongerOnApproach（接近目标时路径变长检测）装饰节点：
// 机器人已接近目标（剩余路径短于 prox_len）时，若重规划出的新路径
// 反而比旧路径长 length_factor 倍以上，说明规划异常（如绕路/抖动），
// 此时才 tick 子节点（通常是恢复/停止分支）；正常情况直接返回 SUCCESS。
//
// 【在 Nav2 导航流程中的位置】
// 默认导航树中包裹"跟随路径"相关分支：防止接近目标时反复重规划
// 导致机器人来回摆动（path oscillation）。
//
// 【BehaviorTree.CPP 核心概念】
// - 装饰节点：条件满足才透传子节点，否则本节点直接 SUCCESS。
// - 输入端口：path（当前路径，黑板）、prox_len（接近判定阈值，米）、
//   length_factor（新路径变长倍数阈值）。
// - 三个判定函数：isPathUpdated / isRobotInGoalProximity / isNewPathLonger。
// - XML 使用示例：
//   <PathLongerOnApproach path="{path}" prox_len="3.0" length_factor="2.0">
//     <FollowPath .../>
//   </PathLongerOnApproach>
// =============================================================================

#include <string>
#include <memory>
#include <vector>
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/decorator/path_longer_on_approach.hpp"

namespace nav2_behavior_tree
{

// 构造函数：从黑板取共享 ROS2 节点（仅用于日志）。
PathLongerOnApproach::PathLongerOnApproach(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

// 判定一：路径是否"更新"——新旧路径都非空、点数不同
// 且终点相同（同一目标下重新规划出了不同的路径）。
bool PathLongerOnApproach::isPathUpdated(
  nav_msgs::msg::Path & new_path,
  nav_msgs::msg::Path & old_path)
{
  return old_path.poses.size() != 0 &&
         new_path.poses.size() != 0 &&
         new_path.poses.size() != old_path.poses.size() &&
         old_path.poses.back().pose.position == new_path.poses.back().pose.position;
}

// 判定二：机器人是否接近目标——旧路径（机器人当前要走的路径）
// 总长度小于 prox_len（米）。
bool PathLongerOnApproach::isRobotInGoalProximity(
  nav_msgs::msg::Path & old_path,
  double & prox_leng)
{
  return nav2_util::geometry_utils::calculate_path_length(old_path, 0) < prox_leng;
}

// 判定三：新路径是否显著更长——新路径长度 > length_factor * 旧路径长度。
bool PathLongerOnApproach::isNewPathLonger(
  nav_msgs::msg::Path & new_path,
  nav_msgs::msg::Path & old_path,
  double & length_factor)
{
  return nav2_util::geometry_utils::calculate_path_length(new_path, 0) >
         length_factor * nav2_util::geometry_utils::calculate_path_length(
    old_path, 0);
}

// tick 主函数：接近目标且新路径异常变长时才执行子节点。
inline BT::NodeStatus PathLongerOnApproach::tick()
{
  // 读取当前路径与两个阈值参数。
  getInput("path", new_path_);
  getInput("prox_len", prox_len_);
  getInput("length_factor", length_factor_);

  // 状态复位检测：若旧路径为空、新路径为空或新旧终点不同
  // （目标变了），标记 first_time_ 重新开始监测。
  if (first_time_ == false) {
    if (old_path_.poses.empty() || new_path_.poses.empty() ||
      old_path_.poses.back().pose != new_path_.poses.back().pose)
    {
      first_time_ = true;
    }
  }

  setStatus(BT::NodeStatus::RUNNING);

  // Check if the path is updated and valid, compare the old and the new path length,
  // given the goal proximity and check if the new path is longer
  // （三个条件全满足：路径已更新 + 机器人接近目标 + 新路径异常变长。）
  if (isPathUpdated(new_path_, old_path_) && isRobotInGoalProximity(old_path_, prox_len_) &&
    isNewPathLonger(new_path_, old_path_, length_factor_) && !first_time_)
  {
    // 条件满足：tick 子节点（通常触发停止/恢复逻辑）。
    const BT::NodeStatus child_state = child_node_->executeTick();
    switch (child_state) {
      // 子节点仍在运行：透传。
      case BT::NodeStatus::RUNNING:
        return child_state;
      // 子节点结束（成功/失败）：更新旧路径快照并复位子节点状态。
      case BT::NodeStatus::SUCCESS:
      case BT::NodeStatus::FAILURE:
        old_path_ = new_path_;
        resetChild();
        return child_state;
      default:
        old_path_ = new_path_;
        return BT::NodeStatus::FAILURE;
    }
  }
  // 正常情况（未触发异常检测）：更新旧路径快照、清除首次标记，
  // 本节点直接返回 SUCCESS（子节点不被 tick，导航正常继续）。
  old_path_ = new_path_;
  first_time_ = false;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "PathLongerOnApproach" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathLongerOnApproach>("PathLongerOnApproach");
}
