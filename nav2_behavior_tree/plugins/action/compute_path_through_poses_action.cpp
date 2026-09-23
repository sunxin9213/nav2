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
// 【文件说明】compute_path_through_poses_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 ComputePathThroughPoses 行为树节点：调用 nav2_planner 的
// "compute_path_through_poses" ROS2 Action
// （nav2_msgs::action::ComputePathThroughPoses），
// 从起点（默认为机器人当前位置）依次经过一组途经点规划一条全局路径。
//
// 【在 Nav2 导航流程中的位置】
// 与 ComputePathToPose 类似，区别在于支持多个途经点；用于多点导航
// （NavigateThroughPoses）任务的规划阶段。
//
// 【行为树核心概念】
// - 输入端口：goals（途经点数组）、planner_id（规划器选择）、
//   start（可选显式起点）。
// - 输出端口：path（规划结果写入黑板，供 FollowPath 跟踪）。
// - on_success/on_aborted/on_cancelled 钩子保证各种结束路径下
//   path 端口的值确定（成功=路径，失败/取消=空路径）。
// - XML 使用示例：
//   <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="GridBased"/>
// =============================================================================

#include <memory>
#include <string>
#include <vector>

#include "nav2_behavior_tree/plugins/action/compute_path_through_poses_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：委托基类创建 "compute_path_through_poses" action 客户端。
ComputePathThroughPosesAction::ComputePathThroughPosesAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::ComputePathThroughPoses>(xml_tag_name, action_name, conf)
{
}

// on_tick 钩子：动作开始前读取输入端口填充 goal_。
void ComputePathThroughPosesAction::on_tick()
{
  // 途经点数组：来自黑板 "goals" 键（多点导航的目标集）。
  getInput("goals", goal_.goals);
  // 规划器 ID：允许动态选择规划器插件。
  getInput("planner_id", goal_.planner_id);
  // 可选起点：提供则置 use_start=true，服务器用它代替当前位姿。
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }
}

// on_success 钩子：规划成功时把结果路径写入黑板 path 键。
BT::NodeStatus ComputePathThroughPosesAction::on_success()
{
  setOutput("path", result_.result->path);
  return BT::NodeStatus::SUCCESS;
}

// on_aborted 钩子：服务器中止（无可行路径等）时输出空路径并返回 FAILURE。
BT::NodeStatus ComputePathThroughPosesAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::FAILURE;
}

// on_cancelled 钩子：被取消时输出空路径；主动取消返回 SUCCESS。
BT::NodeStatus ComputePathThroughPosesAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "ComputePathThroughPoses" 标签绑定到本类构建器。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputePathThroughPosesAction>(
        name, "compute_path_through_poses", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputePathThroughPosesAction>(
    "ComputePathThroughPoses", builder);
}
