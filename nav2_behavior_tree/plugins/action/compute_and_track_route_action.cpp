// Copyright (c) 2025 Open Navigation LLC
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
// 【文件说明】compute_and_track_route_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 ComputeAndTrackRoute 行为树节点：调用 nav2_route 的
// "compute_and_track_route" ROS2 Action——路网导航的"一站式"节点：
// 服务器端先规划路由（Route），然后持续跟踪路由执行（含动态改道），
// 周期性推送反馈（当前/下一个图节点、所在边、当前路由与路径、
// 是否发生了改道）。
//
// 【在 Nav2 导航流程中的位置】
// 路网导航模式下的主执行节点（规划+跟踪一体化）。
//
// 【行为树核心概念】
// - goal 两种模式（同 ComputeRoute）：位姿（use_poses=true）或图节点 ID。
// - on_wait_for_result 钩子：跟踪期间持续做两件事：
//   a) 检测输入变化 -> 置 goal_updated_ 触发基类自动重发新 goal
//      （运行中更新目标/改道）；
//   b) 把服务器反馈转发到输出端口（黑板），供其他节点使用。
// - 输出端口：last_node_id / next_node_id / current_edge_id / route /
//   path / rerouted / execution_duration。
// - XML 使用示例：
//   <ComputeAndTrackRoute goal="{goal}" path="{path}" route="{route}"/>
// =============================================================================

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/compute_and_track_route_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：委托基类创建 "compute_and_track_route" action 客户端，
// 并把反馈缓存对象初始化为空值。
ComputeAndTrackRouteAction::ComputeAndTrackRouteAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
  // 初始化 feedback_ 为干净的空状态（各 ID 为 0，路径/路由为空）。
  nav_msgs::msg::Path empty_path;
  nav2_msgs::msg::Route empty_route;
  feedback_.last_node_id = 0;
  feedback_.next_node_id = 0;
  feedback_.current_edge_id = 0;
  feedback_.route = empty_route;
  feedback_.path = empty_path;
  feedback_.rerouted = false;
}

// on_tick 钩子：动作开始前按两种模式读取输入填充 goal_。
void ComputeAndTrackRouteAction::on_tick()
{
  bool use_poses = false, use_start = false;
  // 读取模式选择。
  getInput("use_poses", use_poses);
  if (use_poses) {
    // 位姿模式：终点位姿 + 可选起点位姿。
    goal_.use_poses = true;
    getInput("goal", goal_.goal);

    goal_.use_start = false;
    getInput("use_start", use_start);
    if (use_start) {
      getInput("start", goal_.start);
      goal_.use_start = true;
    }
  } else {
    // 图节点 ID 模式：起终点用图中节点编号。
    getInput("start_id", goal_.start_id);
    getInput("goal_id", goal_.goal_id);
    goal_.use_start = false;
    goal_.use_poses = false;
  }
}

// on_success 钩子：路由执行成功——复位反馈输出，写出总耗时。
BT::NodeStatus ComputeAndTrackRouteAction::on_success()
{
  resetFeedbackAndOutputPorts();
  setOutput("execution_duration", result_.result->execution_duration);
  return BT::NodeStatus::SUCCESS;
}

// on_aborted 钩子：服务器中止——清空输出，耗时置空，返回 FAILURE。
BT::NodeStatus ComputeAndTrackRouteAction::on_aborted()
{
  resetFeedbackAndOutputPorts();
  setOutput("execution_duration", builtin_interfaces::msg::Duration());
  return BT::NodeStatus::FAILURE;
}

// on_cancelled 钩子：被取消——清空输出；主动取消返回 SUCCESS。
BT::NodeStatus ComputeAndTrackRouteAction::on_cancelled()
{
  resetFeedbackAndOutputPorts();
  setOutput("execution_duration", builtin_interfaces::msg::Duration());
  return BT::NodeStatus::SUCCESS;
}

// on_wait_for_result 钩子：跟踪期间每个等待周期调用——
// 检测目标更新 + 转发服务器反馈。
void ComputeAndTrackRouteAction::on_wait_for_result(
  std::shared_ptr<const Action::Feedback> feedback)
{
  // Check for request updates to the goal
  // ---- 检测输入是否变化（需要重发 goal）----
  bool use_poses = false, use_start = false;
  getInput("use_start", use_start);
  getInput("use_poses", use_poses);

  // 模式切换本身就是目标更新。
  if (goal_.use_poses != use_poses) {
    goal_updated_ = true;
  }

  if (use_poses) {
    // 位姿模式：比较终点（和可选起点）位姿是否变化。
    geometry_msgs::msg::PoseStamped goal;
    getInput("goal", goal);
    if (goal_.goal != goal) {
      goal_updated_ = true;
    }

    if (goal_.use_start != use_start) {
      goal_updated_ = true;
    }
    if (use_start) {
      geometry_msgs::msg::PoseStamped start;
      getInput("start", start);
      if (goal_.start != start) {
        goal_updated_ = true;
      }
    }
  } else {
    // Check if the start and goal IDs have changed
    // ID 模式：比较起终点节点 ID 是否变化。
    unsigned int start_id = 0;
    unsigned int goal_id = 0;
    getInput("start_id", start_id);
    getInput("goal_id", goal_id);
    if (goal_.start_id != start_id) {
      goal_updated_ = true;
    }
    if (goal_.goal_id != goal_id) {
      goal_updated_ = true;
    }
  }

  // If we're updating the request, we need to fully update the goal
  // Easier to call on_tick() again than to duplicate the code
  // （若检测到目标更新：直接复用 on_tick() 重新读取全部端口填充 goal_，
  //   避免重复代码。基类 tick() 检测到 goal_updated_ 后自动重发。）
  if (goal_updated_) {
    on_tick();
  }

  // ---- 转发服务器反馈到输出端口（黑板）----
  if (feedback) {
    feedback_ = *feedback;
    // 最近经过的图节点 / 下一个图节点 / 当前所在的边。
    setOutput("last_node_id", feedback_.last_node_id);
    setOutput("next_node_id", feedback_.next_node_id);
    setOutput("current_edge_id", feedback_.current_edge_id);
    // 当前的路由对象与展开的路径。
    setOutput("route", feedback_.route);
    setOutput("path", feedback_.path);
    // 是否发生了动态改道（如边被阻塞后重新规划）。
    setOutput("rerouted", feedback_.rerouted);
  }
}

// 复位反馈缓存并把所有反馈输出端口置为空值。
void ComputeAndTrackRouteAction::resetFeedbackAndOutputPorts()
{
  nav_msgs::msg::Path empty_path;
  nav2_msgs::msg::Route empty_route;
  feedback_.last_node_id = 0;
  feedback_.next_node_id = 0;
  feedback_.current_edge_id = 0;
  feedback_.route = empty_route;
  feedback_.path = empty_path;
  feedback_.rerouted = false;
  setOutput("last_node_id", feedback_.last_node_id);
  setOutput("next_node_id", feedback_.next_node_id);
  setOutput("current_edge_id", feedback_.current_edge_id);
  setOutput("route", feedback_.route);
  setOutput("path", feedback_.path);
  setOutput("rerouted", feedback_.rerouted);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "ComputeAndTrackRoute" 标签绑定到本类构建器。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputeAndTrackRouteAction>(
        name, "compute_and_track_route", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputeAndTrackRouteAction>(
    "ComputeAndTrackRoute", builder);
}
