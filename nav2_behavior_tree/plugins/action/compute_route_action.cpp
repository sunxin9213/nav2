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
// 【文件说明】compute_route_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 ComputeRoute 行为树节点：调用 nav2_route 的 "compute_route" ROS2
// Action，在路网图（roadmap/graph，节点+边）上规划一条路由。
// 这是 Nav2 新的路网导航体系：先用图搜索得到 Route（一串节点/边），
// 再把 Route 展开成 Path 交给控制器跟踪。
//
// 【在 Nav2 导航流程中的位置】
// 路网导航模式下的规划节点（对应传统模式的 ComputePathToPose）。
//
// 【行为树核心概念】
// - goal 支持两种表达：
//   a) 位姿模式（use_poses=true）：goal/start 为 PoseStamped；
//   b) 图节点 ID 模式（use_poses=false）：start_id/goal_id 为图中节点编号。
// - 输出端口：path（展开的路径）、route（路由对象）、planning_time。
// - on_success/on_aborted/on_cancelled/halt 四条结束路径都经 resetPorts()
//   统一处理输出端口，保证黑板状态确定性。
// - XML 使用示例：
//   <ComputeRoute goal_id="42" path="{path}" route="{route}"/>
// =============================================================================

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/compute_route_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：委托基类创建 "compute_route" action 客户端，无额外初始化。
ComputeRouteAction::ComputeRouteAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

// on_tick 钩子：动作开始前读取输入端口填充 goal_（按两种模式分流）。
void ComputeRouteAction::on_tick()
{
  bool use_poses = false, use_start = false;
  // 读取模式选择：true = 位姿模式，false = 图节点 ID 模式。
  getInput("use_poses", use_poses);
  if (use_poses) {
    // 位姿模式：终点用位姿表达（起点默认为机器人当前位置）。
    goal_.use_poses = true;
    getInput("goal", goal_.goal);

    // 可选：显式提供起点位姿。
    goal_.use_start = false;
    getInput("use_start", use_start);
    if (use_start) {
      getInput("start", goal_.start);
      goal_.use_start = true;
    }
  } else {
    // 图节点 ID 模式：直接用路网图中的节点编号指定起终点。
    getInput("start_id", goal_.start_id);
    getInput("goal_id", goal_.goal_id);
    goal_.use_start = false;
    goal_.use_poses = false;
  }
}

// on_success 钩子：路由规划成功，把三个结果写入黑板。
BT::NodeStatus ComputeRouteAction::on_success()
{
  setOutput("path", result_.result->path);
  setOutput("route", result_.result->route);
  setOutput("planning_time", result_.result->planning_time);
  return BT::NodeStatus::SUCCESS;
}

// 复位输出端口：把 path/route/planning_time 置为空值，
// 防止下游节点使用上一次的过期结果。
void ComputeRouteAction::resetPorts()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  nav2_msgs::msg::Route empty_route;
  setOutput("route", empty_route);
  setOutput("planning_time", builtin_interfaces::msg::Duration());
}

// on_aborted 钩子：服务器中止（无路由等）时清空输出并失败。
BT::NodeStatus ComputeRouteAction::on_aborted()
{
  resetPorts();
  return BT::NodeStatus::FAILURE;
}

// on_cancelled 钩子：被取消时清空输出；主动取消返回 SUCCESS。
BT::NodeStatus ComputeRouteAction::on_cancelled()
{
  resetPorts();
  return BT::NodeStatus::SUCCESS;
}

// halt 重写：节点被中断时先清空输出端口，再走基类清理
// （取消进行中的 action、复位状态为 IDLE）。
void ComputeRouteAction::halt()
{
  resetPorts();
  BtActionNode::halt();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "ComputeRoute" 标签绑定到本类构建器。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputeRouteAction>(
        name, "compute_route", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputeRouteAction>(
    "ComputeRoute", builder);
}
