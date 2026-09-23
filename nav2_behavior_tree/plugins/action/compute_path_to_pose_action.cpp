// Copyright (c) 2018 Intel Corporation
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
// 【文件说明】compute_path_to_pose_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 ComputePathToPose 行为树节点：调用 nav2_planner 的
// "compute_path_to_pose" ROS2 Action（nav2_msgs::action::ComputePathToPose），
// 从起点（默认为机器人当前位置）到单个目标点规划一条全局路径。
//
// 【在 Nav2 导航流程中的位置】
// 典型导航树中位于 PipelineSequence 的第一段：先算路（本节点），
// 再把结果写入黑板 "path" 键，交给下游的 FollowPath 节点执行局部跟踪。
//
// 【行为树核心概念】
// - 输入端口：goal（目标位姿）、planner_id（规划器选择，如 "GridBased"）、
//   start（可选的显式起点，默认用当前位姿）。
// - 输出端口：path（规划得到的 nav_msgs::msg::Path，写入黑板供
//   FollowPath 读取）。
// - 本类重写了 on_success/on_aborted/on_cancelled/halt 四个钩子，
//   在各种结束路径上都把 path 输出端口置为一致的值（成功=真实路径，
//   其他=空路径），保证黑板状态确定性。
// - XML 使用示例：
//   <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
// =============================================================================

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/compute_path_to_pose_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：委托基类创建 "compute_path_to_pose" action 客户端，
// 无额外初始化。
ComputePathToPoseAction::ComputePathToPoseAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::ComputePathToPose>(xml_tag_name, action_name, conf)
{
}

// on_tick 钩子：动作开始前读取输入端口填充 goal_。
void ComputePathToPoseAction::on_tick()
{
  // 目标点：必填，来自黑板（通常由 NavigateToPose 写入）。
  getInput("goal", goal_.goal);
  // 规划器 ID：允许 XML 中动态选择用哪个规划器插件。
  getInput("planner_id", goal_.planner_id);
  // 可选起点：若提供了显式起点则置 use_start=true，
  // 服务器将用它代替机器人当前位姿作为规划起点。
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }
}

// on_success 钩子：规划成功时触发。
BT::NodeStatus ComputePathToPoseAction::on_success()
{
  // 把服务器返回的路径通过输出端口写入黑板 "path" 键。
  setOutput("path", result_.result->path);
  return BT::NodeStatus::SUCCESS;
}

// on_aborted 钩子：服务器中止（如无可行路径）时触发。
BT::NodeStatus ComputePathToPoseAction::on_aborted()
{
  // 输出空路径，避免下游节点使用上一次的过期路径。
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::FAILURE;
}

// on_cancelled 钩子：动作被取消（halt/新目标）时触发。
BT::NodeStatus ComputePathToPoseAction::on_cancelled()
{
  // 同样输出空路径；返回 SUCCESS 表示"主动取消不算失败"。
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::SUCCESS;
}

// halt 重写：节点被中断（父节点放弃本节点）时触发。
void ComputePathToPoseAction::halt()
{
  // 先把黑板上的 path 置空，防止树的其他分支读到过期路径，
  // 再调用基类 halt（取消进行中的 action 并复位状态为 IDLE）。
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  BtActionNode::halt();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "ComputePathToPose" 标签绑定到本类构建器。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputePathToPoseAction>(
        name, "compute_path_to_pose", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputePathToPoseAction>(
    "ComputePathToPose", builder);
}
