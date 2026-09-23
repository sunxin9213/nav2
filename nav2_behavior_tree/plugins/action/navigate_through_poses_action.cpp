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
// 【文件说明】navigate_through_poses_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 NavigateThroughPoses 行为树节点：调用 bt_navigator 的
// "navigate_through_poses" ROS2 Action（nav2_msgs::action::NavigateThroughPoses），
// 让机器人依次经过一组途经点并最终到达最后一个点（多点导航）。
// 这是 Nav2 顶层导航的另一个入口，与 NavigateToPose 对应。
//
// 【行为树核心概念】
// - 输入端口：goals（途经点位姿数组，来自黑板 "goals" 键）、
//   behavior_tree（可选的自定义导航树 XML 路径）。
// - 本节点通常位于导航树根部：整个导航任务 = 一次该 action 调用；
//   上游（bt_navigator）收到 goal 后再驱动内部的规划/跟踪子树。
// - XML 使用示例：
//   <NavigateThroughPoses goals="{goals}" behavior_tree="my_tree.xml"/>
// =============================================================================

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/navigate_through_poses_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：委托基类创建 "navigate_through_poses" action 客户端，
// 无额外初始化（goal 字段在 on_tick 中填充）。
NavigateThroughPosesAction::NavigateThroughPosesAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::NavigateThroughPoses>(xml_tag_name, action_name, conf)
{
}

// on_tick 钩子：动作开始前从端口读取输入填充 goal_。
void NavigateThroughPosesAction::on_tick()
{
  // 读取途经点数组到 goal_.poses；读取失败时记错误日志并返回
  // （保留旧 goal，通常意味着导航到旧目标集）。
  if (!getInput("goals", goal_.poses)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "NavigateThroughPosesAction: goal not provided");
    return;
  }
  // 可选端口：自定义导航子树 XML 文件，替换默认树执行本次任务。
  getInput("behavior_tree", goal_.behavior_tree);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "NavigateThroughPoses" 标签绑定到本类构建器。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::NavigateThroughPosesAction>(
        name, "navigate_through_poses", config);
    };

  factory.registerBuilder<nav2_behavior_tree::NavigateThroughPosesAction>(
    "NavigateThroughPoses", builder);
}
