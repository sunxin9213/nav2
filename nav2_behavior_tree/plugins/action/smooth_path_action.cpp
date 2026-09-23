// Copyright (c) 2021 RoboTech Vision
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
// 【文件说明】smooth_path_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 SmoothPath 行为树节点：调用 nav2_smoother 的 "smooth_path" ROS2
// Action（nav2_msgs::action::SmoothPath），对全局规划得到的路径做平滑
// 处理（如简化的梯度下降、共轭梯度平滑器），提高跟踪舒适度与效率。
//
// 【在 Nav2 导航流程中的位置】
// 位于 ComputePathToPose/ThroughPoses 之后、FollowPath 之前（可选）：
// 规划 -> 平滑 -> 跟踪。是否启用由导航树 XML 决定。
//
// 【行为树核心概念】
// - 输入端口：unsmoothed_path（待平滑路径）、smoother_id（平滑器选择）、
//   max_smoothing_duration（最长平滑耗时，秒）、check_for_collisions
//   （平滑过程中是否做碰撞检查）。
// - 输出端口：smoothed_path（平滑后路径）、smoothing_duration（实际
//   耗时，秒）、was_completed（是否在时限内完成平滑）。
// - on_success 钩子把服务器结果经输出端口写回黑板。
// - XML 使用示例：
//   <SmoothPath unsmoothed_path="{path}" smoothed_path="{smoothed_path}"
//               smoother_id="simple_smoother" max_smoothing_duration="3.0"/>
// =============================================================================

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/smooth_path_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：委托基类创建 "smooth_path" action 客户端，无额外初始化。
SmoothPathAction::SmoothPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::SmoothPath>(xml_tag_name, action_name, conf)
{
}

// on_tick 钩子：动作开始前读取输入端口填充 goal_。
void SmoothPathAction::on_tick()
{
  // 待平滑的原始路径（通常来自黑板 "path" 键，即规划结果）。
  getInput("unsmoothed_path", goal_.path);
  // 平滑器 ID：允许动态选择平滑器插件。
  getInput("smoother_id", goal_.smoother_id);
  // 最长平滑耗时（秒）：限制平滑器的工作时间，转换为 Duration。
  double max_smoothing_duration;
  getInput("max_smoothing_duration", max_smoothing_duration);
  goal_.max_smoothing_duration = rclcpp::Duration::from_seconds(max_smoothing_duration);
  // 是否启用碰撞检查：平滑后逐点校验，避免平滑出穿墙路径。
  getInput("check_for_collisions", goal_.check_for_collisions);
}

// on_success 钩子：平滑成功时把结果写回黑板。
BT::NodeStatus SmoothPathAction::on_success()
{
  // 平滑后的路径（下游 FollowPath 应跟踪它）。
  setOutput("smoothed_path", result_.result->path);
  // 实际平滑耗时（Duration 转秒）。
  setOutput("smoothing_duration", rclcpp::Duration(result_.result->smoothing_duration).seconds());
  // 是否在时限内完整收敛（超时未完成时为 false，但路径仍可用）。
  setOutput("was_completed", result_.result->was_completed);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "SmoothPath" 标签绑定到本类构建器。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::SmoothPathAction>(
        name, "smooth_path", config);
    };

  factory.registerBuilder<nav2_behavior_tree::SmoothPathAction>(
    "SmoothPath", builder);
}
