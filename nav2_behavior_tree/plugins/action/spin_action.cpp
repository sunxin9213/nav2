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
// 【文件说明】spin_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 Spin 行为树节点：调用 nav2_behaviors 服务器的 "spin" ROS2 Action
// （nav2_msgs::action::Spin），让机器人原地旋转指定的角度（弧度）。
// 主要用于导航失败的恢复行为（旋转寻找新方向）或搜索定位。
//
// 【在 Nav2 导航流程中的位置】
// 常出现在默认导航树的 RecoveryBranch（恢复分支）中，与 BackUp、Wait 等
// 一起作为重试策略。
//
// 【行为树核心概念】
// - 输入端口：spin_dist（旋转角度，弧度）、time_allowance（允许的最长
//   执行时间，秒）、is_recovery（是否为恢复行为，决定是否递增黑板上的
//   恢复计数器 number_recoveries）。
// - 构造函数中一次性读取端口填充 goal_（参数在树运行期间不变）；
//   on_tick 中按 is_recovery 决定是否计数。
// - XML 使用示例（恢复分支中的用法）：
//   <Spin spin_dist="1.57" time_allowance="10" is_recovery="true"/>
// =============================================================================

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/spin_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：委托基类创建 "spin" action 客户端，并读取端口填充 goal。
SpinAction::SpinAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::Spin>(xml_tag_name, action_name, conf)
{
  // 读取旋转角度（弧度），填入 goal 的 target_yaw 字段。
  double dist;
  getInput("spin_dist", dist);
  // 读取允许的最长执行时间（秒），转换为 ROS Duration 填入 goal。
  double time_allowance;
  getInput("time_allowance", time_allowance);
  goal_.target_yaw = dist;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
  // 读取"是否为恢复行为"标志：影响 on_tick 中是否递增恢复计数。
  getInput("is_recovery", is_recovery_);
}

// on_tick 钩子：每次动作开始时调用。
void SpinAction::on_tick()
{
  // 若本节点作为恢复行为使用：递增黑板上的 number_recoveries 计数器，
  // 供 RecoveryNode/监控端统计本任务已执行的恢复次数。
  if (is_recovery_) {
    increment_recovery_count();
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "Spin" 标签绑定到本类构建器。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::SpinAction>(name, "spin", config);
    };

  factory.registerBuilder<nav2_behavior_tree::SpinAction>("Spin", builder);
}
