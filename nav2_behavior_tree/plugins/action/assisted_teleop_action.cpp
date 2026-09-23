// Copyright (c) 2022 Joshua Wallace
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
// 【文件说明】assisted_teleop_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 AssistedTeleop 行为树节点：调用 nav2_behaviors 服务器的
// "assisted_teleop" ROS2 Action（nav2_msgs::action::AssistedTeleop），
// 启用"辅助遥控"模式——人工遥控指令经过碰撞检测/避障过滤后再下发给底盘，
// 兼顾人工干预与安全性。
//
// 【在 Nav2 导航流程中的位置】
// 用于需要人工接管但保留安全防护的场景；也可作为恢复行为的一种
// （等待人工脱困）。
//
// 【行为树核心概念】
// - 输入端口：time_allowance（最长执行时间，秒）、is_recovery（是否为
//   恢复行为：影响恢复计数和 on_aborted 的返回值）。
// - on_aborted 钩子的亮点：作为恢复行为时中止返回 FAILURE（触发上游
//   重试），作为普通行为时中止返回 SUCCESS（人工退出视为正常结束）。
// - XML 使用示例：
//   <AssistedTeleop time_allowance="60" is_recovery="false"/>
// =============================================================================

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/assisted_teleop_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：委托基类创建 "assisted_teleop" action 客户端，并读取端口。
AssistedTeleopAction::AssistedTeleopAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::AssistedTeleop>(xml_tag_name, action_name, conf)
{
  // 读取允许的最长执行时间（秒）。
  double time_allowance;
  getInput("time_allowance", time_allowance);
  // 读取"是否为恢复行为"标志。
  getInput("is_recovery", is_recovery_);

  // Populate the input message
  // 填充 goal 的时限字段（防止动作无限占用）。
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

// on_tick 钩子：每次动作开始时按需递增恢复计数。
void AssistedTeleopAction::on_tick()
{
  // 作为恢复行为时递增黑板 number_recoveries 计数。
  if (is_recovery_) {
    increment_recovery_count();
  }
}

// on_aborted 钩子：动作被服务器中止时的状态分流。
BT::NodeStatus AssistedTeleopAction::on_aborted()
{
  // 恢复模式下：中止=恢复失败，返回 FAILURE 让上游继续重试；
  // 普通模式下：中止（如人工退出）视为正常，返回 SUCCESS。
  return is_recovery_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "AssistedTeleop" 标签绑定到本类构建器。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::AssistedTeleopAction>(
        name, "assisted_teleop", config);
    };

  factory.registerBuilder<nav2_behavior_tree::AssistedTeleopAction>("AssistedTeleop", builder);
}
