// Copyright (c) 2018 Samsung Research America
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
// 【文件说明】wait_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 Wait 行为树节点：调用 nav2_behaviors 服务器的 "wait" ROS2 Action
// （nav2_msgs::action::Wait），让机器人原地等待指定的秒数。
//
// 【在 Nav2 导航流程中的位置】
// 主要用于导航恢复策略（RecoveryBranch）：规划或跟踪失败后先等待一段
// 时间（例如等待临时障碍物离开）再重试；也可用于任务流程中的定时停留。
//
// 【行为树核心概念】
// - 输入端口：wait_duration（等待时长，秒，int 类型）。
// - 构造时一次性读取端口填充 goal（时间参数在动作期间不变）；
//   goal_.time.sec 直接用秒字段表达等待时长。
// - on_tick 中递增恢复计数器（Wait 通常作为恢复行为使用）。
// - XML 使用示例：
//   <Wait wait_duration="5"/>
// =============================================================================

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/wait_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：委托基类创建 "wait" action 客户端，并读取等待时长填充 goal。
WaitAction::WaitAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::Wait>(xml_tag_name, action_name, conf)
{
  // 读取等待时长（秒）。
  int duration;
  getInput("wait_duration", duration);
  // 防御性处理：时长非正时取绝对值并告警
  // （避免服务器端因负值立即失败或行为异常）。
  if (duration <= 0) {
    RCLCPP_WARN(
      node_->get_logger(), "Wait duration is negative or zero "
      "(%i). Setting to positive.", duration);
    duration *= -1;
  }

  // 用秒字段填充 goal（builtin_interfaces 的 sec/nanosec 结构）。
  goal_.time.sec = duration;
}

// on_tick 钩子：每次动作开始时递增恢复计数器。
void WaitAction::on_tick()
{
  // Wait 属于恢复行为：递增黑板 number_recoveries 计数。
  increment_recovery_count();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "Wait" 标签绑定到本类构建器。
BT_REGISTER_NODES(factory)
{
  // 构建器 lambda：固定传入 action 服务名 "wait"。
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::WaitAction>(name, "wait", config);
    };

  factory.registerBuilder<nav2_behavior_tree::WaitAction>("Wait", builder);
}
