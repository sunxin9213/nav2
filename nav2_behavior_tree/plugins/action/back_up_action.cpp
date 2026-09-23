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
// 【文件说明】back_up_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 BackUp 行为树节点：调用 nav2_behaviors 服务器的 "backup" ROS2 Action
// （nav2_msgs::action::BackUp），让机器人沿机体坐标系 X 轴负方向
// （即倒车方向）直线后退指定距离。是导航失败后的典型恢复行为。
//
// 【在 Nav2 导航流程中的位置】
// 出现在默认导航树的 RecoveryBranch（恢复分支）：规划/跟踪失败时先 BackUp
// 脱困，再重新规划。
//
// 【行为树核心概念】
// - 输入端口：backup_dist（后退距离，米，负值表示后退）、backup_speed
//   （后退速度大小）、time_allowance（最长执行时间，秒）。
// - goal.target 是机器人机体坐标系下的目标点（x=距离, y=z=0），
//   服务器据此在机体坐标系内开环/闭环后退。
// - on_tick 中递增黑板 number_recoveries（BackUp 总是恢复行为）。
// - XML 使用示例：
//   <BackUp backup_dist="0.30" backup_speed="0.05" time_allowance="10"/>
// =============================================================================

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/back_up_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：委托基类创建 "backup" action 客户端，并读取端口填充 goal。
BackUpAction::BackUpAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::BackUp>(xml_tag_name, action_name, conf)
{
  // 读取后退距离与速度。
  double dist;
  getInput("backup_dist", dist);
  double speed;
  getInput("backup_speed", speed);
  // 读取允许的最长执行时间（秒）。
  double time_allowance;
  getInput("time_allowance", time_allowance);

  // Populate the input message
  // 填充 goal：target 为机体坐标系下的位移向量（仅 x 分量有值，
  // y/z 保持 0 —— 即沿机头方向倒退），speed 为速度大小。
  goal_.target.x = dist;
  goal_.target.y = 0.0;
  goal_.target.z = 0.0;
  goal_.speed = speed;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

// on_tick 钩子：每次动作开始时递增恢复计数器。
void BackUpAction::on_tick()
{
  // BackUp 属于恢复行为：递增黑板 number_recoveries，
  // 供 RecoveryNode/监控端统计恢复次数。
  increment_recovery_count();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "BackUp" 标签绑定到本类构建器。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::BackUpAction>(
        name, "backup", config);
    };

  factory.registerBuilder<nav2_behavior_tree::BackUpAction>("BackUp", builder);
}
