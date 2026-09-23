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
// 【文件说明】drive_on_heading_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 DriveOnHeading 行为树节点：调用 nav2_behaviors 服务器的
// "drive_on_heading" ROS2 Action（nav2_msgs::action::DriveOnHeading），
// 让机器人沿机体坐标系 X 轴正方向（机头方向）直线前进指定距离。
// 与 BackUp 结构完全相同，只是方向相反（前进而非后退）。
//
// 【在 Nav2 导航流程中的位置】
// 可用于狭窄通道的开环脱困、恢复行为，或简单的直线行进任务。
//
// 【行为树核心概念】
// - 输入端口：dist_to_travel（前进距离，米）、speed（速度大小，米/秒）、
//   time_allowance（最长执行时间，秒）。
// - goal.target 为机体坐标系位移向量（仅 x 分量），服务器据此直线行驶。
// - XML 使用示例：
//   <DriveOnHeading dist_to_travel="0.15" speed="0.05" time_allowance="10"/>
// =============================================================================

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/drive_on_heading_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：委托基类创建 "drive_on_heading" action 客户端，并读取端口填充 goal。
DriveOnHeadingAction::DriveOnHeadingAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::DriveOnHeading>(xml_tag_name, action_name, conf)
{
  // 读取前进距离、速度和允许的最长执行时间。
  double dist;
  getInput("dist_to_travel", dist);
  double speed;
  getInput("speed", speed);
  double time_allowance;
  getInput("time_allowance", time_allowance);

  // Populate the input message
  // 填充 goal：target 为机体坐标系位移（仅 x 分量 = 前进距离），
  // speed 为行驶速度，time_allowance 防止动作无限执行。
  goal_.target.x = dist;
  goal_.target.y = 0.0;
  goal_.target.z = 0.0;
  goal_.speed = speed;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "DriveOnHeading" 标签绑定到本类构建器。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::DriveOnHeadingAction>(
        name, "drive_on_heading", config);
    };

  factory.registerBuilder<nav2_behavior_tree::DriveOnHeadingAction>("DriveOnHeading", builder);
}
