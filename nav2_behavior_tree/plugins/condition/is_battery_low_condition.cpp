// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2019 Intel Corporation
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
// 【文件说明】is_battery_low_condition.cpp
// 【所属模块】nav2_behavior_tree/plugins/condition —— Nav2 行为树的条件节点插件
//
// 实现 IsBatteryLow 行为树节点：订阅电池状态话题，判断电池是否低于
// 阈值（电量百分比或电压二选一）。用于低电量触发回充等流程。
//
// 【在 Nav2 导航流程中的位置】
// 电量管理：IsBatteryLow 为真时切换到"返回充电桩"子树
// （通过 Fallback/条件分支实现），与 IsBatteryCharging（正在充电）互补。
//
// 【BehaviorTree.CPP 与 ROS2 API 要点】
// - 条件节点：电量低 -> SUCCESS；否则 FAILURE。
// - 订阅 sensor_msgs::msg::BatteryState（默认话题 "/battery_status"）；
//   回调挂专属回调组，tick 中 spin_some 泵取最新状态。
// - 输入端口：
//   battery_topic（话题名）、min_battery（阈值）、is_voltage
//   （true 按"电压(V)"比较，false 按"百分比(0~1)"比较）。
// - XML 使用示例：
//   <IsBatteryLow battery_topic="/battery_status" min_battery="0.2"
//                 is_voltage="false"/>
// =============================================================================

#include <string>

#include "nav2_behavior_tree/plugins/condition/is_battery_low_condition.hpp"

namespace nav2_behavior_tree
{

// 构造函数：读取端口并创建电池状态订阅。
IsBatteryLowCondition::IsBatteryLowCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  battery_topic_("/battery_status"),
  min_battery_(0.0),
  is_voltage_(false),
  is_battery_low_(false)
{
  // 读取阈值、话题名、比较模式（电压 or 百分比）。
  getInput("min_battery", min_battery_);
  getInput("battery_topic", battery_topic_);
  getInput("is_voltage", is_voltage_);
  // 从黑板取共享 ROS2 节点，创建专属回调组
  // （false = 不加入默认执行器），tick 中手动泵取回调。
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  // 订阅电池状态话题（SystemDefaultsQoS），回调挂专属回调组。
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsBatteryLowCondition::batteryCallback, this, std::placeholders::_1),
    sub_option);
}

// tick 主函数：泵取回调并读取电量标志。
BT::NodeStatus IsBatteryLowCondition::tick()
{
  // 把话题上待处理的电池状态刷进来。
  callback_group_executor_.spin_some();
  // 电量低 -> SUCCESS；否则 FAILURE。
  if (is_battery_low_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

// 话题回调：按选定模式（电压/百分比）更新低电量标志。
void IsBatteryLowCondition::batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  if (is_voltage_) {
    // 电压模式：电压值（伏特）低于阈值即低电量。
    is_battery_low_ = msg->voltage <= min_battery_;
  } else {
    // 百分比模式：剩余电量（0.0~1.0）低于阈值即低电量。
    is_battery_low_ = msg->percentage <= min_battery_;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "IsBatteryLow" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsBatteryLowCondition>("IsBatteryLow");
}
