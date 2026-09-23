// Copyright (c) 2023 Alberto J. Tudela Roldán
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
// 【文件说明】is_battery_charging_condition.cpp
// 【所属模块】nav2_behavior_tree/plugins/condition —— Nav2 行为树的条件节点插件
//
// 实现 IsBatteryCharging 行为树节点：订阅电池状态话题，判断机器人
// 是否正在充电。用于充电桩导航流程（到达充电桩后等待充电开始）。
//
// 【在 Nav2 导航流程中的位置】
// 充电相关任务的条件判断：IsBatteryCharging 为真表示已开始充电。
// 与 IsBatteryLow（电量低）互补，常成对出现在电量管理流程中。
//
// 【BehaviorTree.CPP 与 ROS2 API 要点】
// - 条件节点：正在充电 -> SUCCESS；否则 FAILURE。
// - 订阅 sensor_msgs::msg::BatteryState（默认话题 "/battery_status"，
//   可用端口 battery_topic 覆盖）；回调挂专属回调组，
//   tick 中 spin_some 泵取最新电池状态。
// - 判定依据：消息的 power_supply_status 字段是否为
//   POWER_SUPPLY_STATUS_CHARGING。
// - XML 使用示例：
//   <IsBatteryCharging battery_topic="/battery_status"/>
// =============================================================================

#include <string>

#include "nav2_behavior_tree/plugins/condition/is_battery_charging_condition.hpp"

namespace nav2_behavior_tree
{

// 构造函数：创建电池状态话题的订阅。
IsBatteryChargingCondition::IsBatteryChargingCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  battery_topic_("/battery_status"),
  is_battery_charging_(false)
{
  // 读取电池话题端口（默认 "/battery_status"）。
  getInput("battery_topic", battery_topic_);
  // 从黑板取共享 ROS2 节点，创建专属回调组
  // （false = 不加入默认执行器），tick 中手动泵取。
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

  // 订阅电池状态（SystemDefaultsQoS），回调挂专属回调组。
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  battery_sub_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsBatteryChargingCondition::batteryCallback, this, std::placeholders::_1),
    sub_option);
}

// tick 主函数：泵取回调并读取充电标志。
BT::NodeStatus IsBatteryChargingCondition::tick()
{
  // 把话题上待处理的电池状态刷进来。
  callback_group_executor_.spin_some();
  // 正在充电 -> SUCCESS；否则 FAILURE。
  if (is_battery_charging_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

// 话题回调：根据电源状态字段更新充电标志。
void IsBatteryChargingCondition::batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  // power_supply_status == CHARGING 表示外部电源已接入、正在充电。
  is_battery_charging_ =
    (msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "IsBatteryCharging" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsBatteryChargingCondition>("IsBatteryCharging");
}
