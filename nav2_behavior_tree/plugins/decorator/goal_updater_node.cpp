// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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
// 【文件说明】goal_updater_node.cpp
// 【所属模块】nav2_behavior_tree/plugins/decorator —— Nav2 行为树的装饰节点插件
//
// 实现 GoalUpdater（目标更新器）装饰节点：允许在导航过程中通过 ROS2
// 话题动态更新目标点。订阅 "goal_update" 话题（PoseStamped），
// 若收到的新目标时间戳比当前目标新，则用新目标替换当前目标写入黑板。
// 与 FollowPath 节点的"运行中更新路径"配合，实现边走边改目标。
//
// 【BehaviorTree.CPP 与 ROS2 API 要点】
// - 装饰节点：tick 时先做目标更新逻辑，再透传 tick 给子节点。
// - 输入端口 input_goal（当前目标）、输出端口 output_goal（更新后目标）。
// - 订阅话题名从参数 goal_updater_topic 读取（默认 "goal_update"），
//   QoS 用 SystemDefaultsQoS；回调挂专属回调组，tick 中 spin_some 泵取。
// - 时间戳比较防止"旧消息覆盖新目标"的竞态。
// - XML 使用示例：
//   <GoalUpdater input_goal="{goal}" output_goal="{goal}">
//     <ComputePathToPose goal="{goal}" .../>
//   </GoalUpdater>
// =============================================================================

#include <string>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/decorator/goal_updater_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

// 构造函数：创建目标更新话题的订阅。
GoalUpdater::GoalUpdater(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  // 从黑板取共享 ROS2 节点。
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  // 创建本节点专属回调组（false = 不加入默认执行器），隔离回调并发。
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  // 从参数读取目标更新话题名（未设置时用默认 "goal_update"）。
  std::string goal_updater_topic;
  node_->get_parameter_or<std::string>("goal_updater_topic", goal_updater_topic, "goal_update");

  // 订阅目标更新话题（PoseStamped），回调挂专属回调组。
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_updater_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&GoalUpdater::callback_updated_goal, this, _1),
    sub_option);
}

// tick 主函数：先用话题上的新目标更新黑板，再透传 tick 给子节点。
inline BT::NodeStatus GoalUpdater::tick()
{
  geometry_msgs::msg::PoseStamped goal;

  // 读取当前目标（来自黑板，通常是导航任务的初始目标）。
  getInput("input_goal", goal);

  // 泵取回调：把话题上待处理的目标更新刷进来。
  callback_group_executor_.spin_some();

  // 若已收到过话题目标（时间戳非零）：比较时间戳决定是否采纳。
  if (last_goal_received_.header.stamp != rclcpp::Time(0)) {
    auto last_goal_received_time = rclcpp::Time(last_goal_received_.header.stamp);
    auto goal_time = rclcpp::Time(goal.header.stamp);
    if (last_goal_received_time > goal_time) {
      // 话题目标更新：采纳它作为新目标。
      goal = last_goal_received_;
    } else {
      // 话题目标比当前目标旧（乱序/过期消息）：忽略并告警。
      RCLCPP_WARN(
        node_->get_logger(), "The timestamp of the received goal (%f) is older than the "
        "current goal (%f). Ignoring the received goal.",
        last_goal_received_time.seconds(), goal_time.seconds());
    }
  }

  // 把（可能更新过的）目标写回黑板。
  setOutput("output_goal", goal);
  // 透传 tick 给子节点（子节点会读到更新后的目标）。
  return child_node_->executeTick();
}

// 话题回调：缓存收到的最新目标（tick 中做时间戳比较后决定是否采纳）。
void
GoalUpdater::callback_updated_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_goal_received_ = *msg;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "GoalUpdater" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalUpdater>("GoalUpdater");
}
