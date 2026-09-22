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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_STUCK_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_STUCK_CONDITION_HPP_

#include <string>
#include <atomic>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/odometry.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that tracks robot odometry and returns SUCCESS
 * if robot is stuck somewhere and FAILURE otherwise
 */
class IsStuckCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsStuckCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsStuckCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsStuckCondition() = delete;

  /**
   * @brief A destructor for nav2_behavior_tree::IsStuckCondition
   */
  ~IsStuckCondition() override;

  /**
   * @brief Callback function for odom topic
   * @param msg Shared pointer to nav_msgs::msg::Odometry::SharedPtr message
   */
// 实现该逻辑，并处理相关状态转换。
  void onOdomReceived(const typename nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
// 初始化 NodeStatus tick 对象，读取参数并准备运行时依赖。
  BT::NodeStatus tick() override;

  /**
   * @brief Function to log status when robot is stuck/free
   */
// 实现该逻辑，并处理相关状态转换。
  void logStuck(const std::string & msg) const;

  /**
   * @brief Function to approximate acceleration from the odom history
   */
// 实现该逻辑，并处理相关状态转换。
  void updateStates();

  /**
   * @brief Detect if robot bumped into something by checking for abnormal deceleration
   * @return bool true if robot is stuck, false otherwise
   */
// 检测机器人是否长时间卡住并需要执行恢复策略。
  bool isStuck();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts() {return {};}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread;

  std::atomic<bool> is_stuck_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::deque<nav_msgs::msg::Odometry> odom_history_;
  std::deque<nav_msgs::msg::Odometry>::size_type odom_history_size_;

  // Calculated states
  double current_accel_;

  // Robot specific paramters
  double brake_accel_limit_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_STUCK_CONDITION_HPP_
