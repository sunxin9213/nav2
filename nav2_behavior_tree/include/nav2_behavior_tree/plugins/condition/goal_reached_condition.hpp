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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_REACHED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_REACHED_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when a specified goal
 * is reached and FAILURE otherwise
 */
class GoalReachedCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::GoalReachedCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  GoalReachedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  GoalReachedCondition() = delete;

  /**
   * @brief A destructor for nav2_behavior_tree::GoalReachedCondition
   */
  ~GoalReachedCondition() override;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
// 初始化 NodeStatus tick 对象，读取参数并准备运行时依赖。
  BT::NodeStatus tick() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
// 初始化 ROS 节点、参数和 TF/状态依赖，准备后续判断逻辑。
  void initialize();

  /**
   * @brief Checks if the current robot pose lies within a given distance from the goal
   * @return bool true when goal is reached, false otherwise
   */
// 判断当前机器人位姿是否已进入目标阈值范围内。
  bool isGoalReached();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
// 初始化 PortsList providedPorts 对象，读取参数并准备运行时依赖。
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
// 初始化 InputPort<std::string> 对象，读取参数并准备运行时依赖。
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame")
    };
  }

protected:
  /**
   * @brief Cleanup function
   */
// 清理节点持有的资源，避免重复执行造成状态残留。
  void cleanup()
  {}

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  bool initialized_;
  double goal_reached_tol_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_REACHED_CONDITION_HPP_
