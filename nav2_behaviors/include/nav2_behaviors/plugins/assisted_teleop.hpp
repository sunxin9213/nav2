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

#ifndef NAV2_BEHAVIORS__PLUGINS__ASSISTED_TELEOP_HPP_
#define NAV2_BEHAVIORS__PLUGINS__ASSISTED_TELEOP_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"

namespace nav2_behaviors
{
using AssistedTeleopAction = nav2_msgs::action::AssistedTeleop;

/**
 * @class nav2_behaviors::AssistedTeleop
 * @brief An action server behavior for assisted teleop
 */
class AssistedTeleop : public TimedBehavior<AssistedTeleopAction>
{
public:
  // 创建遥控速度反馈对象；订阅者和参数在生命周期 configure 阶段创建。
  AssistedTeleop();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  // 开始一次辅助遥控 Action，清除抢占标志并设置可选超时。
  Status onRun(const std::shared_ptr<const AssistedTeleopAction::Goal> command) override;

  /**
   * @brief func to run at the completion of the action
   */
  // 清空遥控速度和结束标志，避免下一个 goal 继承上一轮输入。
  void onActionCompletion() override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  // 读取遥控输入、预测未来位姿并按碰撞风险缩放 Twist。
  Status onCycleUpdate() override;

protected:
  /**
   * @brief Configuration of behavior action
   */
  // 声明预测时间和遥控 topic 参数，并创建 Twist/抢占消息订阅。
  void onConfigure() override;

  /**
   * @brief project a position
   * @param pose initial pose to project
   * @param twist velocity to project pose by
   * @param projection_time time to project by
   */
  // 使用当前 Pose2D 和 Twist 计算指定时间后的二维预测位姿。
  geometry_msgs::msg::Pose2D projectPose(
    const geometry_msgs::msg::Pose2D & pose,
    const geometry_msgs::msg::Twist & twist,
    double projection_time);

  /**
   * @brief Callback function for velocity subscriber
   * @param msg received Twist message
   */
  // 缓存最新遥控速度；控制循环会在发布前执行碰撞检查。
  void teleopVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Callback function to preempt assisted teleop
   * @param msg empty message
   */
  // 接收空消息作为用户主动结束遥控的信号。
  void preemptTeleopCallback(const std_msgs::msg::Empty::SharedPtr msg);

  AssistedTeleopAction::Feedback::SharedPtr feedback_;

  // parameters
  double projection_time_;
  double simulation_time_step_;

  geometry_msgs::msg::Twist teleop_twist_;
  bool preempt_teleop_{false};

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr preempt_teleop_sub_;

  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};
}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__ASSISTED_TELEOP_HPP_
