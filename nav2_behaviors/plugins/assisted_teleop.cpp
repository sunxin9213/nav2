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

#include <utility>

#include "nav2_behaviors/plugins/assisted_teleop.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behaviors
{
// 初始化遥控速度缓存和 AssistedTeleop Action 状态。
AssistedTeleop::AssistedTeleop()
: TimedBehavior<AssistedTeleopAction>(),
  feedback_(std::make_shared<AssistedTeleopAction::Feedback>())
{}

void AssistedTeleop::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // projection_time 单位为秒，表示将当前遥控速度向前预测多长时间。
  nav2_util::declare_parameter_if_not_declared(
    node,
    "projection_time", rclcpp::ParameterValue(1.0));

  nav2_util::declare_parameter_if_not_declared(
    node,
    "simulation_time_step", rclcpp::ParameterValue(0.1));
  // simulation_time_step 默认 0.1 秒，决定碰撞预测轨迹的离散时间间隔。

  nav2_util::declare_parameter_if_not_declared(
    node,
    "cmd_vel_teleop", rclcpp::ParameterValue(std::string("cmd_vel_teleop")));
  // cmd_vel_teleop 是遥控输入 Twist topic；SystemDefaultsQoS 遵循 ROS 2 系统默认通信策略。

  node->get_parameter("projection_time", projection_time_);
  node->get_parameter("simulation_time_step", simulation_time_step_);

  std::string cmd_vel_teleop;
  node->get_parameter("cmd_vel_teleop", cmd_vel_teleop);

  vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_teleop, rclcpp::SystemDefaultsQoS(),
    std::bind(
      &AssistedTeleop::teleopVelocityCallback,
      this, std::placeholders::_1));

  preempt_teleop_sub_ = node->create_subscription<std_msgs::msg::Empty>(
    "preempt_teleop", rclcpp::SystemDefaultsQoS(),
    std::bind(
      &AssistedTeleop::preemptTeleopCallback,
      this, std::placeholders::_1));
}

// 接收 AssistedTeleop goal，清除上次抢占状态并设置可选的执行截止时间。
Status AssistedTeleop::onRun(const std::shared_ptr<const AssistedTeleopAction::Goal> command)
{
  preempt_teleop_ = false;
  command_time_allowance_ = command->time_allowance;
  end_time_ = this->clock_->now() + command_time_allowance_;
  return Status::SUCCEEDED;
}

// Action 完成后清空缓存速度和抢占标志，避免下一次 goal 继承旧遥控命令。
void AssistedTeleop::onActionCompletion()
{
  teleop_twist_ = geometry_msgs::msg::Twist();
  preempt_teleop_ = false;
}

// 根据遥控速度预测未来轨迹；若接近障碍物则按剩余安全时间比例缩放速度。
Status AssistedTeleop::onCycleUpdate()
{
  feedback_->current_teleop_duration = elasped_time_;
  action_server_->publish_feedback(feedback_);

  rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN_STREAM(
      logger_,
      "Exceeded time allowance before reaching the " << behavior_name_.c_str() <<
        "goal - Exiting " << behavior_name_.c_str());
    return Status::FAILED;
  }

  // 遥控端通过 preempt_teleop topic 发布空消息，表示用户主动结束 AssistedTeleop。
  if (preempt_teleop_) {
    stopRobot();
    return Status::SUCCEEDED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Current robot pose is not available for " <<
        behavior_name_.c_str());
    return Status::FAILED;
  }
  geometry_msgs::msg::Pose2D projected_pose;
  projected_pose.x = current_pose.pose.position.x;
  projected_pose.y = current_pose.pose.position.y;
  projected_pose.theta = tf2::getYaw(current_pose.pose.orientation);

  geometry_msgs::msg::Twist scaled_twist = teleop_twist_;
  for (double time = simulation_time_step_; time < projection_time_;
    time += simulation_time_step_)
  {
    projected_pose = projectPose(projected_pose, teleop_twist_, simulation_time_step_);

    if (!collision_checker_->isCollisionFree(projected_pose)) {
      if (time == simulation_time_step_) {
        RCLCPP_DEBUG_STREAM_THROTTLE(
          logger_,
          *clock_,
          1000,
          behavior_name_.c_str() << " collided on first time step, setting velocity to zero");
        scaled_twist.linear.x = 0.0f;
        scaled_twist.linear.y = 0.0f;
        scaled_twist.angular.z = 0.0f;
        break;
      } else {
        RCLCPP_DEBUG_STREAM_THROTTLE(
          logger_,
          *clock_,
          1000,
          behavior_name_.c_str() << " collision approaching in " << time << " seconds");
        double scale_factor = time / projection_time_;
        scaled_twist.linear.x *= scale_factor;
        scaled_twist.linear.y *= scale_factor;
        scaled_twist.angular.z *= scale_factor;
        break;
      }
    }
  }
  vel_pub_->publish(std::move(scaled_twist));

  return Status::RUNNING;
}

// 使用二维刚体运动学模型，将 Twist 在 projection_time 内积分到预测位姿。
geometry_msgs::msg::Pose2D AssistedTeleop::projectPose(
  const geometry_msgs::msg::Pose2D & pose,
  const geometry_msgs::msg::Twist & twist,
  double projection_time)
{
  geometry_msgs::msg::Pose2D projected_pose = pose;

  projected_pose.x += projection_time * (
    twist.linear.x * cos(pose.theta) +
    twist.linear.y * sin(pose.theta));

  projected_pose.y += projection_time * (
    twist.linear.x * sin(pose.theta) -
    twist.linear.y * cos(pose.theta));

  projected_pose.theta += projection_time * twist.angular.z;

  return projected_pose;
}

// 缓存最新的遥控 Twist；实际发布由 onCycleUpdate 统一进行碰撞检查和限速。
void AssistedTeleop::teleopVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  teleop_twist_ = *msg;
}

// 收到空消息即设置抢占标志，下一次控制循环会停止机器人并完成 Action。
void AssistedTeleop::preemptTeleopCallback(const std_msgs::msg::Empty::SharedPtr)
{
  preempt_teleop_ = true;
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::AssistedTeleop, nav2_core::Behavior)
