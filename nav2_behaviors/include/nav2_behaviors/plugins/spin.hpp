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

#ifndef NAV2_BEHAVIORS__PLUGINS__SPIN_HPP_
#define NAV2_BEHAVIORS__PLUGINS__SPIN_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace nav2_behaviors
{
using SpinAction = nav2_msgs::action::Spin;

/**
 * @class nav2_behaviors::Spin
 * @brief An action server behavior for spinning in
 */
class Spin : public TimedBehavior<SpinAction>
{
public:
  /**
   * @brief A constructor for nav2_behaviors::Spin
   */
  // 创建旋转行为的反馈对象和运行时状态；参数在 onConfigure 中读取。
  Spin();
  // 释放 Spin 自身资源，基类负责通用 Action Server 和速度发布器清理。
  ~Spin();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  // 读取目标角度并建立起始 yaw、截止时间，准备进入周期控制。
  Status onRun(const std::shared_ptr<const SpinAction::Goal> command) override;

  /**
   * @brief Configuration of behavior action
   */
  // 声明并读取旋转速度、角加速度和前向模拟时间等参数。
  void onConfigure() override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  // 根据已旋转角度发布角速度，接近目标时减速并在完成或失败时停止机器人。
  Status onCycleUpdate() override;

protected:
  /**
   * @brief Check if pose is collision free
   * @param distance Distance to check forward
   * @param cmd_vel current commanded velocity
   * @param pose2d Current pose
   * @return is collision free or not
   */
  // 按当前角速度预测未来 yaw，在 Costmap 中检查旋转轨迹是否会碰撞。
  bool isCollisionFree(
    const double & distance,
    geometry_msgs::msg::Twist * cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d);

  SpinAction::Feedback::SharedPtr feedback_;

  double min_rotational_vel_;
  double max_rotational_vel_;
  double rotational_acc_lim_;
  double cmd_yaw_;
  double prev_yaw_;
  double relative_yaw_;
  double simulate_ahead_time_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__SPIN_HPP_
