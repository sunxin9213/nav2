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

#ifndef NAV2_BEHAVIORS__TIMED_BEHAVIOR_HPP_
#define NAV2_BEHAVIORS__TIMED_BEHAVIOR_HPP_

#include <memory>
#include <string>
#include <cmath>
#include <chrono>
#include <ctime>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_core/behavior.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace nav2_behaviors
{

enum class Status : int8_t
{
  SUCCEEDED = 1,
  FAILED = 2,
  RUNNING = 3,
};

using namespace std::chrono_literals;  //NOLINT

/**
 * @class nav2_behaviors::Behavior
 * @brief An action server Behavior base class implementing the action server and basic factory.
 */
template<typename ActionT>
class TimedBehavior : public nav2_core::Behavior
{
public:
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  /**
   * @brief A TimedBehavior constructor
   */
  // 默认以 10 Hz 执行控制循环；行为未激活前禁止处理 Action 请求。
  TimedBehavior()
  : action_server_(nullptr),
    cycle_frequency_(10.0),
    enabled_(false),
    transform_tolerance_(0.0)
  {
  }

  virtual ~TimedBehavior() = default;

  // Derived classes can override this method to catch the command and perform some checks
  // before getting into the main loop. The method will only be called
  // once and should return SUCCEEDED otherwise behavior will return FAILED.
  // 派生行为在这里读取 Action goal、检查输入约束并记录初始位姿或截止时间。
  virtual Status onRun(const std::shared_ptr<const typename ActionT::Goal> command) = 0;


  // This is the method derived classes should mainly implement
  // and will be called cyclically while it returns RUNNING.
  // Implement the behavior such that it runs some unit of work on each call
  // and provides a status. The Behavior will finish once SUCCEEDED is returned
  // It's up to the derived class to define the final commanded velocity.
  // 每次循环只推进一小步；返回 RUNNING 会继续循环，其他状态会结束当前 goal。
  virtual Status onCycleUpdate() = 0;

  // an opportunity for derived classes to do something on configuration
  // if they chose
  virtual void onConfigure()
  {
  }

  // an opportunity for derived classes to do something on cleanup
  // if they chose
  virtual void onCleanup()
  {
  }

  // an opportunity for a derived class to do something on action completion
  virtual void onActionCompletion()
  {
  }

  // configure the server on lifecycle setup
  // 在 configure 阶段读取共享参数、创建 Action Server 和 LifecyclePublisher；此时尚未真正发布速度。
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker) override
  {
    node_ = parent;
    auto node = node_.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    RCLCPP_INFO(logger_, "Configuring %s", name.c_str());

    behavior_name_ = name;
    tf_ = tf;

    // cycle_frequency 单位为 Hz，默认由服务端设为 10.0；它同时决定控制循环和前向碰撞模拟的离散步长。
    node->get_parameter("cycle_frequency", cycle_frequency_);
    // global_frame/robot_base_frame 定义 TF 查询的目标坐标系和机器人基座坐标系。
    node->get_parameter("global_frame", global_frame_);
    node->get_parameter("robot_base_frame", robot_base_frame_);
    // transform_tolerance 单位为秒，用于容忍 TF 数据相对当前时刻的时间差；过小可能因传感器延迟导致查询失败。
    node->get_parameter("transform_tolerance", transform_tolerance_);

    action_server_ = std::make_shared<ActionServer>(
      node, behavior_name_,
      std::bind(&TimedBehavior::execute, this));

    collision_checker_ = collision_checker;

    // QoS 深度 1 只保留最新速度命令，适合控制 topic：旧速度比丢弃旧消息更危险。
    vel_pub_ = node->template create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    onConfigure();
  }

  // Cleanup server on lifecycle transition
  // 清理阶段释放 Action Server 和速度发布器，避免节点重新配置时复用旧通信实体。
  void cleanup() override
  {
    action_server_.reset();
    vel_pub_.reset();
    onCleanup();
  }

  // Activate server on lifecycle transition
  // 激活阶段允许接收 goal，并使 LifecyclePublisher 可以向底盘发送速度。
  void activate() override
  {
    RCLCPP_INFO(logger_, "Activating %s", behavior_name_.c_str());

    vel_pub_->on_activate();
    action_server_->activate();
    enabled_ = true;
  }

  // Deactivate server on lifecycle transition
  // 停用阶段禁止新请求和速度发布，防止非活动节点继续控制机器人。
  void deactivate() override
  {
    vel_pub_->on_deactivate();
    action_server_->deactivate();
    enabled_ = false;
  }

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  std::string behavior_name_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  std::shared_ptr<ActionServer> action_server_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  double cycle_frequency_;
  double enabled_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
  rclcpp::Duration elasped_time_{0, 0};

  // Clock
  rclcpp::Clock::SharedPtr clock_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_behaviors")};

  // Main execution callbacks for the action server implementation calling the Behavior's
  // onRun and cycle functions to execute a specific behavior
  // Action Server 回调的统一执行入口：先做 goal 校验，再按 cycle_frequency 周期运行行为。
  void execute()
  {
    RCLCPP_INFO(logger_, "Running %s", behavior_name_.c_str());

    if (!enabled_) {
      RCLCPP_WARN(
        logger_,
        "Called while inactive, ignoring request.");
      return;
    }

    if (onRun(action_server_->get_current_goal()) != Status::SUCCEEDED) {
      RCLCPP_INFO(
        logger_,
        "Initial checks failed for %s", behavior_name_.c_str());
      action_server_->terminate_current();
      return;
    }

    auto start_time = clock_->now();

    // Initialize the ActionT result
    auto result = std::make_shared<typename ActionT::Result>();

    rclcpp::WallRate loop_rate(cycle_frequency_);

    while (rclcpp::ok()) {
      elasped_time_ = clock_->now() - start_time;
      if (action_server_->is_cancel_requested()) {
        // 取消请求必须先发送零速度，再用 terminate_all 返回当前耗时，确保机器人停止。
        RCLCPP_INFO(logger_, "Canceling %s", behavior_name_.c_str());
        stopRobot();
        result->total_elapsed_time = elasped_time_;
        action_server_->terminate_all(result);
        onActionCompletion();
        return;
      }

      // TODO(orduno) #868 Enable preempting a Behavior on-the-fly without stopping
      if (action_server_->is_preempt_requested()) {
        RCLCPP_ERROR(
          logger_, "Received a preemption request for %s,"
          " however feature is currently not implemented. Aborting and stopping.",
          behavior_name_.c_str());
        stopRobot();
        result->total_elapsed_time = clock_->now() - start_time;
        action_server_->terminate_current(result);
        onActionCompletion();
        return;
      }

      switch (onCycleUpdate()) {
        case Status::SUCCEEDED:
          // 行为成功后写入总耗时并完成当前 Action goal。
          RCLCPP_INFO(
            logger_,
            "%s completed successfully", behavior_name_.c_str());
          result->total_elapsed_time = clock_->now() - start_time;
          action_server_->succeeded_current(result);
          onActionCompletion();
          return;

        case Status::FAILED:
          // 行为失败或碰撞/超时后终止 goal；派生类负责在返回前停止运动。
          RCLCPP_WARN(logger_, "%s failed", behavior_name_.c_str());
          result->total_elapsed_time = clock_->now() - start_time;
          action_server_->terminate_current(result);
          onActionCompletion();
          return;

        case Status::RUNNING:

        default:
          loop_rate.sleep();
          break;
      }
    }
  }

  // Stop the robot with a commanded velocity
  // 发布全零 Twist 作为安全停止命令；调用者应在取消、失败和成功结束前执行它。
  void stopRobot()
  {
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.x = 0.0;
    cmd_vel->linear.y = 0.0;
    cmd_vel->angular.z = 0.0;

    vel_pub_->publish(std::move(cmd_vel));
  }
};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__TIMED_BEHAVIOR_HPP_
