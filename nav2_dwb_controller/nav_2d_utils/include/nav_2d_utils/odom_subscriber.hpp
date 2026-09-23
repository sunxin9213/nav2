/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NAV_2D_UTILS__ODOM_SUBSCRIBER_HPP_
#define NAV_2D_UTILS__ODOM_SUBSCRIBER_HPP_

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include "nav_2d_msgs/msg/twist2_d_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav_2d_utils
{

/**
 * @class OdomSubscriber
 * Wrapper for some common odometry operations. Subscribes to the topic with a mutex.
 */
// 里程计订阅器：对"订阅 /odom 并以线程安全方式缓存最新速度"这一常见模式的
// 封装。DWB 控制器每周期需要当前实际速度（例如限制加速度、振荡检测），
// 本类把 ROS2 订阅样板（声明参数、创建订阅、互斥锁保护）集中到一处。
class OdomSubscriber
{
public:
  /**
   * @brief Constructor that subscribes to an Odometry topic
   *
   * @param nh NodeHandle for creating subscriber
   * @param default_topic Name of the topic that will be loaded of the odom_topic param is not set.
   */
  // 构造函数：创建里程计订阅
  //   1. 声明参数 odom_topic（若未声明过），默认值为 default_topic；
  //   2. 读取实际的话题名；
  //   3. 以 SystemDefaultsQoS 创建订阅，回调绑定到 odomCallback。
  // 注意 nh 是生命周期节点（LifecycleNode）——nav2 组件的标准节点类型
  explicit OdomSubscriber(
    nav2_util::LifecycleNode::SharedPtr nh,
    std::string default_topic = "odom")
  {
    // 声明参数（若已声明则跳过，避免重复声明抛异常）——ROS2 参数学习要点
    nav2_util::declare_parameter_if_not_declared(
      nh, "odom_topic", rclcpp::ParameterValue(default_topic));

    std::string odom_topic;
    // 读取参数；读取失败时回退到默认值
    nh->get_parameter_or("odom_topic", odom_topic, default_topic);
    // 创建订阅：SystemDefaultsQoS（可靠传输+保持最近），std::bind 成员回调
    odom_sub_ =
      nh->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&OdomSubscriber::odomCallback, this, std::placeholders::_1));
  }

  // 取最新速度（无时间戳版本）：内部缓存变量的拷贝（锁在回调里，读端无锁）
  inline nav_2d_msgs::msg::Twist2D getTwist() {return odom_vel_.velocity;}
  // 取最新速度（带 header 时间戳版本）
  inline nav_2d_msgs::msg::Twist2DStamped getTwistStamped() {return odom_vel_;}

protected:
  // 订阅回调：把 nav_msgs::msg::Odometry 的 twist 字段平移到 2D 速度消息。
  // mutex 保护：回调运行在 ROS 执行器的独立线程，与控制器线程并发访问
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // ROS_INFO_ONCE("odom received!");
    std::lock_guard<std::mutex> lock(odom_mutex_);  // 加锁保证写入原子性
    odom_vel_.header = msg->header;                  // 保留时间戳与坐标系
    odom_vel_.velocity.x = msg->twist.twist.linear.x;    // x 线速度
    odom_vel_.velocity.y = msg->twist.twist.linear.y;    // y 线速度
    odom_vel_.velocity.theta = msg->twist.twist.angular.z;  // 绕 z 角速度
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;  // 订阅句柄
  nav_2d_msgs::msg::Twist2DStamped odom_vel_;  // 最新缓存速度（含 header）
  std::mutex odom_mutex_;                       // 保护 odom_vel_ 的互斥锁
};

}  // namespace nav_2d_utils

#endif  // NAV_2D_UTILS__ODOM_SUBSCRIBER_HPP_
