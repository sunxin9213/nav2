// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_UTIL__ODOMETRY_UTILS_HPP_
#define NAV2_UTIL__ODOMETRY_UTILS_HPP_

#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <deque>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_util
{

/**
 * @class OdomSmoother
 * Wrapper for getting smooth odometry readings using a simple moving avergae.
 * Subscribes to the topic with a mutex.
 * 里程计平滑器：通过滑动时间窗口内的简单移动平均(SMA)输出平滑速度。
 * 内部以互斥锁保护订阅回调与用户查询之间的数据竞争。
 */
class OdomSmoother
{
public:
  /**
   * @brief Constructor that subscribes to an Odometry topic
   * @param parent NodeHandle for creating subscriber
   * @param filter_duration Duration for odom history (seconds)
   * @param odom_topic Topic on which odometry should be received
   */
  explicit OdomSmoother(
    const rclcpp::Node::WeakPtr & parent,
    double filter_duration = 0.3,
    const std::string & odom_topic = "odom");

  /**
   * @brief Overloadded Constructor for nav_util::LifecycleNode parent
   * that subscribes to an Odometry topic
   * @param parent NodeHandle for creating subscriber
   * @param filter_duration Duration for odom history (seconds)
   * @param odom_topic Topic on which odometry should be received
   */
  explicit OdomSmoother(
    const nav2_util::LifecycleNode::WeakPtr & parent,
    double filter_duration = 0.3,
    const std::string & odom_topic = "odom");

  /**
   * @brief Get twist msg from smoother
   * @return twist Twist msg
   */
  inline geometry_msgs::msg::Twist getTwist() {return vel_smooth_.twist;}

  /**
   * @brief Get twist stamped msg from smoother
   * @return twist TwistStamped msg
   */
  inline geometry_msgs::msg::TwistStamped getTwistStamped() {return vel_smooth_;}

protected:
  /**
   * @brief Callback of odometry subscriber to process
   * @param msg Odometry msg to smooth
   */
  // 里程计订阅回调：维护时间窗口并更新平滑结果
  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Update internal state of the smoother after getting new data
   */
  // 新数据入队后重算累加和与平均速度
  void updateState();

  // 里程计订阅句柄
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // 窗口内速度的累加和（不含 header 信息，只用到 twist 部分）
  nav_msgs::msg::Odometry odom_cumulate_;
  // 最近一次计算得到的平滑速度（含时间戳/坐标系头）
  geometry_msgs::msg::TwistStamped vel_smooth_;
  // 保护上述共享状态的互斥锁（回调与查询接口跨线程）
  std::mutex odom_mutex_;

  // 滑动窗口的时间长度（秒）
  rclcpp::Duration odom_history_duration_;
  // 里程计消息的历史队列（双端队列，头出尾进）
  std::deque<nav_msgs::msg::Odometry> odom_history_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__ODOMETRY_UTILS_HPP_
