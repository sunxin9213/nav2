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

// ============================================================================
// 【文件概况】OdomSmoother 的实现
// ----------------------------------------------------------------------------
// 概述: 见 odometry_utils.hpp。本文件实现:
//   1. 两个构造函数: 订阅 odom 话题并把速度累加器清零
//   2. odomCallback(): 加锁 → 淘汰超出窗口时长的旧消息
//      （同步从累加器中减去其速度）→ 新消息入队
//   3. updateState(): 累加最新速度，SMA 均值写入 vel_smooth_
// 数据结构技巧: "累加器 + 双端队列"使均值维护为 O(1)，
//               无需每次重新遍历窗口求和。
// ============================================================================

#include <string>

#include "nav2_util/odometry_utils.hpp"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

namespace nav2_util
{
// 普通节点版本构造函数
OdomSmoother::OdomSmoother(
  const rclcpp::Node::WeakPtr & parent,
  double filter_duration,
  const std::string & odom_topic)
: odom_history_duration_(rclcpp::Duration::from_seconds(filter_duration))
{
  // WeakPtr::lock() 取回共享指针：若父节点已销毁则得到空指针
  auto node = parent.lock();
  // 创建里程计订阅，回调绑定到成员函数 odomCallback
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&OdomSmoother::odomCallback, this, std::placeholders::_1));

  // 累加器初值清零：odom_cumulate_ 始终保存窗口内所有消息的速度之和，
  // 配合窗口大小即可 O(1) 求得滑动平均
  odom_cumulate_.twist.twist.linear.x = 0;
  odom_cumulate_.twist.twist.linear.y = 0;
  odom_cumulate_.twist.twist.linear.z = 0;
  odom_cumulate_.twist.twist.angular.x = 0;
  odom_cumulate_.twist.twist.angular.y = 0;
  odom_cumulate_.twist.twist.angular.z = 0;
}

// 生命周期节点版本构造函数：逻辑与上面完全一致，仅节点类型不同
OdomSmoother::OdomSmoother(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  double filter_duration,
  const std::string & odom_topic)
: odom_history_duration_(rclcpp::Duration::from_seconds(filter_duration))
{
  auto node = parent.lock();
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&OdomSmoother::odomCallback, this, std::placeholders::_1));

  odom_cumulate_.twist.twist.linear.x = 0;
  odom_cumulate_.twist.twist.linear.y = 0;
  odom_cumulate_.twist.twist.linear.z = 0;
  odom_cumulate_.twist.twist.angular.x = 0;
  odom_cumulate_.twist.twist.angular.y = 0;
  odom_cumulate_.twist.twist.angular.z = 0;
}

void OdomSmoother::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // 加锁保护共享状态（回调线程与用户查询线程并发访问）
  std::lock_guard<std::mutex> lock(odom_mutex_);

  // update cumulated odom only if history is not empty
  if (!odom_history_.empty()) {
    // to store current time
    // 当前消息时间戳
    auto current_time = rclcpp::Time(msg->header.stamp);

    // to store time of the first odom in history
    // 窗口中最早一条消息的时间戳
    auto front_time = rclcpp::Time(odom_history_.front().header.stamp);

    // update cumulated odom when duration has exceeded and pop earliest msg
    // 滑动窗口淘汰：把所有"距当前消息超过窗口时长"的旧消息弹出，
    // 每弹一条就从累加器中减去它的速度，保持累加器与窗口内容一致
    while (current_time - front_time > odom_history_duration_) {
      const auto & odom = odom_history_.front();
      odom_cumulate_.twist.twist.linear.x -= odom.twist.twist.linear.x;
      odom_cumulate_.twist.twist.linear.y -= odom.twist.twist.linear.y;
      odom_cumulate_.twist.twist.linear.z -= odom.twist.twist.linear.z;
      odom_cumulate_.twist.twist.angular.x -= odom.twist.twist.angular.x;
      odom_cumulate_.twist.twist.angular.y -= odom.twist.twist.angular.y;
      odom_cumulate_.twist.twist.angular.z -= odom.twist.twist.angular.z;
      odom_history_.pop_front();

      // 防止 while 条件在空队列上解引用 front()
      if (odom_history_.empty()) {
        break;
      }

      // update with the timestamp of earliest odom message in history
      // 刷新"最早消息时间戳"以继续判断下一条
      front_time = rclcpp::Time(odom_history_.front().header.stamp);
    }
  }

  // 新消息入队并重算平滑速度
  odom_history_.push_back(*msg);
  updateState();
}

void OdomSmoother::updateState()
{
  // 把最新一条消息的速度累加进窗口总和
  const auto & odom = odom_history_.back();
  odom_cumulate_.twist.twist.linear.x += odom.twist.twist.linear.x;
  odom_cumulate_.twist.twist.linear.y += odom.twist.twist.linear.y;
  odom_cumulate_.twist.twist.linear.z += odom.twist.twist.linear.z;
  odom_cumulate_.twist.twist.angular.x += odom.twist.twist.angular.x;
  odom_cumulate_.twist.twist.angular.y += odom.twist.twist.angular.y;
  odom_cumulate_.twist.twist.angular.z += odom.twist.twist.angular.z;

  // 平滑速度 = 窗口速度总和 / 窗口消息数（简单滑动平均）
  vel_smooth_.header = odom.header;
  vel_smooth_.twist.linear.x = odom_cumulate_.twist.twist.linear.x / odom_history_.size();
  vel_smooth_.twist.linear.y = odom_cumulate_.twist.twist.linear.y / odom_history_.size();
  vel_smooth_.twist.linear.z = odom_cumulate_.twist.twist.linear.z / odom_history_.size();
  vel_smooth_.twist.angular.x = odom_cumulate_.twist.twist.angular.x / odom_history_.size();
  vel_smooth_.twist.angular.y = odom_cumulate_.twist.twist.angular.y / odom_history_.size();
  vel_smooth_.twist.angular.z = odom_cumulate_.twist.twist.angular.z / odom_history_.size();
}

}  // namespace nav2_util
