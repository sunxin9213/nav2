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

// =============================================================================
// 【文件说明】is_stuck_condition.cpp
// 【所属模块】nav2_behavior_tree/plugins/condition —— Nav2 行为树的条件节点插件
//
// 实现 IsStuck 行为树节点：通过分析里程计（odometry）数据检测机器人
// 是否"卡住"（stuck）。当前实现检测"异常急减速"：若加速度低于刹车
// 极限（brake_accel_limit_，默认 -10 m/s^2），说明机器人可能撞上了
// 未感知到的障碍物——正常刹车不会这么急。
//
// 【在 Nav2 导航流程中的位置】
// 用于脱困：IsStuck 条件为真时触发恢复分支（后退、旋转等）。
// 注意：该节点默认导航树未启用，更多作为示例/扩展模板。
//
// 【BehaviorTree.CPP 与 ROS2 API 要点】
// - 条件节点：卡住 -> SUCCESS；正常 -> FAILURE。
// - 本节点演示了 BT 节点中的"后台线程"模式：
//   订阅 "odom" 话题的回调挂专属回调组，并在独立线程中 spin 执行器
//   （与 tick 线程分离，里程计数据实时更新，tick 只读结果标志）。
// - 加速度由最近两条里程计的线速度差分近似计算（历史队列长度 10）。
// - XML 使用示例：
//   <IsStuck/>
// =============================================================================

#include <string>
#include <chrono>

#include "nav2_behavior_tree/plugins/condition/is_stuck_condition.hpp"

using namespace std::chrono_literals; // NOLINT

namespace nav2_behavior_tree
{

// 构造函数：创建里程计订阅和后台 spin 线程。
IsStuckCondition::IsStuckCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  is_stuck_(false),
  odom_history_size_(10),
  current_accel_(0.0),
  brake_accel_limit_(-10.0)
{
  // 从黑板取共享 ROS2 节点。
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  // 创建专属回调组（false = 不加入默认执行器）。
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  // 在独立线程中持续 spin 执行器：里程计回调实时触发，
  // 与行为树的 tick 循环并行——本节点是少数使用后台线程的 BT 节点。
  callback_group_executor_thread = std::thread([this]() {callback_group_executor_.spin();});

  // 订阅里程计话题 "odom"（SystemDefaultsQoS），回调挂专属回调组。
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsStuckCondition::onOdomReceived, this, std::placeholders::_1),
    sub_option);

  RCLCPP_DEBUG(node_->get_logger(), "Initialized an IsStuckCondition BT node");

  RCLCPP_INFO_ONCE(node_->get_logger(), "Waiting on odometry");
}

// 析构函数：优雅关闭后台线程——先 cancel 执行器再 join 线程，
// 防止线程在节点销毁后仍访问成员。
IsStuckCondition::~IsStuckCondition()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsStuckCondition BT node");
  callback_group_executor_.cancel();
  callback_group_executor_thread.join();
}

// 里程计回调（后台线程中执行）：维护滑动窗口并更新卡住判定。
void IsStuckCondition::onOdomReceived(const typename nav_msgs::msg::Odometry::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(node_->get_logger(), "Got odometry");

  // 维护固定长度的里程计历史队列（deque 滑动窗口）：
  // 超长时从头部弹出最旧的数据。
  while (odom_history_.size() >= odom_history_size_) {
    odom_history_.pop_front();
  }

  // 新里程计入队。
  odom_history_.push_back(*msg);

  // 基于最新历史更新加速度估计和卡住标志。
  updateStates();
}

// tick 主函数：只读后台线程维护的 is_stuck_ 标志（线程安全读取点）。
BT::NodeStatus IsStuckCondition::tick()
{
  // TODO(orduno) #383 Once check for is stuck and state calculations are moved to robot class
  //              this becomes
  // if (robot_state_.isStuck()) {

  // 卡住：返回 SUCCESS（条件成立，触发上游的恢复分支）。
  if (is_stuck_) {
    logStuck("Robot got stuck!");
    return BT::NodeStatus::SUCCESS;  // Successfully detected a stuck condition
  }

  // 正常：返回 FAILURE。
  logStuck("Robot is free");
  return BT::NodeStatus::FAILURE;  // Failed to detected a stuck condition
}

// 日志去重：状态没变就不重复打印（避免每个 tick 刷屏）。
void IsStuckCondition::logStuck(const std::string & msg) const
{
  static std::string prev_msg;

  if (msg == prev_msg) {
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
  prev_msg = msg;
}

// 由里程计回调调用：更新加速度估计与卡住判定。
void IsStuckCondition::updateStates()
{
  // Approximate acceleration
  // TODO(orduno) #400 Smooth out velocity history for better accel approx.
  // 用最近两条里程计做差分近似加速度：
  // accel = (v_curr - v_prev) / (t_curr - t_prev)。
  if (odom_history_.size() > 2) {
    // 最新一条里程计及其时间戳（sec + nanosec 拼成秒）。
    auto curr_odom = odom_history_.end()[-1];
    double curr_time = static_cast<double>(curr_odom.header.stamp.sec);
    curr_time += (static_cast<double>(curr_odom.header.stamp.nanosec)) * 1e-9;

    // 倒数第二条及其时间戳。
    auto prev_odom = odom_history_.end()[-2];
    double prev_time = static_cast<double>(prev_odom.header.stamp.sec);
    prev_time += (static_cast<double>(prev_odom.header.stamp.nanosec)) * 1e-9;

    // 差分计算 x 方向线加速度。
    double dt = curr_time - prev_time;
    double vel_diff = static_cast<double>(
      curr_odom.twist.twist.linear.x - prev_odom.twist.twist.linear.x);
    current_accel_ = vel_diff / dt;
  }

  // 更新卡住判定标志（供 tick 读取）。
  is_stuck_ = isStuck();
}

// 卡住判定：检查是否出现超过刹车极限的异常减速。
bool IsStuckCondition::isStuck()
{
  // TODO(orduno) #400 The robot getting stuck can result on different types of motion
  // depending on the state prior to getting stuck (sudden change in accel, not moving at all,
  // random oscillations, etc). For now, we only address the case where there is a sudden
  // harsh deceleration. A better approach to capture all situations would be to do a forward
  // simulation of the robot motion and compare it with the actual one.
  // （卡住有多种表现：急减速、完全不动、来回振荡等；当前只处理
  //   "急减速"情形，更完善的方案是前向仿真对比。）

  // Detect if robot bumped into something by checking for abnormal deceleration
  // 加速度低于刹车极限（比正常刹车更急的减速）：
  // 很可能撞上了未感知的障碍物，判定为卡住。
  if (current_accel_ < brake_accel_limit_) {
    RCLCPP_DEBUG(
      node_->get_logger(), "Current deceleration is beyond brake limit."
      " brake limit: %.2f, current accel: %.2f", brake_accel_limit_, current_accel_);

    return true;
  }

  return false;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "IsStuck" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsStuckCondition>("IsStuck");
}
