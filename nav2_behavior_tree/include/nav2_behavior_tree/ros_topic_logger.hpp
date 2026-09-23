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

// =============================================================================
// 【文件说明】ros_topic_logger.hpp
// 【所属模块】nav2_behavior_tree —— Nav2 的行为树核心模块
//
// 本文件定义了 RosTopicLogger 类——把行为树的"状态变迁日志"发布到
// ROS2 话题的观察器（BT::StatusChangeLogger）。
//
// 【在 Nav2 导航系统中的位置和作用】
// BehaviorTree.CPP 提供"日志观察器"机制：树里任意节点状态发生改变
// （如 IDLE->RUNNING、RUNNING->SUCCESS）时，会回调已注册 logger 的
// callback()。本类把这些状态变迁事件缓存下来，在 flush() 时打包成
// nav2_msgs::msg::BehaviorTreeLog 消息发布到 "behavior_tree_log" 话题。
// 外部工具（如 rviz 插件、用户脚本）可订阅该话题回放/调试树执行过程。
//
// 【BehaviorTree.CPP 核心概念】
// - BT::StatusChangeLogger：框架的日志基类，构造时挂到根节点上，
//   之后整棵树的状态变化都会流经 callback()。
// - callback() 逐事件记录：时间戳、节点名、前状态、当前状态。
// - flush() 由框架在每轮 tick 结束时调用：把缓存的事件批量发布。
// - 与 Groot 的区别：Groot 通过 ZMQ 协议实时可视化（见
//   BehaviorTreeEngine 中的 bt_zmq_publisher），本类走普通 ROS2 话题，
//   适合程序化消费。
// =============================================================================

#ifndef NAV2_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_
#define NAV2_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_

#include <vector>
#include <memory>
#include <utility>

#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "nav2_msgs/msg/behavior_tree_status_change.h"
#include "tf2_ros/buffer_interface.h"

namespace nav2_behavior_tree
{

/**
 * @brief A class to publish BT logs on BT status change
 *        （在行为树状态变化时记录日志并发布到 ROS2 话题的观察器）
 */
class RosTopicLogger : public BT::StatusChangeLogger
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RosTopicLogger
   *        （构造函数：挂载到树根节点并创建日志发布器）
   * @param ros_node Weak pointer to parent rclcpp::Node
   *        （父 ROS2 节点的弱引用，用于取时钟/日志器/创建发布器）
   * @param tree BT to monitor
   *        （要监控的行为树）
   */
  RosTopicLogger(const rclcpp::Node::WeakPtr & ros_node, const BT::Tree & tree)
  : StatusChangeLogger(tree.rootNode())
  {
    // 从弱引用提升出共享的节点指针。
    auto node = ros_node.lock();
    // 取父节点的时钟与日志器（与父节点共用输出配置）。
    clock_ = node->get_clock();
    logger_ = node->get_logger();
    // 创建日志发布器：话题 "behavior_tree_log"，QoS 深度 10
    // （ROS2 QoS：深度表示发布端最多缓存多少条未确认消息）。
    log_pub_ = node->create_publisher<nav2_msgs::msg::BehaviorTreeLog>(
      "behavior_tree_log",
      rclcpp::QoS(10));
  }

  /**
   * @brief Callback function which is called each time BT changes status
   *        （状态变化回调：框架在任意节点状态改变时自动调用）
   * @param timestamp Timestamp of BT status change
   *        （状态变化的时间戳，自纪元起的时长）
   * @param node Node that changed status
   *        （发生状态变化的节点）
   * @param prev_status Previous status of the node
   *        （变化前的状态）
   * @param status Current status of the node
   *        （变化后的状态）
   */
  void callback(
    BT::Duration timestamp,
    const BT::TreeNode & node,
    BT::NodeStatus prev_status,
    BT::NodeStatus status) override
  {
    nav2_msgs::msg::BehaviorTreeStatusChange event;

    // BT timestamps are a duration since the epoch. Need to convert to a time_point
    // before converting to a msg.
    // BT 的时间戳是"自纪元起的时长"，需先转成 time_point 再转成 ROS 消息
    // 里的 builtin_interfaces::Time（tf2_ros::toMsg 完成该转换）。
    event.timestamp = tf2_ros::toMsg(tf2::TimePoint(timestamp));
    event.node_name = node.name();
    // 用框架自带的 toStr 把状态枚举转成字符串（false = 首字母小写形式）。
    event.previous_status = toStr(prev_status, false);
    event.current_status = toStr(status, false);
    // 事件入缓存队列，等 flush() 批量发布。
    event_log_.push_back(std::move(event));

    // 同时打一条 DEBUG 日志：[时间] 节点名 旧状态 -> 新状态
    // （true = 大写缩写形式，如 "RUN"）。
    RCLCPP_DEBUG(
      logger_, "[%.3f]: %25s %s -> %s",
      std::chrono::duration<double>(timestamp).count(),
      node.name().c_str(),
      toStr(prev_status, true).c_str(),
      toStr(status, true).c_str() );
  }

  /**
   * @brief Clear log buffer if any
   *        （刷新：把缓存的所有状态变迁事件打包发布到话题并清空缓存）
   */
  void flush() override
  {
    if (!event_log_.empty()) {
      // 组装 BehaviorTreeLog 消息：时间戳 + 事件列表。
      auto log_msg = std::make_unique<nav2_msgs::msg::BehaviorTreeLog>();
      log_msg->timestamp = clock_->now();
      log_msg->event_log = event_log_;
      // 发布并清空缓存（std::move 把消息所有权移交给发布调用）。
      log_pub_->publish(std::move(log_msg));
      event_log_.clear();
    }
  }

protected:
  // ---- 成员变量说明 ----
  // 共享时钟：为日志消息打时间戳。
  rclcpp::Clock::SharedPtr clock_;
  // 日志器：输出 DEBUG 级状态变迁信息。
  rclcpp::Logger logger_{rclcpp::get_logger("bt_navigator")};
  // 日志话题发布器（"behavior_tree_log"）。
  rclcpp::Publisher<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr log_pub_;
  // 状态变迁事件缓存：flush() 时批量发布。
  std::vector<nav2_msgs::msg::BehaviorTreeStatusChange> event_log_;
};

}   // namespace nav2_behavior_tree

#endif   // NAV2_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_
