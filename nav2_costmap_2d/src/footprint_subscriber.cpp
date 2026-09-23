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
// ============================================================================
// 中文注释：足迹话题订阅器（FootprintSubscriber）实现文件
//
// 【文件功能】
// 本文件实现对外部发布的机器人足迹（geometry_msgs/PolygonStamped）
// 的订阅与查询。与 Costmap2DROS 在参数中静态配置足迹不同，
// 本组件支持"动态足迹"：其他节点（如负载变化的检测节点）发布
// 实时足迹话题，本订阅器缓存最新消息供查询。
//
// 【在 Nav2 中的位置】
// 被 CollisionMonitor（碰撞监控器）等需要动态足迹的组件使用。
//
// 【线程安全设计】
// 回调与查询可能在不同线程：使用 std::atomic_load/atomic_store
// 操作 shared_ptr（免锁的原子指针交换），读取方总是拿到完整的
// 旧消息或新消息，不会读到半更新状态。
//
// 【坐标变换】
// getFootprintInRobotFrame() 把发布坐标系下的足迹变换到机器人
// 基坐标系（robot_base_frame）：先平移 (-x,-y) 再旋转 -theta，
// 等价于施加 TF 逆变换。
// ============================================================================

#include <string>
#include <vector>
#include <memory>

#include "nav2_costmap_2d/footprint_subscriber.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace nav2_costmap_2d
{

// ============================================================================
// 构造函数（生命周期节点版）：创建足迹话题订阅
// ============================================================================

FootprintSubscriber::FootprintSubscriber(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  const std::string & topic_name,
  tf2_ros::Buffer & tf,
  std::string robot_base_frame,
  double transform_tolerance)
: tf_(tf),
  robot_base_frame_(robot_base_frame),
  transform_tolerance_(transform_tolerance)
{
  auto node = parent.lock();
  // 订阅 PolygonStamped 足迹消息，回调为 footprint_callback
  footprint_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    topic_name, rclcpp::SystemDefaultsQoS(),
    std::bind(&FootprintSubscriber::footprint_callback, this, std::placeholders::_1));
}

// ============================================================================
// 构造函数（普通节点版）：与上相同，支持非生命周期节点
// ============================================================================

FootprintSubscriber::FootprintSubscriber(
  const rclcpp::Node::WeakPtr & parent,
  const std::string & topic_name,
  tf2_ros::Buffer & tf,
  std::string robot_base_frame,
  double transform_tolerance)
: tf_(tf),
  robot_base_frame_(robot_base_frame),
  transform_tolerance_(transform_tolerance)
{
  auto node = parent.lock();
  footprint_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    topic_name, rclcpp::SystemDefaultsQoS(),
    std::bind(&FootprintSubscriber::footprint_callback, this, std::placeholders::_1));
}

// ============================================================================
// getFootprintRaw()：获取最近收到的原始足迹（发布坐标系下）
// 尚未收到任何足迹时返回 false
// ============================================================================

bool
FootprintSubscriber::getFootprintRaw(
  std::vector<geometry_msgs::msg::Point> & footprint,
  std_msgs::msg::Header & footprint_header)
{
  if (!footprint_received_) {
    return false;   // 从未收到过足迹
  }

  // 原子读取最新消息（免锁线程安全）
  auto current_footprint = std::atomic_load(&footprint_);
  // Polygon → Point 向量转换
  footprint = toPointVector(
    std::make_shared<geometry_msgs::msg::Polygon>(current_footprint->polygon));
  footprint_header = current_footprint->header;

  return true;
}

// ============================================================================
// getFootprintInRobotFrame()：获取变换到机器人基坐标系的足迹
// 用 TF 查询发布坐标系→机器人系的位姿，再做逆变换
// ============================================================================

bool
FootprintSubscriber::getFootprintInRobotFrame(
  std::vector<geometry_msgs::msg::Point> & footprint,
  std_msgs::msg::Header & footprint_header)
{
  // 先取发布坐标系下的原始足迹
  if (!getFootprintRaw(footprint, footprint_header)) {
    return false;
  }

  // 查询机器人在足迹发布坐标系中的位姿
  // （getCurrentPose 返回 footprint_frame → robot_base_frame 的变换）
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, tf_, footprint_header.frame_id, robot_base_frame_,
      transform_tolerance_, footprint_header.stamp))
  {
    return false;
  }

  double x = current_pose.pose.position.x;
  double y = current_pose.pose.position.y;
  double theta = tf2::getYaw(current_pose.pose.orientation);

  // 逆变换：先平移 (-x,-y)，再旋转 (-theta)
  // 等价于把足迹从发布坐标系变换到机器人基坐标系
  std::vector<geometry_msgs::msg::Point> temp;
  transformFootprint(-x, -y, 0, footprint, temp);
  transformFootprint(0, 0, -theta, temp, footprint);

  // 更新消息头为机器人基坐标系
  footprint_header.frame_id = robot_base_frame_;
  footprint_header.stamp = current_pose.header.stamp;

  return true;
}

// ============================================================================
// footprint_callback()：订阅回调——原子保存最新消息并置"已收到"标志
// ============================================================================

void
FootprintSubscriber::footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  // 原子交换 shared_ptr：读取方不会看到半更新状态
  std::atomic_store(&footprint_, msg);
  if (!footprint_received_) {
    footprint_received_ = true;
  }
}

}  // namespace nav2_costmap_2d
