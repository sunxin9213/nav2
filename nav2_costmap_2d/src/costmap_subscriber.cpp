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
// 中文注释：代价地图话题订阅器（CostmapSubscriber）实现文件
//
// 【文件功能】
// 本文件实现对 nav2_msgs/Costmap 话题（即 Costmap2DPublisher 发布的
// <topic>_raw 原始代价地图）的订阅与查询。它把收到的消息转换为
// Costmap2D 对象供外部直接使用栅格接口（getCost/worldToMap 等）。
//
// 【在 Nav2 中的位置】
// 被 CollisionMonitor（碰撞监控器）等"消费别的节点代价地图"的组件
// 使用，配合 CostmapTopicCollisionChecker 实现基于话题的碰撞检测。
//
// 【线程安全设计】
// 回调与查询可能在不同线程：用 std::atomic_load/atomic_store
// 原子交换 shared_ptr 消息（免锁）；getCostmap() 每次把最新消息
// 转写进本地 Costmap2D。
//
// 【QoS】
// KeepLast(1) + transient_local + reliable：只保留最新一帧，
// 且晚启动的订阅者也能立即收到一次地图。
// ============================================================================

#include <string>
#include <memory>

#include "nav2_costmap_2d/costmap_subscriber.hpp"

namespace nav2_costmap_2d
{

// ============================================================================
// 构造函数（生命周期节点版）：订阅原始代价地图话题
// ============================================================================

CostmapSubscriber::CostmapSubscriber(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  const std::string & topic_name)
: topic_name_(topic_name)
{
  auto node = parent.lock();
  // 订阅 nav2_msgs/Costmap（原始 0~255 代价）
  costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    topic_name_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&CostmapSubscriber::costmapCallback, this, std::placeholders::_1));
}

// ============================================================================
// 构造函数（普通节点版）：同上，支持非生命周期节点
// ============================================================================

CostmapSubscriber::CostmapSubscriber(
  const rclcpp::Node::WeakPtr & parent,
  const std::string & topic_name)
: topic_name_(topic_name)
{
  auto node = parent.lock();
  costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    topic_name_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&CostmapSubscriber::costmapCallback, this, std::placeholders::_1));
}

// ============================================================================
// getCostmap()：获取最新的 Costmap2D 地图对象
// 尚未收到任何地图时抛异常；正常时把最新消息转写进本地地图返回
// ============================================================================

std::shared_ptr<Costmap2D> CostmapSubscriber::getCostmap()
{
  if (!costmap_received_) {
    throw std::runtime_error("Costmap is not available");
  }
  toCostmap2D();   // 消息 → Costmap2D 转写
  return costmap_;
}

// ============================================================================
// toCostmap2D()：把最新消息数据转写到本地 Costmap2D
// 首次创建地图；几何参数变化时 resize；数据按下标一一对应拷贝
// ============================================================================

void CostmapSubscriber::toCostmap2D()
{
  // 原子读取最新消息（免锁线程安全）
  auto current_costmap_msg = std::atomic_load(&costmap_msg_);

  if (costmap_ == nullptr) {
    // 首次：按消息元数据创建地图（尺寸/分辨率/原点）
    costmap_ = std::make_shared<Costmap2D>(
      current_costmap_msg->metadata.size_x, current_costmap_msg->metadata.size_y,
      current_costmap_msg->metadata.resolution, current_costmap_msg->metadata.origin.position.x,
      current_costmap_msg->metadata.origin.position.y);
  } else if (costmap_->getSizeInCellsX() != current_costmap_msg->metadata.size_x ||  // NOLINT
    costmap_->getSizeInCellsY() != current_costmap_msg->metadata.size_y ||
    costmap_->getResolution() != current_costmap_msg->metadata.resolution ||
    costmap_->getOriginX() != current_costmap_msg->metadata.origin.position.x ||
    costmap_->getOriginY() != current_costmap_msg->metadata.origin.position.y)
  {
    // Update the size of the costmap
    // 几何参数变化（地图被调整/滚动窗口平移）：resize 本地地图
    costmap_->resizeMap(
      current_costmap_msg->metadata.size_x, current_costmap_msg->metadata.size_y,
      current_costmap_msg->metadata.resolution,
      current_costmap_msg->metadata.origin.position.x,
      current_costmap_msg->metadata.origin.position.y);
  }

  // 逐格拷贝代价数据（消息与本地地图同为行优先一维布局）
  unsigned char * master_array = costmap_->getCharMap();
  unsigned int index = 0;
  for (unsigned int i = 0; i < current_costmap_msg->metadata.size_x; ++i) {
    for (unsigned int j = 0; j < current_costmap_msg->metadata.size_y; ++j) {
      master_array[index] = current_costmap_msg->data[index];
      ++index;
    }
  }
}

// ============================================================================
// costmapCallback()：订阅回调——原子保存最新消息并置"已收到"标志
// ============================================================================

void CostmapSubscriber::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  // 原子交换：读取方总是拿到完整的旧或新消息
  std::atomic_store(&costmap_msg_, msg);
  if (!costmap_received_) {
    costmap_received_ = true;
  }
}

}  // namespace nav2_costmap_2d
