/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2019, Samsung Research America, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
// ============================================================================
// 中文注释：代价地图发布器（Costmap2DPublisher）实现文件
//
// 【文件功能】
// 本文件实现主代价地图的可视化/共享发布器。它把 Costmap2D 的内部
// unsigned char 代价数组发布为 ROS 消息，提供三种输出通道：
//   1. <topic>（nav_msgs/OccupancyGrid）：翻译成标准占用栅格 [-1,100]，
//      供 RViz 显示（Costmap 显示插件）；
//   2. <topic>_raw（nav2_msgs/Costmap）：原始 0~255 代价，
//      供其他 Nav2 组件（如 CollisionMonitor）精确使用；
//   3. <topic>_updates（map_msgs/OccupancyGridUpdate）：增量更新——
//      只发布自上次以来变化过的矩形区域，大幅节省带宽。
// 另外还提供 get_costmap 服务（nav2_msgs/GetCostmap）按需取整图。
//
// 【在 Nav2 中的位置】
// 由 Costmap2DROS 持有，每次地图更新周期结束后调用 publishCostmap()。
//
// 【核心机制】
// - 代价翻译表 cost_translation_table_（静态、全进程共享）：
//   0→0（自由）、1~252 线性映射到 1~98、253→99（内切）、
//   254→100（致命）、255→-1（未知）；
// - 增量更新维护：外部通过 updateBounds() 标记脏矩形 [x0_,y0_)-[xn_,yn_)，
//   发布后复位；当地图几何参数（尺寸/原点/分辨率）变化或配置了
//   always_send_full_costmap 时退回全图发布；
// - QoS：transient_local + reliable——晚启动的 RViz 也能立即收到地图。
// ============================================================================
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"

#include <string>
#include <memory>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_costmap_2d
{

// 静态代价翻译表：costmap 值(0~255) → OccupancyGrid 值(-1~100)
// 静态成员：同一进程内多个发布器实例共享，只初始化一次
char * Costmap2DPublisher::cost_translation_table_ = NULL;

// ============================================================================
// 构造函数：创建三个发布器 + 一个服务，初始化翻译表与增量边界
// ============================================================================
Costmap2DPublisher::Costmap2DPublisher(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  Costmap2D * costmap,
  std::string global_frame,
  std::string topic_name,
  bool always_send_full_costmap)
: costmap_(costmap),
  global_frame_(global_frame),
  topic_name_(topic_name),
  active_(false),
  always_send_full_costmap_(always_send_full_costmap)
{
  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  // transient_local QoS：新订阅者（如刚打开的 RViz）也能收到最后一条消息
  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  // TODO(bpwilcox): port onNewSubscription functionality for publisher
  // 1) 标准占用栅格发布器（翻译后的 -1~100 值）
  costmap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name,
    custom_qos);
  // 2) 原始代价发布器（0~255，无损）
  costmap_raw_pub_ = node->create_publisher<nav2_msgs::msg::Costmap>(
    topic_name + "_raw",
    custom_qos);
  // 3) 增量更新发布器（只发变化的矩形区域）
  costmap_update_pub_ = node->create_publisher<map_msgs::msg::OccupancyGridUpdate>(
    topic_name + "_updates", custom_qos);

  // Create a service that will use the callback function to handle requests.
  // 4) get_costmap 服务：按需返回整张地图
  costmap_service_ = node->create_service<nav2_msgs::srv::GetCostmap>(
    "get_costmap", std::bind(
      &Costmap2DPublisher::costmap_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  // ---- 初始化代价翻译表（仅一次）----
  if (cost_translation_table_ == NULL) {
    cost_translation_table_ = new char[256];

    // special values:
    // 特殊语义值的映射
    cost_translation_table_[0] = 0;  // NO obstacle           → 0（自由）
    cost_translation_table_[253] = 99;  // INSCRIBED obstacle  → 99
    cost_translation_table_[254] = 100;  // LETHAL obstacle    → 100（占据）
    cost_translation_table_[255] = -1;  // UNKNOWN            → -1（未知）

    // regular cost values scale the range 1 to 252 (inclusive) to fit
    // into 1 to 98 (inclusive).
    // 普通代价 1~252 线性缩放到 1~98
    for (int i = 1; i < 253; i++) {
      cost_translation_table_[i] = static_cast<char>(1 + (97 * (i - 1)) / 251);
    }
  }

  // ---- 初始化增量更新矩形为"空"（x0_ >= xn_ 表示无脏区）----
  xn_ = yn_ = 0;
  x0_ = costmap_->getSizeInCellsX();
  y0_ = costmap_->getSizeInCellsY();
}

Costmap2DPublisher::~Costmap2DPublisher() {}

// TODO(bpwilcox): find equivalent/workaround to ros::SingleSubscriberPublishr
/*
void Costmap2DPublisher::onNewSubscription(const ros::SingleSubscriberPublisher& pub)
{
  prepareGrid();
  pub.publish(grid_);
} */

// ============================================================================
// prepareGrid()：把主地图打包为 OccupancyGrid 消息（含翻译表转换）
// 加锁读取几何参数与数据，逐格翻译为 -1~100
// ============================================================================
// prepare grid_ message for publication.
void Costmap2DPublisher::prepareGrid()
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  // 读取当前几何参数（供发布后判断地图是否变化）
  grid_resolution = costmap_->getResolution();
  grid_width = costmap_->getSizeInCellsX();
  grid_height = costmap_->getSizeInCellsY();

  grid_ = std::make_unique<nav_msgs::msg::OccupancyGrid>();

  // 消息头：坐标系与时间戳
  grid_->header.frame_id = global_frame_;
  grid_->header.stamp = clock_->now();

  grid_->info.resolution = grid_resolution;

  grid_->info.width = grid_width;
  grid_->info.height = grid_height;

  // 计算消息原点：格 (0,0) 中心的世界坐标减半格
  // （OccupancyGrid 的 origin 约定为左下角格左下角，而 mapToWorld
  //   给出的是格中心，故减 resolution/2）
  double wx, wy;
  costmap_->mapToWorld(0, 0, wx, wy);
  grid_->info.origin.position.x = wx - grid_resolution / 2;
  grid_->info.origin.position.y = wy - grid_resolution / 2;
  grid_->info.origin.position.z = 0.0;
  grid_->info.origin.orientation.w = 1.0;   // 无旋转
  // 记录原点：供发布后检测滚动窗口平移
  saved_origin_x_ = costmap_->getOriginX();
  saved_origin_y_ = costmap_->getOriginY();

  // 分配数据数组并逐格翻译
  grid_->data.resize(grid_->info.width * grid_->info.height);

  unsigned char * data = costmap_->getCharMap();
  for (unsigned int i = 0; i < grid_->data.size(); i++) {
    grid_->data[i] = cost_translation_table_[data[i]];
  }
}

// ============================================================================
// prepareCostmap()：把主地图打包为 nav2_msgs/Costmap 原始消息
// 与 prepareGrid 类似，但数据不做翻译（保留 0~255 原始代价）
// ============================================================================
void Costmap2DPublisher::prepareCostmap()
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  double resolution = costmap_->getResolution();

  costmap_raw_ = std::make_unique<nav2_msgs::msg::Costmap>();

  costmap_raw_->header.frame_id = global_frame_;
  costmap_raw_->header.stamp = clock_->now();

  costmap_raw_->metadata.layer = "master";   // 主地图层标识
  costmap_raw_->metadata.resolution = resolution;

  costmap_raw_->metadata.size_x = costmap_->getSizeInCellsX();
  costmap_raw_->metadata.size_y = costmap_->getSizeInCellsY();

  // 原点换算（同 prepareGrid 的半格修正）
  double wx, wy;
  costmap_->mapToWorld(0, 0, wx, wy);
  costmap_raw_->metadata.origin.position.x = wx - resolution / 2;
  costmap_raw_->metadata.origin.position.y = wy - resolution / 2;
  costmap_raw_->metadata.origin.position.z = 0.0;
  costmap_raw_->metadata.origin.orientation.w = 1.0;

  // 直接拷贝原始代价（无损）
  costmap_raw_->data.resize(costmap_raw_->metadata.size_x * costmap_raw_->metadata.size_y);

  unsigned char * data = costmap_->getCharMap();
  for (unsigned int i = 0; i < costmap_raw_->data.size(); i++) {
    costmap_raw_->data[i] = data[i];
  }
}

// ============================================================================
// publishCostmap()：发布入口（每个更新周期调用一次）
// 策略：
//   1) _raw 话题：有订阅者就全图发布（原始代价）；
//   2) 主话题：有订阅者且（强制全图 或 地图几何变化）→ 全图发布；
//      否则若有脏矩形（x0_<xn_）→ 只发增量更新；
//   3) 发布后复位脏矩形。
// ============================================================================
void Costmap2DPublisher::publishCostmap()
{
  // 原始代价话题：有订阅者时每次都发全图
  if (costmap_raw_pub_->get_subscription_count() > 0) {
    prepareCostmap();
    costmap_raw_pub_->publish(std::move(costmap_raw_));
  }
  float resolution = costmap_->getResolution();

  // 判断是否需要全图发布：
  // 强制全图 / 分辨率变化 / 尺寸变化 / 原点变化（滚动窗口平移）
  if (always_send_full_costmap_ || grid_resolution != resolution ||
    grid_width != costmap_->getSizeInCellsX() ||
    grid_height != costmap_->getSizeInCellsY() ||
    saved_origin_x_ != costmap_->getOriginX() ||
    saved_origin_y_ != costmap_->getOriginY())
  {
    if (costmap_pub_->get_subscription_count() > 0) {
      prepareGrid();
      costmap_pub_->publish(std::move(grid_));
    }
  } else if (x0_ < xn_) {
    // 存在脏矩形：发布增量更新消息（只含变化区域）
    if (costmap_update_pub_->get_subscription_count() > 0) {
      std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
      // Publish Just an Update
      auto update = std::make_unique<map_msgs::msg::OccupancyGridUpdate>();
      update->header.stamp = rclcpp::Time();
      update->header.frame_id = global_frame_;
      // 脏矩形的位置与尺寸
      update->x = x0_;
      update->y = y0_;
      update->width = xn_ - x0_;
      update->height = yn_ - y0_;
      update->data.resize(update->width * update->height);
      // 逐格翻译脏区内的代价
      unsigned int i = 0;
      for (unsigned int y = y0_; y < yn_; y++) {
        for (unsigned int x = x0_; x < xn_; x++) {
          unsigned char cost = costmap_->getCost(x, y);
          update->data[i++] = cost_translation_table_[cost];
        }
      }
      costmap_update_pub_->publish(std::move(update));
    }
  }

  // 复位脏矩形为空（下次重新累计）
  xn_ = yn_ = 0;
  x0_ = costmap_->getSizeInCellsX();
  y0_ = costmap_->getSizeInCellsY();
}

// ============================================================================
// costmap_service_callback()：get_costmap 服务回调
// 按需返回整张主地图（原始 0~255 代价，不做翻译）
// 服务类型：nav2_msgs/srv/GetCostmap
// ============================================================================
void
Costmap2DPublisher::costmap_service_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request>/*request*/,
  const std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> response)
{
  RCLCPP_DEBUG(logger_, "Received costmap service request");

  // TODO(bpwilcox): Grab correct orientation information
  // 原点姿态：单位四元数（无旋转）
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, 0.0);

  // 读取地图尺寸与数据指针
  auto size_x = costmap_->getSizeInCellsX();
  auto size_y = costmap_->getSizeInCellsY();
  auto data_length = size_x * size_y;
  unsigned char * data = costmap_->getCharMap();
  auto current_time = clock_->now();

  // 填充响应：消息头与元数据（尺寸/分辨率/原点/时间戳）
  response->map.header.stamp = current_time;
  response->map.header.frame_id = global_frame_;
  response->map.metadata.size_x = size_x;
  response->map.metadata.size_y = size_y;
  response->map.metadata.resolution = costmap_->getResolution();
  response->map.metadata.layer = "master";
  response->map.metadata.map_load_time = current_time;
  response->map.metadata.update_time = current_time;
  response->map.metadata.origin.position.x = costmap_->getOriginX();
  response->map.metadata.origin.position.y = costmap_->getOriginY();
  response->map.metadata.origin.position.z = 0.0;
  response->map.metadata.origin.orientation = tf2::toMsg(quaternion);
  // 整块拷贝原始代价数据
  response->map.data.resize(data_length);
  response->map.data.assign(data, data + data_length);
}

}  // end namespace nav2_costmap_2d
