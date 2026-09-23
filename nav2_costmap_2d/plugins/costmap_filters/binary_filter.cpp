/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022 Samsung R&D Institute Russia
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
 *   * Neither the name of the <ORGANIZATION> nor the names of its
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
 * Author: Alexey Merzlyakov
 *********************************************************************/
// ============================================================================
// 中文注释：二值状态过滤器（BinaryFilter）实现文件
//
// 【文件功能】
// 本文件实现 Nav2 代价地图过滤器体系中的"二值过滤器"。
// 它根据机器人当前所在的掩码区域输出一个布尔状态（true/false），
// 并发布到 binary_state 话题（std_msgs/Bool）——
// 可用于"进入某区域自动开/关某功能"的语义触发，例如：
// 进入充电区自动切换任务状态、进入货区触发装卸逻辑等。
//
// 【在 Nav2 中的位置】
// 继承链：Layer → CostmapFilter → BinaryFilter。
// 与 SpeedFilter 一样不修改主地图（process() 忽略 master_grid），
// 输出是话题消息而非地图数据。
//
// 【状态翻转模型】
//   计算值 = base_ + mask_data * multiplier_
//   - 计算值 > flip_threshold_ → 状态翻转为 !default_state_（"开"）
//   - 计算值 <= flip_threshold_ → 状态恢复 default_state_（"关"）
// 机器人离开掩码范围或掩码格为未知时：
//   - 出界 → 恢复默认状态；
//   - 未知 → 保持当前状态不变（限流警告）。
// 状态只在变化时发布（changeState 内发布）。
// ============================================================================

#include "nav2_costmap_2d/costmap_filters/binary_filter.hpp"

#include <cmath>
#include <utility>
#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/occ_grid_values.hpp"

namespace nav2_costmap_2d
{

// 构造：初始化各订阅/发布器/掩码指针为空，状态复位为默认值
BinaryFilter::BinaryFilter()
: filter_info_sub_(nullptr), mask_sub_(nullptr),
  binary_state_pub_(nullptr), filter_mask_(nullptr), mask_frame_(""), global_frame_(""),
  default_state_(false), binary_state_(default_state_)
{
}

// ============================================================================
// initializeFilter()：过滤器激活——读取特有参数、订阅 filter info、
// 创建状态发布器并初始化为默认状态
// ============================================================================
void BinaryFilter::initializeFilter(
  const std::string & filter_info_topic)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Declare parameters specific to BinaryFilter only
  // BinaryFilter 特有参数：
  std::string binary_state_topic;
  declareParameter("default_state", rclcpp::ParameterValue(false));          // 默认（初始）状态
  node->get_parameter(name_ + "." + "default_state", default_state_);
  declareParameter("binary_state_topic", rclcpp::ParameterValue("binary_state"));// 状态发布话题
  node->get_parameter(name_ + "." + "binary_state_topic", binary_state_topic);
  declareParameter("flip_threshold", rclcpp::ParameterValue(50.0));           // 状态翻转阈值
  node->get_parameter(name_ + "." + "flip_threshold", flip_threshold_);

  filter_info_topic_ = filter_info_topic;
  // Setting new costmap filter info subscriber
  // 订阅 filter info 话题（transient_local QoS）
  RCLCPP_INFO(
    logger_,
    "BinaryFilter: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());
  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&BinaryFilter::filterInfoCallback, this, std::placeholders::_1));

  // Get global frame required for binary state publisher
  // 全局坐标系
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // Create new binary state publisher
  // 创建布尔状态生命周期发布器并显式激活
  binary_state_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    binary_state_topic, rclcpp::QoS(10));
  binary_state_pub_->on_activate();

  // Reset parameters
  // 复位换算参数（等 filter info 填入）
  base_ = BASE_DEFAULT;
  multiplier_ = MULTIPLIER_DEFAULT;

  // Initialize state as "false" by-default
  // 以默认状态初始化（会立即发布一次初始状态）
  changeState(default_state_);
}

// ============================================================================
// filterInfoCallback()：CostmapFilterInfo 回调——校验类型、读取
// base/multiplier、订阅真正的掩码话题
// ============================================================================
void BinaryFilter::filterInfoCallback(
  const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!mask_sub_) {
    RCLCPP_INFO(
      logger_,
      "BinaryFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "BinaryFilter: New costmap filter info arrived from %s topic. Updating old filter info.",
      filter_info_topic_.c_str());
    // Resetting previous subscriber each time when new costmap filter information arrives
    // 新 filter info 到达：销毁旧掩码订阅
    mask_sub_.reset();
  }

  // 类型必须是 BINARY_FILTER，否则报错返回
  if (msg->type != BINARY_FILTER) {
    RCLCPP_ERROR(logger_, "BinaryFilter: Mode %i is not supported", msg->type);
    return;
  }

  // Set base_ and multiplier_
  // 读取换算参数
  base_ = msg->base;
  multiplier_ = msg->multiplier;
  // Set topic name to receive filter mask from
  mask_topic_ = msg->filter_mask_topic;

  // Setting new filter mask subscriber
  // 创建掩码地图订阅
  RCLCPP_INFO(
    logger_,
    "BinaryFilter: Subscribing to \"%s\" topic for filter mask...",
    mask_topic_.c_str());
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&BinaryFilter::maskCallback, this, std::placeholders::_1));
}

// ============================================================================
// maskCallback()：掩码回调——保存掩码消息与坐标系
// ============================================================================
void BinaryFilter::maskCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!filter_mask_) {
    RCLCPP_INFO(
      logger_,
      "BinaryFilter: Received filter mask from %s topic.", mask_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "BinaryFilter: New filter mask arrived from %s topic. Updating old filter mask.",
      mask_topic_.c_str());
    filter_mask_.reset();
  }

  filter_mask_ = msg;                   // 保存掩码消息
  mask_frame_ = msg->header.frame_id;   // 记录掩码坐标系
}

// ============================================================================
// process()：过滤器核心——按机器人位置查掩码并按阈值翻转二值状态
// 不修改主地图（master_grid 被忽略），输出是 binary_state 话题消息
// ============================================================================
void BinaryFilter::process(
  nav2_costmap_2d::Costmap2D & /*master_grid*/,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/,
  const geometry_msgs::msg::Pose2D & pose)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  // 尚未收到掩码：限流警告
  if (!filter_mask_) {
    // Show warning message every 2 seconds to not litter an output
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "BinaryFilter: Filter mask was not received");
    return;
  }

  geometry_msgs::msg::Pose2D mask_pose;  // robot coordinates in mask frame

  // Transforming robot pose from current layer frame to mask frame
  // 机器人位姿变换到掩码坐标系
  if (!transformPose(global_frame_, pose, mask_frame_, mask_pose)) {
    return;
  }

  // Converting mask_pose robot position to filter_mask_ indexes (mask_robot_i, mask_robot_j)
  // 机器人位置 → 掩码栅格坐标
  unsigned int mask_robot_i, mask_robot_j;
  if (!worldToMask(filter_mask_, mask_pose.x, mask_pose.y, mask_robot_i, mask_robot_j)) {
    // Robot went out of mask range. Set "false" state by-default
    // 机器人走出掩码范围：恢复默认状态
    RCLCPP_WARN(
      logger_,
      "BinaryFilter: Robot is outside of filter mask. Resetting binary state to default.");
    changeState(default_state_);
    return;
  }

  // Getting filter_mask data from cell where the robot placed
  // 读取机器人所在掩码格子的值
  int8_t mask_data = getMaskData(filter_mask_, mask_robot_i, mask_robot_j);
  if (mask_data == nav2_util::OCC_GRID_UNKNOWN) {
    // Corresponding filter mask cell is unknown.
    // Warn and do nothing.
    // 未知格：保持当前状态（只打警告）
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "BinaryFilter: Filter mask [%i, %i] data is unknown. Do nothing.",
      mask_robot_i, mask_robot_j);
    return;
  }
  // Check and flip binary state, if necessary
  // 阈值判断：计算值 = base + mask_data * multiplier
  // 超过阈值 → "开"（非默认态）；否则 → "关"（默认态）
  // 仅在状态实际变化时才调用 changeState（内部发布消息）
  if (base_ + mask_data * multiplier_ > flip_threshold_) {
    if (binary_state_ == default_state_) {
      changeState(!default_state_);
    }
  } else {
    if (binary_state_ != default_state_) {
      changeState(default_state_);
    }
  }
}

// ============================================================================
// resetFilter()：过滤器去激活/复位——恢复默认状态并销毁订阅/发布器
// ============================================================================
void BinaryFilter::resetFilter()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  RCLCPP_INFO(logger_, "BinaryFilter: Resetting the filter to default state");
  changeState(default_state_);

  filter_info_sub_.reset();
  mask_sub_.reset();
  if (binary_state_pub_) {
    binary_state_pub_->on_deactivate();
    binary_state_pub_.reset();
  }
}

// ============================================================================
// isActive()：过滤器是否活跃——已收到掩码即视为活跃
// ============================================================================
bool BinaryFilter::isActive()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (filter_mask_) {
    return true;
  }
  return false;
}

// ============================================================================
// changeState()：切换状态并发布 std_msgs/Bool 消息
// （所有状态变化都通过这里统一发布）
// ============================================================================
void BinaryFilter::changeState(const bool state)
{
  binary_state_ = state;
  if (state) {
    RCLCPP_INFO(logger_, "BinaryFilter: Switched on");
  } else {
    RCLCPP_INFO(logger_, "BinaryFilter: Switched off");
  }

  // Forming and publishing new BinaryState message
  // 组装并发布布尔状态消息（订阅方据此触发/关闭功能）
  std::unique_ptr<std_msgs::msg::Bool> msg =
    std::make_unique<std_msgs::msg::Bool>();
  msg->data = state;
  binary_state_pub_->publish(std::move(msg));
}

}  // namespace nav2_costmap_2d

// pluginlib 导出宏：注册 BinaryFilter 为 Layer 插件
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::BinaryFilter, nav2_costmap_2d::Layer)
