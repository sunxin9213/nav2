/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Samsung Research Russia
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
// 中文注释：限速区过滤器（SpeedFilter）实现文件
//
// 【文件功能】
// 本文件实现 Nav2 代价地图过滤器体系中的"限速区过滤器"。
// 它订阅一张"限速掩码地图"（OccupancyGrid，用户预先绘制），根据机器人
// 当前位置所在的掩码格子计算速度限制值，并把限速指令以
// nav2_msgs/SpeedLimit 消息发布到 speed_limit 话题——
// 控制器（如 MPPI/Regulated Pure Pursuit）和速度平滑器订阅该消息
// 动态调整最大速度，实现"限速区/限速走廊"功能。
//
// 【在 Nav2 中的位置】
// 继承链：Layer → CostmapFilter → SpeedFilter。
// 与 KeepoutFilter 不同：它不修改主地图（process() 忽略 master_grid），
// 而是影响"速度维度"。
//
// 【速度计算模型】
// CostmapFilterInfo 消息中的 base 与 multiplier 决定换算公式：
//   speed_limit = filter_mask_data * multiplier + base
// 其中 filter_mask_data ∈ [1..100]（掩码格子值）。两种模式：
//   - 百分比模式（type=SPEED_FILTER_PERCENT）：结果是最大速度的百分比；
//   - 绝对模式（type=SPEED_FILTER_ABSOLUTE）：结果是 m/s 绝对值。
// 掩码值 0（SPEED_MASK_NO_LIMIT）表示不限速；-1（未知）视为错误。
// 只有当限速值发生变化时才发布新消息（避免刷屏）。
// ============================================================================

#include "nav2_costmap_2d/costmap_filters/speed_filter.hpp"

#include <cmath>
#include <utility>
#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace nav2_costmap_2d
{

// 构造：初始化订阅/发布器/掩码指针为空，限速值复位为"无限速"
SpeedFilter::SpeedFilter()
: filter_info_sub_(nullptr), mask_sub_(nullptr),
  speed_limit_pub_(nullptr), filter_mask_(nullptr), mask_frame_(""), global_frame_(""),
  speed_limit_(NO_SPEED_LIMIT), speed_limit_prev_(NO_SPEED_LIMIT)
{
}

// ============================================================================
// initializeFilter()：过滤器激活——声明特有参数、订阅 filter info、
// 创建限速发布器
// ============================================================================
void SpeedFilter::initializeFilter(
  const std::string & filter_info_topic)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Declare "speed_limit_topic" parameter specific to SpeedFilter only
  // SpeedFilter 特有参数：限速消息发布话题名
  std::string speed_limit_topic;
  declareParameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));
  node->get_parameter(name_ + "." + "speed_limit_topic", speed_limit_topic);

  filter_info_topic_ = filter_info_topic;
  // Setting new costmap filter info subscriber
  // 订阅 filter info 话题（transient_local：晚启动也能收到）
  RCLCPP_INFO(
    logger_,
    "SpeedFilter: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());
  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SpeedFilter::filterInfoCallback, this, std::placeholders::_1));

  // Get global frame required for speed limit publisher
  // 全局坐标系（限速消息的 header 使用）
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // Create new speed limit publisher
  // 创建限速消息生命周期发布器并显式激活
  speed_limit_pub_ = node->create_publisher<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic, rclcpp::QoS(10));
  speed_limit_pub_->on_activate();

  // Reset speed conversion states
  // 复位换算参数（等 filter info 消息填入）
  base_ = BASE_DEFAULT;
  multiplier_ = MULTIPLIER_DEFAULT;
  percentage_ = false;
}

// ============================================================================
// filterInfoCallback()：CostmapFilterInfo 消息回调——
// 读取 base/multiplier 与限速模式，然后订阅真正的掩码话题
// ============================================================================
void SpeedFilter::filterInfoCallback(
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
      "SpeedFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "SpeedFilter: New costmap filter info arrived from %s topic. Updating old filter info.",
      filter_info_topic_.c_str());
    // Resetting previous subscriber each time when new costmap filter information arrives
    // 新 filter info 到达：销毁旧掩码订阅，稍后按新话题重建
    mask_sub_.reset();
  }

  // Set base_/multiplier_ or use speed limit in % of maximum speed
  // 读取速度换算参数 base 与 multiplier
  base_ = msg->base;
  multiplier_ = msg->multiplier;
  if (msg->type == SPEED_FILTER_PERCENT) {
    // Using speed limit in % of maximum speed
    // 百分比模式：限速值是最大速度的百分比
    percentage_ = true;
    RCLCPP_INFO(
      logger_,
      "SpeedFilter: Using expressed in a percent from maximum speed"
      "speed_limit = %f + filter_mask_data * %f",
      base_, multiplier_);
  } else if (msg->type == SPEED_FILTER_ABSOLUTE) {
    // Using speed limit in m/s
    // 绝对模式：限速值是 m/s
    percentage_ = false;
    RCLCPP_INFO(
      logger_,
      "SpeedFilter: Using absolute speed_limit = %f + filter_mask_data * %f",
      base_, multiplier_);
  } else {
    // 不支持的模式：报错返回（不订阅掩码）
    RCLCPP_ERROR(logger_, "SpeedFilter: Mode is not supported");
    return;
  }

  // 从 filter info 中取出掩码话题名
  mask_topic_ = msg->filter_mask_topic;

  // Setting new filter mask subscriber
  // 创建掩码地图订阅
  RCLCPP_INFO(
    logger_,
    "SpeedFilter: Subscribing to \"%s\" topic for filter mask...",
    mask_topic_.c_str());
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SpeedFilter::maskCallback, this, std::placeholders::_1));
}

// ============================================================================
// maskCallback()：掩码地图回调——保存掩码指针与坐标系
// （直接保存 OccupancyGrid 消息的 shared_ptr，无需转 Costmap2D）
// ============================================================================
void SpeedFilter::maskCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!filter_mask_) {
    RCLCPP_INFO(
      logger_,
      "SpeedFilter: Received filter mask from %s topic.", mask_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "SpeedFilter: New filter mask arrived from %s topic. Updating old filter mask.",
      mask_topic_.c_str());
    filter_mask_.reset();
  }

  filter_mask_ = msg;                        // 保存掩码消息
  mask_frame_ = msg->header.frame_id;        // 记录掩码坐标系
}

// ============================================================================
// process()：过滤器核心——按机器人位置查掩码计算限速并发布
// 注意：SpeedFilter 不修改主地图（master_grid 参数被忽略），
// 它的输出是发布到 speed_limit 话题的消息
// ============================================================================
void SpeedFilter::process(
  nav2_costmap_2d::Costmap2D & /*master_grid*/,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/,
  const geometry_msgs::msg::Pose2D & pose)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  // 尚未收到掩码：限流警告（每 2 秒最多一条）
  if (!filter_mask_) {
    // Show warning message every 2 seconds to not litter an output
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "SpeedFilter: Filter mask was not received");
    return;
  }

  geometry_msgs::msg::Pose2D mask_pose;  // robot coordinates in mask frame

  // Transforming robot pose from current layer frame to mask frame
  // 机器人位姿从全局坐标系变换到掩码坐标系（异坐标系时用 TF）
  if (!transformPose(global_frame_, pose, mask_frame_, mask_pose)) {
    return;
  }

  // Converting mask_pose robot position to filter_mask_ indexes (mask_robot_i, mask_robot_j)
  // 机器人位置（世界坐标）→ 掩码栅格坐标
  unsigned int mask_robot_i, mask_robot_j;
  if (!worldToMask(filter_mask_, mask_pose.x, mask_pose.y, mask_robot_i, mask_robot_j)) {
    return;
  }

  // Getting filter_mask data from cell where the robot placed and
  // calculating speed limit value
  // 读取机器人所在掩码格子的值并计算限速
  int8_t speed_mask_data = getMaskData(filter_mask_, mask_robot_i, mask_robot_j);
  if (speed_mask_data == SPEED_MASK_NO_LIMIT) {
    // Corresponding filter mask cell is free.
    // Setting no speed limit there.
    // 掩码值 0：该区域不限速
    speed_limit_ = NO_SPEED_LIMIT;
  } else if (speed_mask_data == SPEED_MASK_UNKNOWN) {
    // Corresponding filter mask cell is unknown.
    // Do nothing.
    // 掩码值 -1（未知）：限速掩码不应有未知格，报错返回
    RCLCPP_ERROR(
      logger_,
      "SpeedFilter: Found unknown cell in filter_mask[%i, %i], "
      "which is invalid for this kind of filter",
      mask_robot_i, mask_robot_j);
    return;
  } else {
    // Normal case: speed_mask_data in range of [1..100]
    // 正常情况：按线性公式换算限速值
    speed_limit_ = speed_mask_data * multiplier_ + base_;
    if (percentage_) {
      // 百分比模式：合法范围 [0,100]，越界则放弃限速
      if (speed_limit_ < 0.0 || speed_limit_ > 100.0) {
        RCLCPP_WARN(
            logger_,
            "SpeedFilter: Speed limit in filter_mask[%i, %i] is %f%%, "
            "out of bounds of [0, 100]. Setting it to no-limit value.",
            mask_robot_i, mask_robot_j, speed_limit_);
        speed_limit_ = NO_SPEED_LIMIT;
      }
    } else {
      // 绝对模式：不允许负速度
      if (speed_limit_ < 0.0) {
        RCLCPP_WARN(
            logger_,
            "SpeedFilter: Speed limit in filter_mask[%i, %i] is less than 0 m/s, "
            "which can not be true. Setting it to no-limit value.",
            mask_robot_i, mask_robot_j);
        speed_limit_ = NO_SPEED_LIMIT;
      }
    }
  }

  // 只有限速值发生变化时才发布（避免每个更新周期都发消息）
  if (speed_limit_ != speed_limit_prev_) {
    if (speed_limit_ != NO_SPEED_LIMIT) {
      RCLCPP_DEBUG(logger_, "SpeedFilter: Speed limit is set to %f", speed_limit_);
    } else {
      RCLCPP_DEBUG(logger_, "SpeedFilter: Speed limit is set to its default value");
    }

    // Forming and publishing new SpeedLimit message
    // 组装并发布 SpeedLimit 消息（控制器/速度平滑器会订阅）
    std::unique_ptr<nav2_msgs::msg::SpeedLimit> msg =
      std::make_unique<nav2_msgs::msg::SpeedLimit>();
    msg->header.frame_id = global_frame_;
    msg->header.stamp = clock_->now();
    msg->percentage = percentage_;       // true=百分比模式 false=绝对值模式
    msg->speed_limit = speed_limit_;    // 限速值（% 或 m/s）
    speed_limit_pub_->publish(std::move(msg));

    speed_limit_prev_ = speed_limit_;   // 记录已发布的值
  }
}

// ============================================================================
// resetFilter()：过滤器去激活/复位——销毁订阅并去激活发布器
// ============================================================================
void SpeedFilter::resetFilter()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  filter_info_sub_.reset();
  mask_sub_.reset();
  if (speed_limit_pub_) {
    speed_limit_pub_->on_deactivate();
    speed_limit_pub_.reset();
  }
}

// ============================================================================
// isActive()：过滤器是否活跃——已收到掩码即视为活跃
// ============================================================================
bool SpeedFilter::isActive()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (filter_mask_) {
    return true;
  }
  return false;
}

}  // namespace nav2_costmap_2d

// pluginlib 导出宏：注册 SpeedFilter 为 Layer 插件
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::SpeedFilter, nav2_costmap_2d::Layer)
