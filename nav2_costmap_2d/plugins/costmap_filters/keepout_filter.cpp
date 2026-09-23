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
 *  CAUSED AND ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Alexey Merzlyakov
 *********************************************************************/
// ============================================================================
// 中文注释：禁行区过滤器（KeepoutFilter）实现文件
//
// 【文件功能】
// 本文件实现 Nav2 代价地图过滤器体系中的"禁行区过滤器"。
// 它订阅一张"禁行掩码地图"（OccupancyGrid，用户用地图编辑器预先绘制，
// 由 keepout_filter 的掩码发布节点发布），并把掩码中标记的禁行区
// 代价叠加到主代价地图上，使规划器/控制器把禁行区当作高代价/障碍
// 从而绕开（软约束：非致命代价时规划器尽量绕行，必要时仍可通过）。
//
// 【在 Nav2 中的位置】
// 继承链：Layer → CostmapFilter → KeepoutFilter。
// 使用两步订阅（典型 ROS2 "服务发现"模式）：
//   1) 订阅 filter_info_topic（nav2_msgs/CostmapFilterInfo）：
//      消息里含掩码话题名 mask_topic、base、multiplier；
//   2) 再订阅 mask_topic（nav_msgs/OccupancyGrid）得到掩码地图本身。
// 两个订阅都用 transient_local+reliable QoS，晚启动也能收到数据。
//
// 【核心逻辑】
// process()：把掩码值写进主地图——掩码值有效且大于主地图现值时覆盖
// （只增不减），使禁行区表现为高代价；NO_INFORMATION 不处理。
// 同坐标系时先计算两图重叠窗口，只在重叠区迭代（性能优化）；
// 异坐标系时通过 TF 逐点变换。
// ============================================================================

#include <string>
#include <memory>
#include <algorithm>
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_costmap_2d/costmap_filters/keepout_filter.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace nav2_costmap_2d
{

// 构造：初始化各订阅/掩码指针为空
KeepoutFilter::KeepoutFilter()
: filter_info_sub_(nullptr), mask_sub_(nullptr), mask_costmap_(nullptr),
  mask_frame_(""), global_frame_("")
{
}

// ============================================================================
// initializeFilter()：过滤器激活时调用——创建 filter info 订阅
// ============================================================================
void KeepoutFilter::initializeFilter(
  const std::string & filter_info_topic)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  filter_info_topic_ = filter_info_topic;
  // Setting new costmap filter info subscriber
  // transient_local QoS：掩码信息是"静态"的，晚启动也能拿到最后一条
  RCLCPP_INFO(
    logger_,
    "KeepoutFilter: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());
  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutFilter::filterInfoCallback, this, std::placeholders::_1));

  // 记录主地图的全局坐标系
  global_frame_ = layered_costmap_->getGlobalFrameID();
}

// ============================================================================
// filterInfoCallback()：CostmapFilterInfo 消息回调
// 第一次收到时创建掩码订阅；再次收到时销毁旧订阅重建
// （掩码话题可能在运行中被更换）
// ============================================================================
void KeepoutFilter::filterInfoCallback(
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
      "KeepoutFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "KeepoutFilter: New costmap filter info arrived from %s topic. Updating old filter info.",
      filter_info_topic_.c_str());
    // Resetting previous subscriber each time when new costmap filter information arrives
    // 新的 filter info 到达：销毁旧的掩码订阅，稍后按新话题重建
    mask_sub_.reset();
  }

  // Checking that base and multiplier are set to their default values
  // KeepoutFilter 不使用 base/multiplier（掩码值直接作为代价），
  // 若设置了非默认值给出错误提示
  if (msg->base != BASE_DEFAULT || msg->multiplier != MULTIPLIER_DEFAULT) {
    RCLCPP_ERROR(
      logger_,
      "KeepoutFilter: For proper use of keepout filter base and multiplier"
      " in CostmapFilterInfo message should be set to their default values (%f and %f)",
      BASE_DEFAULT, MULTIPLIER_DEFAULT);
  }

  // 从 filter info 中取出真正的掩码话题名
  mask_topic_ = msg->filter_mask_topic;

  // Setting new filter mask subscriber
  // 创建掩码地图订阅（OccupancyGrid）
  RCLCPP_INFO(
    logger_,
    "KeepoutFilter: Subscribing to \"%s\" topic for filter mask...",
    mask_topic_.c_str());
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutFilter::maskCallback, this, std::placeholders::_1));
}

// ============================================================================
// maskCallback()：掩码地图（OccupancyGrid）回调
// 把掩码转存为 Costmap2D 对象便于按栅格访问，并记录待更新区域
// ============================================================================
void KeepoutFilter::maskCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!mask_costmap_) {
    RCLCPP_INFO(
      logger_,
      "KeepoutFilter: Received filter mask from %s topic.", mask_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "KeepoutFilter: New filter mask arrived from %s topic. Updating old filter mask.",
      mask_topic_.c_str());
    mask_costmap_.reset();   // 丢弃旧掩码
  }

  // Making a new mask_costmap_
  // 用 OccupancyGrid 直接构造 Costmap2D（复制数据并转换 0~100 → 0~255）
  mask_costmap_ = std::make_unique<Costmap2D>(*msg);
  mask_frame_ = msg->header.frame_id;   // 记录掩码坐标系

  // 标记有新数据，待更新区域为整张掩码
  has_updated_data_ = true;
  x_ = y_ = 0;
  width_ = msg->info.width;
  height_ = msg->info.height;
}

// ============================================================================
// updateBounds()：分层更新第一阶段——调用基类记录位姿，
// 并在收到新掩码时把掩码覆盖区域并入更新边界
// ============================================================================
void KeepoutFilter::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  // 基类：记录机器人位姿
  CostmapFilter::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

  // 没有新掩码数据则不扩展边界
  if (!has_updated_data_) {
    return;
  }

  double wx, wy;

  // 把掩码覆盖区域的左下角转为世界坐标，并入更新边界
  layered_costmap_->getCostmap()->mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  // 右上角同理
  layered_costmap_->getCostmap()->mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  has_updated_data_ = false;   // 数据已提交
}

// ============================================================================
// process()：过滤器核心——把掩码代价叠加到主代价地图
// 同坐标系：先计算主地图窗口与掩码的重叠区，只在重叠区迭代（优化）；
// 异坐标系：查一次 TF，逐点做坐标变换后查掩码
// ============================================================================
void KeepoutFilter::process(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j,
  const geometry_msgs::msg::Pose2D & /*pose*/)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  // 尚未收到掩码：限流警告（每 2 秒最多一条）
  if (!mask_costmap_) {
    // Show warning message every 2 seconds to not litter an output
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "KeepoutFilter: Filter mask was not received");
    return;
  }

  // TF 变换对象（同坐标系时保持单位阵）
  tf2::Transform tf2_transform;
  tf2_transform.setIdentity();  // initialize by identical transform
  int mg_min_x, mg_min_y;  // masger_grid indexes of bottom-left window corner
  int mg_max_x, mg_max_y;  // masger_grid indexes of top-right window corner

  if (mask_frame_ != global_frame_) {
    // Filter mask and current layer are in different frames:
    // prepare frame transformation if mask_frame_ != global_frame_
    // 异坐标系：查询 mask_frame ← global_frame 的变换（只需查一次）
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        mask_frame_, global_frame_, tf2::TimePointZero,
        transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        logger_,
        "KeepoutFilter: Failed to get costmap frame (%s) "
        "transformation to mask frame (%s) with error: %s",
        global_frame_.c_str(), mask_frame_.c_str(), ex.what());
      return;
    }
    tf2::fromMsg(transform.transform, tf2_transform);

    // 异坐标系时无法用重叠优化，遍历整个窗口
    mg_min_x = min_i;
    mg_min_y = min_j;
    mg_max_x = max_i;
    mg_max_y = max_j;
  } else {
    // Filter mask and current layer are in the same frame:
    // apply the following optimization - iterate only in overlapped
    // (min_i, min_j)..(max_i, max_j) & mask_costmap_ area.
    //
    //           mask_costmap_
    //       *----------------------------*
    //       |                            |
    //       |                            |
    //       |      (2)                   |
    // *-----+-------*                    |
    // |     |///////|<- overlapped area  |
    // |     |///////|   to iterate in    |
    // |     *-------+--------------------*
    // |    (1)      |
    // |             |
    // *-------------*
    //  master_grid (min_i, min_j)..(max_i, max_j) window
    //
    // ToDo: after costmap rotation will be added, this should be re-worked.
    // 同坐标系优化：只在主地图窗口与掩码的重叠区（图中斜线区域）迭代。
    // 方法：把掩码四角的世界坐标换算成主地图栅格坐标，与窗口边界取交集

    double wx, wy;  // world coordinates

    // Calculating bounds corresponding to bottom-left overlapping (1) corner
    // mask_costmap_ -> master_grid intexes conversion
    // 掩码左下角（取半格中心）转主地图栅格坐标
    const double half_cell_size = 0.5 * mask_costmap_->getResolution();
    wx = mask_costmap_->getOriginX() + half_cell_size;
    wy = mask_costmap_->getOriginY() + half_cell_size;
    master_grid.worldToMapNoBounds(wx, wy, mg_min_x, mg_min_y);
    // Calculation of (1) corner bounds
    // 掩码完全在窗口左/下方：无重叠，直接返回
    if (mg_min_x >= max_i || mg_min_y >= max_j) {
      // There is no overlapping. Do nothing.
      return;
    }
    // 与窗口边界取交集
    mg_min_x = std::max(min_i, mg_min_x);
    mg_min_y = std::max(min_j, mg_min_y);

    // Calculating bounds corresponding to top-right window (2) corner
    // mask_costmap_ -> master_grid intexes conversion
    // 掩码右上角转主地图栅格坐标
    wx = mask_costmap_->getOriginX() +
      mask_costmap_->getSizeInCellsX() * mask_costmap_->getResolution() + half_cell_size;
    wy = mask_costmap_->getOriginY() +
      mask_costmap_->getSizeInCellsY() * mask_costmap_->getResolution() + half_cell_size;
    master_grid.worldToMapNoBounds(wx, wy, mg_max_x, mg_max_y);
    // Calculation of (2) corner bounds
    // 掩码完全在窗口右/上方：无重叠
    if (mg_max_x <= min_i || mg_max_y <= min_j) {
      // There is no overlapping. Do nothing.
      return;
    }
    mg_max_x = std::min(max_i, mg_max_x);
    mg_max_y = std::min(max_j, mg_max_y);
  }

  // unsigned<-signed conversions.
  // 有符号→无符号转换（前面已保证重叠区为正）
  unsigned const int mg_min_x_u = static_cast<unsigned int>(mg_min_x);
  unsigned const int mg_min_y_u = static_cast<unsigned int>(mg_min_y);
  unsigned const int mg_max_x_u = static_cast<unsigned int>(mg_max_x);
  unsigned const int mg_max_y_u = static_cast<unsigned int>(mg_max_y);

  unsigned int i, j;  // master_grid iterators
  unsigned int index;  // corresponding index of master_grid
  double gl_wx, gl_wy;  // world coordinates in a global_frame_
  double msk_wx, msk_wy;  // world coordinates in a mask_frame_
  unsigned int mx, my;  // mask_costmap_ coordinates
  unsigned char data, old_data;  // master_grid element data

  // Main master_grid updating loop
  // Iterate in costmap window by master_grid indexes
  // 主循环：遍历重叠区内的每个主地图格子
  unsigned char * master_array = master_grid.getCharMap();
  for (i = mg_min_x_u; i < mg_max_x_u; i++) {
    for (j = mg_min_y_u; j < mg_max_y_u; j++) {
      // 一维下标 index = i + j * size_x
      index = master_grid.getIndex(i, j);
      old_data = master_array[index];
      // Calculating corresponding to (i, j) point at mask_costmap_:
      // Get world coordinates in global_frame_
      // 主地图栅格 → 世界坐标
      master_grid.mapToWorld(i, j, gl_wx, gl_wy);
      if (mask_frame_ != global_frame_) {
        // Transform (i, j) point from global_frame_ to mask_frame_
        // 异坐标系：把世界坐标变换到掩码坐标系
        tf2::Vector3 point(gl_wx, gl_wy, 0);
        point = tf2_transform * point;
        msk_wx = point.x();
        msk_wy = point.y();
      } else {
        // In this case master_grid and filter-mask are in the same frame
        // 同坐标系：直接使用
        msk_wx = gl_wx;
        msk_wy = gl_wy;
      }
      // Get mask coordinates corresponding to (i, j) point at mask_costmap_
      // 世界坐标 → 掩码栅格坐标，成功则读取掩码代价
      if (mask_costmap_->worldToMap(msk_wx, msk_wy, mx, my)) {
        data = mask_costmap_->getCost(mx, my);
        // Update if mask_ data is valid and greater than existing master_grid's one
        // 掩码值为 NO_INFORMATION：不处理；
        // 否则仅当掩码值大于现值（或现值为未知）时写入——只增不减
        if (data == NO_INFORMATION) {
          continue;
        }
        if (data > old_data || old_data == NO_INFORMATION) {
          master_array[index] = data;
        }
      }
    }
  }
}

// ============================================================================
// resetFilter()：过滤器去激活/复位时调用——销毁两个订阅
// ============================================================================
void KeepoutFilter::resetFilter()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  filter_info_sub_.reset();
  mask_sub_.reset();
}

// ============================================================================
// isActive()：过滤器是否活跃——已收到掩码即视为活跃
// （LayeredCostmap 用 current 状态判断是否需要等待该层）
// ============================================================================
bool KeepoutFilter::isActive()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (mask_costmap_) {
    return true;
  }
  return false;
}

}  // namespace nav2_costmap_2d

// pluginlib 导出宏：注册 KeepoutFilter 为 Layer 插件
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::KeepoutFilter, nav2_costmap_2d::Layer)
