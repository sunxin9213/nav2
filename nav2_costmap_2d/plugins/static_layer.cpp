/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
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
// 中文注释：静态层（StaticLayer）实现文件
//
// 【文件功能】
// 本文件实现 Nav2 代价地图插件体系中的"静态层"。它订阅 map 话题上的
// nav_msgs::msg::OccupancyGrid（占用栅格地图，通常由 nav2_map_server 发布），
// 把静态地图的占据数据 [-1,100] 转换为代价地图的代价值
// [0,255]（FREE_SPACE/LETHAL_OBSTACLE/NO_INFORMATION），
// 作为全局代价地图的基础底图。
//
// 【在 Nav2 中的位置】
// 在 LayeredCostmap 分层体系中最典型的用法：全局代价地图
// (global_costmap) 通常由 StaticLayer + ObstacleLayer + InflationLayer 组成；
// 本层几乎总是排在 layers 列表第一位，提供"先验"的墙体/静态障碍信息。
//
// 【核心概念】
// 1. OccupancyGrid 消息：data 数组按行优先存储，取值 -1(未知)~100(占据)；
//    interpretValue() 负责把它映射为 costmap 的 0/254/255 三值或连续值。
// 2. QoS 机制：地图服务常使用 transient_local（迟加入的订阅者也能收到
//    最后一次发布的地图），本层可配置此 QoS 以兼容。
// 3. 世界坐标与地图坐标互换：mapToWorld()/worldToMap() 完成米 ↔ 栅格换算；
//    一维下标 index = mx + my * size_x。
// 4. 静态层对主地图使用 updateWithTrueOverwrite（直接覆盖）而非
//    updateWithMax，因为静态地图本身就是权威数据源。
// 5. 滚动窗口(rolling)模式下，静态层与主地图坐标系可能不同，
//    需要通过 TF2 把主地图坐标变换回地图坐标系再取值。
// ============================================================================

#include "nav2_costmap_2d/static_layer.hpp"

#include <algorithm>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/validate_messages.hpp"

// pluginlib 导出宏：注册 StaticLayer 为 Layer 插件，
// 由 Costmap2DROS 在 on_configure 时按 YAML 中的 plugin 字段动态加载
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::StaticLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

// 构造函数：仅初始化地图缓冲指针为空，其余工作推迟到 onInitialize()
StaticLayer::StaticLayer()
: map_buffer_(nullptr)
{
}

StaticLayer::~StaticLayer()
{
}

// ============================================================================
// onInitialize()：插件初始化。读取参数并创建地图话题订阅
// ============================================================================
void
StaticLayer::onInitialize()
{
  // 获取全局坐标系名称（如 "map"），本层输出的数据都以该坐标系为参考
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // 读取并声明所有参数（见 getParameters()）
  getParameters();

  // ---- 构造地图订阅的 QoS ----
  rclcpp::QoS map_qos(10);  // initialize to default
  if (map_subscribe_transient_local_) {
    // transient_local + reliable + keep_last(1)：
    // "迟到的订阅者也能拿到最后一次发布的地图"——与 map_server 的
    // 发布 QoS 匹配，保证重启/晚启动的节点仍能收到地图
    map_qos.transient_local();
    map_qos.reliable();
    map_qos.keep_last(1);
  }

  RCLCPP_INFO(
    logger_,
    "Subscribing to the map topic (%s) with %s durability",
    map_topic_.c_str(),
    map_subscribe_transient_local_ ? "transient local" : "volatile");

  // node_ 是 weak_ptr，lock() 提升后才能使用生命周期节点
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // 订阅 map 话题（OccupancyGrid），回调为 incomingMap()
  map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    std::bind(&StaticLayer::incomingMap, this, std::placeholders::_1));

  // 可选：订阅增量地图更新话题 <map_topic>_updates，
  // 地图服务器支持局部更新而不必重发全图
  if (subscribe_to_updates_) {
    RCLCPP_INFO(logger_, "Subscribing to updates");
    map_update_sub_ = node->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      map_topic_ + "_updates",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&StaticLayer::incomingUpdate, this, std::placeholders::_1));
  }
}

// activate()：生命周期激活阶段（本层无额外动作，参数回调已在初始化注册）
void
StaticLayer::activate()
{
}

// deactivate()：生命周期去激活阶段，注销动态参数回调
void
StaticLayer::deactivate()
{
  dyn_params_handler_.reset();
}

// reset()：复位层状态，强制下一周期重新向主地图写入全部数据
void
StaticLayer::reset()
{
  has_updated_data_ = true;
  current_ = false;
}

// ============================================================================
// getParameters()：集中声明并读取本层及全局参数
// 注意两类参数名的区别：
//   - 层私有参数："<层名>.<参数名>"，如 "static_layer.map_topic"
//   - 全局参数：直接 "map_topic"（由 Costmap2DROS 声明）
// ============================================================================
void
StaticLayer::getParameters()
{
  int temp_lethal_threshold = 0;
  double temp_tf_tol = 0.0;

  // ---- 参数声明（必须先声明后读取，否则抛异常）----
  declareParameter("enabled", rclcpp::ParameterValue(true));                       // 是否启用本层
  declareParameter("subscribe_to_updates", rclcpp::ParameterValue(false));         // 是否订阅地图增量更新
  declareParameter("map_subscribe_transient_local", rclcpp::ParameterValue(true)); // 地图订阅 QoS 持久性
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.0));            // TF 查询容差（秒）
  declareParameter("map_topic", rclcpp::ParameterValue(""));                       // 层私有地图话题名
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(false));   // 是否清除足迹下的代价

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // ---- 参数读取 ----
  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "subscribe_to_updates", subscribe_to_updates_);
  node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
  // 地图话题支持两级配置：层私有优先，否则回退到全局 map_topic 参数
  std::string private_map_topic, global_map_topic;
  node->get_parameter(name_ + "." + "map_topic", private_map_topic);
  node->get_parameter("map_topic", global_map_topic);
  if (!private_map_topic.empty()) {
    map_topic_ = private_map_topic;
  } else {
    map_topic_ = global_map_topic;
  }
  node->get_parameter(
    name_ + "." + "map_subscribe_transient_local",
    map_subscribe_transient_local_);
  // 以下为 Costmap2DROS 声明的全局参数
  node->get_parameter("track_unknown_space", track_unknown_space_);       // 是否区分未知区域
  node->get_parameter("use_maximum", use_maximum_);                       // 写主地图时取 max 还是覆盖
  node->get_parameter("lethal_cost_threshold", temp_lethal_threshold);    // 视为致命障碍的占据阈值
  node->get_parameter("unknown_cost_value", unknown_cost_value_);         // 地图中的未知值（通常 -1）
  node->get_parameter("trinary_costmap", trinary_costmap_);               // 是否输出三值代价图
  node->get_parameter("transform_tolerance", temp_tf_tol);

  // Enforce bounds
  // 把致命阈值裁剪到 [0,100]（OccupancyGrid 占据概率的合法区间）
  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  map_received_ = false;                  // 尚未收到任何地图
  map_received_in_update_bounds_ = false; // 本周期 updateBounds 时是否已有地图

  // 秒 → tf2 时长类型
  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  // Add callback for dynamic parameters
  // 注册动态参数回调，支持运行时改参数
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &StaticLayer::dynamicParametersCallback,
      this, std::placeholders::_1));
}

// ============================================================================
// processMap()：把收到的完整 OccupancyGrid 转换为本层内部的代价地图
// 处理尺寸对齐/调整，然后逐格转换代价值
// ============================================================================
void
StaticLayer::processMap(const nav_msgs::msg::OccupancyGrid & new_map)
{
  RCLCPP_DEBUG(logger_, "StaticLayer: Process map");

  // 新地图的尺寸（格数）
  unsigned int size_x = new_map.info.width;
  unsigned int size_y = new_map.info.height;

  RCLCPP_DEBUG(
    logger_,
    "StaticLayer: Received a %d X %d map at %f m/pix", size_x, size_y,
    new_map.info.resolution);

  // resize costmap if size, resolution or origin do not match
  // 若静态地图与主代价地图尺寸/分辨率/原点不一致，需要调整大小
  Costmap2D * master = layered_costmap_->getCostmap();
  // 非滚动模式下：若不匹配（或尺寸未锁定），调整整个分层地图及所有层的尺寸
  if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||
    master->getSizeInCellsY() != size_y ||
    master->getResolution() != new_map.info.resolution ||
    master->getOriginX() != new_map.info.origin.position.x ||
    master->getOriginY() != new_map.info.origin.position.y ||
    !layered_costmap_->isSizeLocked()))
  {
    // Update the size of the layered costmap (and all layers, including this one)
    RCLCPP_INFO(
      logger_,
      "StaticLayer: Resizing costmap to %d X %d at %f m/pix", size_x, size_y,
      new_map.info.resolution);
    layered_costmap_->resizeMap(
      size_x, size_y, new_map.info.resolution,
      new_map.info.origin.position.x,
      new_map.info.origin.position.y,
      true);
  } else if (size_x_ != size_x || size_y_ != size_y ||  // NOLINT
    resolution_ != new_map.info.resolution ||
    origin_x_ != new_map.info.origin.position.x ||
    origin_y_ != new_map.info.origin.position.y)
  {
    // only update the size of the costmap stored locally in this layer
    // 滚动窗口模式下（或主地图无需变）：只调整本层内部缓存地图的尺寸
    RCLCPP_INFO(
      logger_,
      "StaticLayer: Resizing static layer to %d X %d at %f m/pix", size_x, size_y,
      new_map.info.resolution);
    resizeMap(
      size_x, size_y, new_map.info.resolution,
      new_map.info.origin.position.x, new_map.info.origin.position.y);
  }

  unsigned int index = 0;

  // we have a new map, update full size of map
  // 加锁保护本层数据（与 updateCosts 等并发访问互斥）
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  // initialize the costmap with static data
  // 逐格转换：OccupancyGrid 的 int8 data → costmap 的 unsigned char 代价
  // 两个数组同为行优先一维布局，直接按相同下标搬运
  for (unsigned int i = 0; i < size_y; ++i) {
    for (unsigned int j = 0; j < size_x; ++j) {
      unsigned char value = new_map.data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }

  // 记录地图坐标系（如 "map"），供后续 TF 变换使用
  map_frame_ = new_map.header.frame_id;

  // 标记本层待写入主地图的区域为整张地图（x_,y_ 为起点，width_/height_ 为尺寸）
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  has_updated_data_ = true;   // 通知 updateBounds 有新数据需要处理

  current_ = true;            // 本层输出有效
}

// ============================================================================
// matchSize()：与主代价地图尺寸对齐（本层继承 Costmap2D 的存储）
// 滚动窗口模式下静态层尺寸与主地图无关，不做调整
// ============================================================================
void
StaticLayer::matchSize()
{
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
  if (!layered_costmap_->isRolling()) {
    Costmap2D * master = layered_costmap_->getCostmap();
    resizeMap(
      master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
      master->getOriginX(), master->getOriginY());
  }
}

// ============================================================================
// interpretValue()：把 OccupancyGrid 的占据值 (-1~100) 翻译为代价 (0~255)
// 这是"静态地图语义 → 代价地图语义"的核心映射：
//   -1/unknown_cost_value → NO_INFORMATION(255) 或 FREE_SPACE(0)
//   >= lethal_threshold_  → LETHAL_OBSTACLE(254)
//   其余（trinary 关闭时）→ 按比例线性缩放到 [0,254]
// ============================================================================
unsigned char
StaticLayer::interpretValue(unsigned char value)
{
  // check if the static value is above the unknown or lethal thresholds
  // 未知区域的两种处理策略：
  //   track_unknown_space_=true：保留为 NO_INFORMATION(255)；
  //   track_unknown_space_=false：当作自由空间 FREE_SPACE(0)
  if (track_unknown_space_ && value == unknown_cost_value_) {
    return NO_INFORMATION;
  } else if (!track_unknown_space_ && value == unknown_cost_value_) {
    return FREE_SPACE;
  } else if (value >= lethal_threshold_) {
    // 占据概率达到致命阈值 → 致命障碍
    return LETHAL_OBSTACLE;
  } else if (trinary_costmap_) {
    // 三值代价图模式：非障碍非未知一律视为自由空间
    return FREE_SPACE;
  }

  // 连续代价模式：按占据比例线性映射到 [0, LETHAL_OBSTACLE]
  double scale = static_cast<double>(value) / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

// ============================================================================
// incomingMap()：map 话题订阅回调（OccupancyGrid）
// 首张地图立即处理；后续地图只缓存到 map_buffer_，
// 延迟到下一个 updateBounds 周期再处理，避免在回调中做重活
// ============================================================================
void
StaticLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map)
{
  // 先做消息合法性校验（尺寸、分辨率、数据长度等），坏消息直接丢弃
  if (!nav2_util::validateMsg(*new_map)) {
    RCLCPP_ERROR(logger_, "Received map message is malformed. Rejecting.");
    return;
  }
  if (!map_received_) {
    // 第一张地图：直接处理（此时通常还没开始更新周期）
    processMap(*new_map);
    map_received_ = true;
    return;
  }
  // 已有地图：仅缓存，等 updateBounds 中再切换，保证线程安全
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  map_buffer_ = new_map;
}

// ============================================================================
// incomingUpdate()：<map_topic>_updates 话题回调（增量地图更新）
// 只更新地图的一个子矩形区域
// ============================================================================
void
StaticLayer::incomingUpdate(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  // 越界检查：更新矩形必须完全落在本层地图范围内，否则忽略
  if (update->y < static_cast<int32_t>(y_) ||
    y_ + height_ < update->y + update->height ||
    update->x < static_cast<int32_t>(x_) ||
    x_ + width_ < update->x + update->width)
  {
    RCLCPP_WARN(
      logger_,
      "StaticLayer: Map update ignored. Exceeds bounds of static layer.\n"
      "Static layer origin: %d, %d   bounds: %d X %d\n"
      "Update origin: %d, %d   bounds: %d X %d",
      x_, y_, width_, height_, update->x, update->y, update->width,
      update->height);
    return;
  }

  // 坐标系一致性检查：更新必须与当前地图处于同一 frame
  if (update->header.frame_id != map_frame_) {
    RCLCPP_WARN(
      logger_,
      "StaticLayer: Map update ignored. Current map is in frame %s "
      "but update was in frame %s",
      map_frame_.c_str(), update->header.frame_id.c_str());
    return;
  }

  // 把更新矩形逐格写入本层代价地图（interpretValue 做值转换）
  // 一维下标换算：index = (update->y + y) * size_x_ + update->x + x
  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height; y++) {
    unsigned int index_base = (update->y + y) * size_x_;
    for (unsigned int x = 0; x < update->width; x++) {
      unsigned int index = index_base + x + update->x;
      costmap_[index] = interpretValue(update->data[di++]);
    }
  }

  has_updated_data_ = true;   // 通知 updateBounds 有新数据
}


// ============================================================================
// updateBounds()：分层更新第一阶段——计算本层需要写入主地图的世界坐标边界
// ============================================================================
void
StaticLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  // 尚未收到地图：无法提供任何边界
  if (!map_received_) {
    map_received_in_update_bounds_ = false;
    return;
  }
  map_received_in_update_bounds_ = true;

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  // If there is a new available map, load it.
  // 若缓存了新地图（incomingMap 中暂存的），在这里切换处理——
  // 保证地图替换发生在更新周期内而不是订阅回调里
  if (map_buffer_) {
    processMap(*map_buffer_);
    map_buffer_ = nullptr;
  }

  // 非滚动模式下：没有新数据且无额外边界则无需更新
  if (!layered_costmap_->isRolling() ) {
    if (!(has_updated_data_ || has_extra_bounds_)) {
      return;
    }
  }

  // 合并额外边界（其他层请求重绘的区域）
  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx, wy;

  // 把本层待更新区域的左下角栅格坐标转换为世界坐标，
  // 并与传入边界取 min —— 扩大总更新范围
  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  // 右上角同理，取 max
  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  has_updated_data_ = false;   // 数据已提交，清标志

  // 把机器人足迹也纳入边界（用于足迹清除功能）
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

// ============================================================================
// updateFootprint()：把机器人足迹顶点扩展进更新边界
// （为 footprint_clearing_enabled_ 时清除足迹下代价做准备）
// ============================================================================
void
StaticLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {return;}

  // 把足迹多边形按当前位姿旋转平移到世界坐标
  transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  // 依次用每个顶点"触碰"边界（touch：把点扩进 min/max 边界）
  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

// ============================================================================
// updateCosts()：分层更新第二阶段——把本层静态数据写入主代价地图
// 非滚动模式：本层与主地图同坐标系，直接整块拷贝；
// 滚动模式：两者坐标系可能不同，需逐格做 TF 变换
// ============================================================================
void
StaticLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }
  // 本周期 updateBounds 时还没有地图 → 无法写入（限流打印警告）
  if (!map_received_in_update_bounds_) {
    static int count = 0;
    // throttle warning down to only 1/10 message rate
    if (++count == 10) {
      RCLCPP_WARN(logger_, "Can't update static costmap layer, no map received");
      count = 0;
    }
    return;
  }

  // 足迹清除：把机器人自身占据区域强行置为自由空间，
  // 防止机器人"被自己的初始位置卡住"
  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }

  if (!layered_costmap_->isRolling()) {
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
    // 非滚动：两图同坐标。静态地图是权威底图，默认直接覆盖
    // （use_maximum_=true 时改用取最大值合并）
    if (!use_maximum_) {
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    } else {
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    }
  } else {
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer
    // 滚动窗口：主地图是跟随机器人移动的滑动窗口，与本层地图原点不同，
    // 甚至可能处于不同坐标系，因此需要 TF 变换
    unsigned int mx, my;
    double wx, wy;
    // Might even be in a different frame
    // 查询 map_frame_ → global_frame_ 的静态变换（TimePointZero 取最新可用）
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        map_frame_, global_frame_, tf2::TimePointZero,
        transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "StaticLayer: %s", ex.what());
      return;
    }
    // Copy map data given proper transformations
    // 转为 tf2::Transform 便于矩阵运算
    tf2::Transform tf2_transform;
    tf2::fromMsg(transform.transform, tf2_transform);

    // 遍历主地图更新区域内的每个格子
    for (int i = min_i; i < max_i; ++i) {
      for (int j = min_j; j < max_j; ++j) {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        // 主地图栅格坐标 → 世界坐标（考虑主地图分辨率与原点）
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from global_frame_ to map_frame_
        // 世界坐标经 TF 变换到静态地图的坐标系
        tf2::Vector3 p(wx, wy, 0);
        p = tf2_transform * p;
        // Set master_grid with cell from map
        // 变换后的世界坐标 → 本层地图栅格坐标，成功则写入主地图
        if (worldToMap(p.x(), p.y(), mx, my)) {
          if (!use_maximum_) {
            master_grid.setCost(i, j, getCost(mx, my));
          } else {
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
          }
        }
      }
    }
  }
  current_ = true;   // 本层输出有效
}

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
// 中文注释：动态参数回调。运行时允许修改 transform_tolerance/enabled/
// footprint_clearing_enabled；订阅相关的参数不支持动态修改，会被拒绝。
rcl_interfaces::msg::SetParametersResult
StaticLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  // 遍历本次修改的所有参数
  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    // 订阅相关参数在运行期不可变（订阅已建立），拒绝修改
    if (param_name == name_ + "." + "map_subscribe_transient_local" ||
      param_name == name_ + "." + "map_topic" ||
      param_name == name_ + "." + "subscribe_to_updates")
    {
      RCLCPP_WARN(
        logger_, "%s is not a dynamic parameter "
        "cannot be changed while running. Rejecting parameter update.", param_name.c_str());
    } else if (param_type == ParameterType::PARAMETER_DOUBLE) {
      // TF 容差可以动态调整
      if (param_name == name_ + "." + "transform_tolerance") {
        transform_tolerance_ = tf2::durationFromSec(parameter.as_double());
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      // 启用开关：重新启用时标记整图重写
      if (param_name == name_ + "." + "enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();

        x_ = y_ = 0;
        width_ = size_x_;
        height_ = size_y_;
        has_updated_data_ = true;
        current_ = false;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      // 足迹清除开关可以动态调整
      if (param_name == name_ + "." + "footprint_clearing_enabled") {
        footprint_clearing_enabled_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_costmap_2d
