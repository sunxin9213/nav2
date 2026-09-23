/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
// 中文注释：体素层（VoxelLayer）实现文件
//
// 【文件功能】
// 本文件实现 Nav2 代价地图插件体系中的"体素层"——ObstacleLayer 的 3D 扩展。
// 它在 2D 代价地图之上维护一个三维体素栅格（VoxelGrid，nav2_voxel_grid 包），
// 按 z 轴分层记录障碍占据情况，可实现：
//   - 3D 标记：只有当某 (x,y) 列在垂直方向上被占据的体素数达到
//     mark_threshold 时，才把 2D 格子标记为致命障碍（过滤桌面下方等
//     "机器人能钻过去"的低矮结构）；
//   - 3D 清除：清除射线在体素空间中追踪，逐列统计未知体素数，
//     结合 unknown_threshold 判断格子应置为 FREE_SPACE 还是 NO_INFORMATION。
//
// 【在 Nav2 中的位置】
// LayeredCostmap 分层体系的可选感知层，通常用于局部代价地图替代
// ObstacleLayer，配合深度相机/3D 激光使用。继承关系：
//   Layer → CostmapLayer → ObstacleLayer → VoxelLayer
//
// 【核心概念】
// 1. 体素栅格内存布局：每个 (x,y) 列用一个 32 位整数的低 VOXEL_BITS(16)
//    位表示 z 向占据（每列最多 16 层 z_voxels），栅格数组为一维
//    unsigned int[size_x*size_y]，index = mx + my*size_x。
// 2. worldToMap3D / worldToMap3DFloat：三维世界坐标 → 体素格坐标。
// 3. 滚动窗口 updateOrigin：重写版本需同时搬移 2D costmap_ 与体素栅格。
// 4. 可发布 voxel_grid 话题（nav2_msgs/VoxelGrid，transient_local QoS）
//    供 RViz 可视化 3D 占据情况；清除射线终点发布到 clearing_endpoints。
// ============================================================================

#include "nav2_costmap_2d/voxel_layer.hpp"

#include <algorithm>
#include <cassert>
#include <vector>
#include <memory>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

// 每列体素位图使用的位数（最多 16 层 z 体素）
#define VOXEL_BITS 16
// pluginlib 导出宏：注册 VoxelLayer 为 Layer 插件
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::VoxelLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

// ============================================================================
// onInitialize()：先复用父类（障碍层）的传感器初始化，
// 再补充 3D 体素相关参数与体素地图发布器
// ============================================================================
void VoxelLayer::onInitialize()
{
  // 调用父类初始化：解析 observation_sources、创建观测缓冲与订阅
  ObstacleLayer::onInitialize();

  // ---- 声明体素层私有参数 ----
  declareParameter("enabled", rclcpp::ParameterValue(true));                       // 是否启用
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));    // 足迹清除
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));            // 障碍最大高度（米）
  declareParameter("z_voxels", rclcpp::ParameterValue(10));                       // z 方向体素层数
  declareParameter("origin_z", rclcpp::ParameterValue(0.0));                       // 体素栅格 z 原点（米）
  declareParameter("z_resolution", rclcpp::ParameterValue(0.2));                   // z 方向分辨率（米/层）
  declareParameter("unknown_threshold", rclcpp::ParameterValue(15));               // 未知体素数阈值
  declareParameter("mark_threshold", rclcpp::ParameterValue(0));                   // 标记阈值（列占据体素数）
  declareParameter("combination_method", rclcpp::ParameterValue(1));               // 与主地图合并方式
  declareParameter("publish_voxel_map", rclcpp::ParameterValue(false));            // 是否发布体素地图

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // ---- 读取体素层参数 ----
  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
  node->get_parameter(name_ + "." + "max_obstacle_height", max_obstacle_height_);
  node->get_parameter(name_ + "." + "z_voxels", size_z_);                    // z 层数
  node->get_parameter(name_ + "." + "origin_z", origin_z_);                  // z 原点
  node->get_parameter(name_ + "." + "z_resolution", z_resolution_);          // z 分辨率
  node->get_parameter(name_ + "." + "unknown_threshold", unknown_threshold_);// 未知阈值
  node->get_parameter(name_ + "." + "mark_threshold", mark_threshold_);      // 标记阈值
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  node->get_parameter(name_ + "." + "publish_voxel_map", publish_voxel_);

  // 可视化发布使用 transient_local + reliable QoS（晚启动的 RViz 也能收到）
  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  // 可选：创建体素栅格发布器（生命周期发布器需显式 on_activate）
  if (publish_voxel_) {
    voxel_pub_ = node->create_publisher<nav2_msgs::msg::VoxelGrid>(
      "voxel_grid", custom_qos);
    voxel_pub_->on_activate();
  }

  // 清除射线终点点云发布器（调试可视化用）
  clearing_endpoints_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "clearing_endpoints", custom_qos);
  clearing_endpoints_pub_->on_activate();

  // 未知阈值补齐高位：体素位图共 16 位，实际只用 size_z_ 层，
  // 未使用的位算作"未知"，因此阈值加上 (16 - size_z_)
  unknown_threshold_ += (VOXEL_BITS - size_z_);
  matchSize();   // 按尺寸初始化体素栅格

  // Add callback for dynamic parameters
  // 注册动态参数回调
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &VoxelLayer::dynamicParametersCallback,
      this, std::placeholders::_1));
}

// 析构：注销动态参数回调
VoxelLayer::~VoxelLayer()
{
  dyn_params_handler_.reset();
}

// ============================================================================
// matchSize()：先对齐 2D 尺寸（父类），再按 x/y/z 尺寸重建体素栅格
// ============================================================================
void VoxelLayer::matchSize()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  ObstacleLayer::matchSize();
  // 体素栅格：x/y 尺寸与 2D 地图一致，z 为 size_z_ 层
  voxel_grid_.resize(size_x_, size_y_, size_z_);
  assert(voxel_grid_.sizeX() == size_x_ && voxel_grid_.sizeY() == size_y_);
}

// ============================================================================
// reset()：复位（父类先复位观测缓存，再清空两张地图）
// ============================================================================
void VoxelLayer::reset()
{
  // Call the base class method before adding our own functionality
  ObstacleLayer::reset();
  resetMaps();
}

// ============================================================================
// resetMaps()：清空 2D 代价地图与 3D 体素栅格
// ============================================================================
void VoxelLayer::resetMaps()
{
  // Call the base class method before adding our own functionality
  // Note: at the time this was written, ObstacleLayer doesn't implement
  // resetMaps so this goes to the next layer down Costmap2DLayer which also
  // doesn't implement this, so it actually goes all the way to Costmap2D
  ObstacleLayer::resetMaps();   // 实际调到 Costmap2D::resetMaps 清 2D 地图
  voxel_grid_.reset();          // 清空体素位图
}

// ============================================================================
// updateBounds()：分层更新第一阶段——3D 标记障碍 + 3D 射线清除
// 逻辑与 ObstacleLayer 相同，但距离判断用 3D 欧氏距离，
// 标记时用体素占据计数决定是否写入 2D 致命障碍
// ============================================================================
void VoxelLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  // 滚动窗口：原点平移到机器人中心（注意 VoxelLayer 重写了 updateOrigin，
  // 会同时搬移 2D 地图和体素栅格）
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
  // 合并其他层请求的额外边界
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  // get the marking observations
  // 收集标记观测（isCurrent 判断传感器是否失联）
  current = getMarkingObservations(observations) && current;

  // get the clearing observations
  // 收集清除观测
  current = getClearingObservations(clearing_observations) && current;

  // update the global current status
  current_ = current;

  // raytrace freespace
  // 3D 射线清除：在体素空间中追踪视线，清除已消失的障碍
  for (unsigned int i = 0; i < clearing_observations.size(); ++i) {
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  // 3D 标记：遍历每条观测的每个点，写入体素栅格
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end();
    ++it)
  {
    const Observation & obs = *it;

    const sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);

    // 预计算距离平方上下限（避免内层循环反复开方）
    double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
    double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

    // PointCloud2 字段迭代器
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      // if the obstacle is too high or too far away from the robot we won't add it
      // 高度过滤：高于 max_obstacle_height_ 的点忽略
      if (*iter_z > max_obstacle_height_) {
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      // 3D 欧氏距离平方（比 2D 障碍层多考虑 z 分量）
      double sq_dist = (*iter_x - obs.origin_.x) * (*iter_x - obs.origin_.x) +
        (*iter_y - obs.origin_.y) * (*iter_y - obs.origin_.y) +
        (*iter_z - obs.origin_.z) * (*iter_z - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_max_range) {
        continue;
      }

      // If the point is too close, do not consider it
      if (sq_dist < sq_obstacle_min_range) {
        continue;
      }

      // now we need to compute the map coordinates for the observation
      // 三维世界坐标 → 体素格坐标；低于 z 原点的点被钳位到第 0 层
      unsigned int mx, my, mz;
      if (*iter_z < origin_z_) {
        if (!worldToMap3D(*iter_x, *iter_y, origin_z_, mx, my, mz)) {
          continue;
        }
      } else if (!worldToMap3D(*iter_x, *iter_y, *iter_z, mx, my, mz)) {
        continue;
      }

      // mark the cell in the voxel grid and check if we should also mark it in the costmap
      // 关键差异：先写入体素栅格；只有该列占据体素数 >= mark_threshold_
      // 时才把 2D 格子标为致命障碍（过滤稀疏噪声/可穿越结构）
      if (voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_)) {
        // 一维下标 index = mx + my * size_x
        unsigned int index = getIndex(mx, my);

        costmap_[index] = LETHAL_OBSTACLE;
        // 把该点扩展进更新边界
        touch(
          static_cast<double>(*iter_x), static_cast<double>(*iter_y),
          min_x, min_y, max_x, max_y);
      }
    }
  }

  // 可选：把体素栅格打包成 VoxelGrid 消息发布（RViz 3D 可视化）
  if (publish_voxel_) {
    auto grid_msg = std::make_unique<nav2_msgs::msg::VoxelGrid>();
    unsigned int size = voxel_grid_.sizeX() * voxel_grid_.sizeY();
    grid_msg->size_x = voxel_grid_.sizeX();
    grid_msg->size_y = voxel_grid_.sizeY();
    grid_msg->size_z = voxel_grid_.sizeZ();
    grid_msg->data.resize(size);
    // 直接内存拷贝体素位图（每列一个 unsigned int）
    memcpy(&grid_msg->data[0], voxel_grid_.getData(), size * sizeof(unsigned int));

    // 填充原点与各轴分辨率
    grid_msg->origin.x = origin_x_;
    grid_msg->origin.y = origin_y_;
    grid_msg->origin.z = origin_z_;

    grid_msg->resolutions.x = resolution_;
    grid_msg->resolutions.y = resolution_;
    grid_msg->resolutions.z = z_resolution_;
    grid_msg->header.frame_id = global_frame_;
    grid_msg->header.stamp = clock_->now();

    voxel_pub_->publish(std::move(grid_msg));
  }

  // 足迹顶点扩展进边界
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

// ============================================================================
// raytraceFreespace()：3D 射线清除——在体素空间追踪视线
// 与 2D 版本差异：用 3D 线段（起点=传感器，终点=观测点），
// 端点按 x/y/z 三个维度做地图边界裁剪；
// clearVoxelLineInMap 沿线清体素并同步更新 2D costmap_
// ============================================================================
void VoxelLayer::raytraceFreespace(
  const Observation & clearing_observation, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  // 清除终点点云（仅在有订阅者时才填充发布，节省开销）
  auto clearing_endpoints_ = std::make_unique<sensor_msgs::msg::PointCloud2>();

  // 空点云直接返回
  if (clearing_observation.cloud_->height == 0 || clearing_observation.cloud_->width == 0) {
    return;
  }

  double sensor_x, sensor_y, sensor_z;
  // 传感器原点（世界坐标）
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  double oz = clearing_observation.origin_.z;

  // 原点转体素格坐标（浮点，供 3D 直线算法使用），出界则无法清除
  if (!worldToMap3DFloat(ox, oy, oz, sensor_x, sensor_y, sensor_z)) {
    RCLCPP_WARN(
      logger_,
      "Sensor origin at (%.2f, %.2f %.2f) is out of map bounds "
      "(%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f). "
      "The costmap cannot raytrace for it.",
      ox, oy, oz,
      origin_x_, origin_y_, origin_z_,
      origin_x_ + getSizeInMetersX(), origin_y_ + getSizeInMetersY(),
      origin_z_ + getSizeInMetersZ());

    return;
  }

  bool publish_clearing_points;

  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }
    // 只在有人订阅 clearing_endpoints 时才构建并发布终点点云
    publish_clearing_points = (node->count_subscribers("clearing_endpoints") > 0);
  }

  // ---- 初始化终点点云消息的结构（xyz 三个 FLOAT32 字段）----
  clearing_endpoints_->data.clear();
  clearing_endpoints_->width = clearing_observation.cloud_->width;
  clearing_endpoints_->height = clearing_observation.cloud_->height;
  clearing_endpoints_->is_dense = true;
  clearing_endpoints_->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(*clearing_endpoints_);
  modifier.setPointCloud2Fields(
    3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);

  // 终点点云写入迭代器
  sensor_msgs::PointCloud2Iterator<float> clearing_endpoints_iter_x(*clearing_endpoints_, "x");
  sensor_msgs::PointCloud2Iterator<float> clearing_endpoints_iter_y(*clearing_endpoints_, "y");
  sensor_msgs::PointCloud2Iterator<float> clearing_endpoints_iter_z(*clearing_endpoints_, "z");

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  // 预计算三维地图边界（世界坐标）
  double map_end_x = origin_x_ + getSizeInMetersX();
  double map_end_y = origin_y_ + getSizeInMetersY();
  double map_end_z = origin_z_ + getSizeInMetersZ();

  // 遍历清除点云的每个点
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*(clearing_observation.cloud_), "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*(clearing_observation.cloud_), "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*(clearing_observation.cloud_), "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    double wpx = *iter_x;
    double wpy = *iter_y;
    double wpz = *iter_z;

    // 端点回缩：向传感器方向缩短 (2*resolution)，
    // 避免清除射线端点恰好落在障碍表面格上把障碍本身清掉
    double distance = dist(ox, oy, oz, wpx, wpy, wpz);
    double scaling_fact = 1.0;
    scaling_fact = std::max(std::min(scaling_fact, (distance - 2 * resolution_) / distance), 0.0);
    wpx = scaling_fact * (wpx - ox) + ox;
    wpy = scaling_fact * (wpy - oy) + oy;
    wpz = scaling_fact * (wpz - oz) + oz;

    // 射线方向向量 (a,b,c) 与缩放系数 t（初始 1.0）
    double a = wpx - ox;
    double b = wpy - oy;
    double c = wpz - oz;
    double t = 1.0;

    // we can only raytrace to a maximum z height
    // z 方向裁剪：上限为地图顶（留 0.01 余量），下限为地面
    if (wpz > map_end_z) {
      // we know we want the vector's z value to be max_z
      t = std::max(0.0, std::min(t, (map_end_z - 0.01 - oz) / c));
    } else if (wpz < origin_z_) {
      // and we can only raytrace down to the floor
      // we know we want the vector's z value to be 0.0
      t = std::min(t, (origin_z_ - oz) / c);
    }

    // the minimum value to raytrace from is the origin
    // x/y 下边界裁剪
    if (wpx < origin_x_) {
      t = std::min(t, (origin_x_ - ox) / a);
    }
    if (wpy < origin_y_) {
      t = std::min(t, (origin_y_ - oy) / b);
    }

    // the maximum value to raytrace to is the end of the map
    // x/y 上边界裁剪
    if (wpx > map_end_x) {
      t = std::min(t, (map_end_x - ox) / a);
    }
    if (wpy > map_end_y) {
      t = std::min(t, (map_end_y - oy) / b);
    }

    // 用最小缩放系数 t 截断射线终点（三个维度中先越界的决定 t）
    wpx = ox + a * t;
    wpy = oy + b * t;
    wpz = oz + c * t;

    double point_x, point_y, point_z;
    // 截断后的终点转体素格坐标（浮点）
    if (worldToMap3DFloat(wpx, wpy, wpz, point_x, point_y, point_z)) {
      // 清除射线的距离上下限（格数）
      unsigned int cell_raytrace_max_range = cellDistance(clearing_observation.raytrace_max_range_);
      unsigned int cell_raytrace_min_range = cellDistance(clearing_observation.raytrace_min_range_);


      // voxel_grid_.markVoxelLine(sensor_x, sensor_y, sensor_z, point_x, point_y, point_z);
      // 3D 直线追踪：从传感器到终点清除沿途体素，
      // 并同步维护 2D costmap_（FREE_SPACE / NO_INFORMATION 由 unknown_threshold_ 决定）
      voxel_grid_.clearVoxelLineInMap(
        sensor_x, sensor_y, sensor_z, point_x, point_y, point_z,
        costmap_,
        unknown_threshold_, mark_threshold_, FREE_SPACE, NO_INFORMATION,
        cell_raytrace_max_range, cell_raytrace_min_range);

      // 把射线终点扩展进更新边界
      updateRaytraceBounds(
        ox, oy, wpx, wpy, clearing_observation.raytrace_max_range_,
        clearing_observation.raytrace_min_range_, min_x, min_y,
        max_x,
        max_y);

      // 有订阅者时记录清除射线终点（调试可视化）
      if (publish_clearing_points) {
        *clearing_endpoints_iter_x = wpx;
        *clearing_endpoints_iter_y = wpy;
        *clearing_endpoints_iter_z = wpz;

        ++clearing_endpoints_iter_x;
        ++clearing_endpoints_iter_y;
        ++clearing_endpoints_iter_z;
      }
    }
  }

  // 发布清除终点点云
  if (publish_clearing_points) {
    clearing_endpoints_->header.frame_id = global_frame_;
    clearing_endpoints_->header.stamp = clearing_observation.cloud_->header.stamp;

    clearing_endpoints_pub_->publish(std::move(clearing_endpoints_));
  }
}

// ============================================================================
// updateOrigin()：滚动窗口原点平移（VoxelLayer 重写版）
// 与 Costmap2D::updateOrigin 思路相同——先把新旧窗口的交集暂存到局部数组，
// 复位地图、更新原点，再把交集搬回新位置；
// 区别是必须同时搬移 2D costmap_ 和 3D 体素栅格两份数据
// ============================================================================
void VoxelLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  // project the new origin into the grid
  // 新原点换算为栅格偏移量（格数）
  int cell_ox, cell_oy;
  cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  // compute the associated world coordinates for the origin cell
  // beacuase we want to keep things grid-aligned
  // 把新原点对齐到栅格线（避免亚栅格漂移累积）
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  // To save casting from unsigned int to int a bunch of times
  int size_x = size_x_;
  int size_y = size_y_;

  // we need to compute the overlap of the new and existing windows
  // 计算新旧窗口的重叠区域（裁剪到 [0, size] 范围内）
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = std::min(std::max(cell_ox, 0), size_x);
  lower_left_y = std::min(std::max(cell_oy, 0), size_y);
  upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  // we need a map to store the obstacles in the window temporarily
  // 临时缓冲区：保存重叠区内的 2D 代价与体素位图
  unsigned char * local_map = new unsigned char[cell_size_x * cell_size_y];
  unsigned int * local_voxel_map = new unsigned int[cell_size_x * cell_size_y];
  unsigned int * voxel_map = voxel_grid_.getData();

  // copy the local window in the costmap to the local map
  // 把重叠区数据拷贝到临时数组（两份：代价图 + 体素图）
  copyMapRegion(
    costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x,
    cell_size_x,
    cell_size_y);
  copyMapRegion(
    voxel_map, lower_left_x, lower_left_y, size_x_, local_voxel_map, 0, 0, cell_size_x,
    cell_size_x,
    cell_size_y);

  // we'll reset our maps to unknown space if appropriate
  // 复位整张地图（新移入的区域为默认/未知值）
  resetMaps();

  // update the origin with the appropriate world coordinates
  // 更新原点到对齐后的栅格坐标
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  // compute the starting cell location for copying data back in
  // 重叠区在新窗口中的起始位置
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  // now we want to copy the overlapping information back into the map, but in its new location
  // 把暂存的重叠区数据搬回新位置的地图中
  copyMapRegion(
    local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x,
    cell_size_y);
  copyMapRegion(
    local_voxel_map, 0, 0, cell_size_x, voxel_map, start_x, start_y, size_x_,
    cell_size_x,
    cell_size_y);

  // make sure to clean up
  // 释放临时缓冲区
  delete[] local_map;
  delete[] local_voxel_map;
}

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
// 中文注释：动态参数回调。高度/阈值类参数热更新；
// origin_z、z_resolution、z_voxels 变化需要重建体素栅格（matchSize）；
// publish_voxel_map 运行期不可改（发布器已固定）。
rcl_interfaces::msg::SetParametersResult
VoxelLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;
  bool resize_map_needed = false;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      // 最大高度可直接更新
      if (param_name == name_ + "." + "max_obstacle_height") {
        max_obstacle_height_ = parameter.as_double();
      } else if (param_name == name_ + "." + "origin_z") {
        // z 原点变化 → 体素栅格坐标系变化 → 需要重建
        origin_z_ = parameter.as_double();
        resize_map_needed = true;
      } else if (param_name == name_ + "." + "z_resolution") {
        // z 分辨率变化 → 同样需要重建
        z_resolution_ = parameter.as_double();
        resize_map_needed = true;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
        current_ = false;   // 重启后需要重新积累观测
      } else if (param_name == name_ + "." + "footprint_clearing_enabled") {
        footprint_clearing_enabled_ = parameter.as_bool();
      } else if (param_name == name_ + "." + "publish_voxel_map") {
        // 发布器在初始化时创建，运行期不可切换
        RCLCPP_WARN(
            logger_, "publish voxel map is not a dynamic parameter "
          "cannot be changed while running. Rejecting parameter update.");
        continue;
      }

    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == name_ + "." + "z_voxels") {
        // z 层数变化 → 栅格尺寸变化 → 重建
        size_z_ = parameter.as_int();
        resize_map_needed = true;
      } else if (param_name == name_ + "." + "unknown_threshold") {
        // 未知阈值：注意要补上未使用的高位位数
        unknown_threshold_ = parameter.as_int() + (VOXEL_BITS - size_z_);
      } else if (param_name == name_ + "." + "mark_threshold") {
        mark_threshold_ = parameter.as_int();
      } else if (param_name == name_ + "." + "combination_method") {
        combination_method_ = parameter.as_int();
      }
    }
  }

  // 尺寸相关参数变化后重建体素栅格
  if (resize_map_needed) {
    matchSize();
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_costmap_2d
