/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// ============================================================================
// 中文注释：体素网格点云可视化工具（costmap_2d_cloud）可执行程序
//
// 【文件功能】
// 本文件是一个独立的调试工具节点：订阅 VoxelLayer 发布的 voxel_grid
// 话题（nav2_msgs/VoxelGrid，3D 体素位图），把其中 MARKED（已占据）与
// UNKNOWN（未知）的体素分别转换为带颜色的 PointCloud2 点云发布：
//   - voxel_marked_cloud：占据体素（红色）
//   - voxel_unknown_cloud：未知体素（绿色）
// 供 RViz 以 PointCloud2 显示 3D 占据情况，便于调试体素层。
//
// 【在 Nav2 中的位置】
// 纯调试工具，不属于代价地图运行时主链路；与 costmap_2d_markers
// 互为替代（后者输出 Marker 立方体列表）。
//
// 【体素栅格内存模型】
// VoxelGrid.data 是一维 uint32 数组，每个元素对应一个 (x,y) 列，
// 低 16 位按 z 层记录占据位图；
// getVoxel(x,y,z,...) 负责位运算提取状态（FREE/MARKED/UNKNOWN）。
// ============================================================================

#include <string>
#include <vector>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_voxel_grid/voxel_grid.hpp"
#include "nav2_msgs/msg/voxel_grid.hpp"
#include "nav2_util/execution_timer.hpp"

// ============================================================================
// mapToWorld3D()：三维栅格坐标 → 世界坐标（格中心约定，+0.5）
// 与 Costmap2D::mapToWorld 的 3D 版本
// ============================================================================
static inline void mapToWorld3D(
  const unsigned int mx,
  const unsigned int my, const unsigned int mz,
  const double origin_x, const double origin_y, const double origin_z,
  const double x_resolution, const double y_resolution,
  const double z_resolution,
  double & wx, double & wy, double & wz)
{
  // returns the center point of the cell
  wx = origin_x + (mx + 0.5) * x_resolution;
  wy = origin_y + (my + 0.5) * y_resolution;
  wz = origin_z + (mz + 0.5) * z_resolution;
}

// Cell：一个体素的世界坐标 + 状态（用于收集后转点云）
struct Cell
{
  double x;
  double y;
  double z;
  nav2_voxel_grid::VoxelStatus status;
};
typedef std::vector<Cell> V_Cell;

// 按状态索引的颜色表（下标对应 VoxelStatus 枚举）：
// FREE=0（黑透明）、UNKNOWN=1（绿色）、MARKED=2（蓝色）
float g_colors_r[] = {0.0f, 0.0f, 1.0f};
float g_colors_g[] = {0.0f, 0.0f, 0.0f};
float g_colors_b[] = {0.0f, 1.0f, 0.0f};
float g_colors_a[] = {0.0f, 0.5f, 1.0f};

// 全局缓存：占据体素与未知体素两个集合
V_Cell g_marked;
V_Cell g_unknown;

// 全局节点与两个发布器（占据点云、未知点云）
rclcpp::Node::SharedPtr g_node;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_marked;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_unknown;

/**
 * @brief An helper function to fill pointcloud2 of both the marked and unknown points from voxel_grid
 * @param cloud PointCloud2 Ptr which needs to be filled
 * @param num_channels Represents the total number of points that are going to be filled
 * @param header Carries the header information that needs to be assigned to PointCloud2 header
 * @param g_cells contains the x, y, z values that needs to be added to the PointCloud2
 */
// 中文注释：把 Cell 集合填充为带 RGB 颜色的 PointCloud2 消息。
// 字段结构：x/y/z (FLOAT32) + r/g/b (UINT8)，每个体素一个点，
// 颜色按体素状态查颜色表。
void pointCloud2Helper(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> & cloud,
  uint32_t num_channels,
  std_msgs::msg::Header header,
  V_Cell & g_cells)
{
  // 消息头与点云结构：一维点云（height=1，width=点数）
  cloud->header = header;
  cloud->width = num_channels;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->is_bigendian = false;
  // Modifier：按字段定义分配内存
  sensor_msgs::PointCloud2Modifier modifier(*cloud);

  // 定义 6 个字段：坐标 xyz + 颜色 rgb
  modifier.setPointCloud2Fields(
    6, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "r", 1, sensor_msgs::msg::PointField::UINT8,
    "g", 1, sensor_msgs::msg::PointField::UINT8,
    "b", 1, sensor_msgs::msg::PointField::UINT8);

  // 各字段写入迭代器
  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud, "b");

  // 逐点写入坐标与颜色（颜色按状态查表，0~1 缩放到 0~255）
  for (uint32_t i = 0; i < num_channels; ++i) {
    Cell & c = g_cells[i];
    // assigning value to the point cloud2's iterator
    *iter_x = c.x;
    *iter_y = c.y;
    *iter_z = c.z;
    *iter_r = g_colors_r[c.status] * 255.0;
    *iter_g = g_colors_g[c.status] * 255.0;
    *iter_b = g_colors_b[c.status] * 255.0;

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
  }
}

// ============================================================================
// voxelCallback()：voxel_grid 话题回调——
// 遍历全部体素，按状态分为 marked/unknown 两个点集并发布
// ============================================================================
void voxelCallback(const nav2_msgs::msg::VoxelGrid::ConstSharedPtr grid)
{
  // 空体素网格直接丢弃
  if (grid->data.empty()) {
    RCLCPP_ERROR(g_node->get_logger(), "Received empty voxel grid");
    return;
  }

  // 性能计时（调试用）
  nav2_util::ExecutionTimer timer;
  timer.start();

  RCLCPP_DEBUG(g_node->get_logger(), "Received voxel grid");
  // 提取消息中的几何信息
  const std::string frame_id = grid->header.frame_id;
  const rclcpp::Time stamp = grid->header.stamp;
  const uint32_t * data = &grid->data.front();   // 体素位图数据
  const double x_origin = grid->origin.x;
  const double y_origin = grid->origin.y;
  const double z_origin = grid->origin.z;
  const double x_res = grid->resolutions.x;
  const double y_res = grid->resolutions.y;
  const double z_res = grid->resolutions.z;
  const uint32_t x_size = grid->size_x;
  const uint32_t y_size = grid->size_y;
  const uint32_t z_size = grid->size_z;

  // 清空上一帧的缓存
  g_marked.clear();
  g_unknown.clear();
  uint32_t num_marked = 0;
  uint32_t num_unknown = 0;
  // 三重循环遍历所有体素 (x,y,z)
  for (uint32_t y_grid = 0; y_grid < y_size; ++y_grid) {
    for (uint32_t x_grid = 0; x_grid < x_size; ++x_grid) {
      for (uint32_t z_grid = 0; z_grid < z_size; ++z_grid) {
        // 从位图中提取该体素的状态（FREE/MARKED/UNKNOWN）
        nav2_voxel_grid::VoxelStatus status =
          nav2_voxel_grid::VoxelGrid::getVoxel(
          x_grid, y_grid,
          z_grid, x_size, y_size, z_size, data);
        if (status == nav2_voxel_grid::UNKNOWN) {
          // 未知体素：记录世界坐标并加入 unknown 集合
          Cell c;
          c.status = status;
          mapToWorld3D(
            x_grid, y_grid, z_grid, x_origin, y_origin,
            z_origin, x_res, y_res, z_res, c.x, c.y, c.z);

          g_unknown.push_back(c);

          ++num_unknown;
        } else if (status == nav2_voxel_grid::MARKED) {
          // 占据体素：记录世界坐标并加入 marked 集合
          Cell c;
          c.status = status;
          mapToWorld3D(
            x_grid, y_grid, z_grid, x_origin, y_origin,
            z_origin, x_res, y_res, z_res, c.x, c.y, c.z);

          g_marked.push_back(c);

          ++num_marked;
        }
      }
    }
  }

  // 点云消息头（沿用体素网格的坐标系与时间戳）
  std_msgs::msg::Header pcl_header;
  pcl_header.frame_id = frame_id;
  pcl_header.stamp = stamp;

  // 分别发布占据点云与未知点云
  {
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pointCloud2Helper(cloud, num_marked, pcl_header, g_marked);
    pub_marked->publish(std::move(cloud));
  }

  {
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pointCloud2Helper(cloud, num_unknown, pcl_header, g_unknown);
    pub_unknown->publish(std::move(cloud));
  }

  // 记录处理耗时
  timer.end();
  RCLCPP_DEBUG(
    g_node->get_logger(), "Published %d points in %f seconds",
    num_marked + num_unknown, timer.elapsed_time_in_seconds());
}

// ============================================================================
// main()：创建节点、两个点云发布器、体素订阅，进入事件循环
// ============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("costmap_2d_cloud");

  RCLCPP_DEBUG(g_node->get_logger(), "Starting up costmap_2d_cloud");

  // 占据体素点云发布器（队列 1：只保留最新）
  pub_marked = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "voxel_marked_cloud", 1);
  // 未知体素点云发布器
  pub_unknown = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "voxel_unknown_cloud", 1);
  // 订阅 VoxelLayer 发布的体素栅格
  auto sub = g_node->create_subscription<nav2_msgs::msg::VoxelGrid>(
    "voxel_grid", rclcpp::SystemDefaultsQoS(), voxelCallback);

  // 阻塞事件循环直到节点关闭
  rclcpp::spin(g_node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
