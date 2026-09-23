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
 *         Steve Macenski
 *********************************************************************/
// ============================================================================
// 中文注释：体素网格 Marker 可视化工具（costmap_2d_markers）可执行程序
//
// 【文件功能】
// 本文件是一个独立的调试工具节点：订阅 VoxelLayer 发布的 voxel_grid
// 话题（nav2_msgs/VoxelGrid），把其中 MARKED（占据）体素转换为
// visualization_msgs/Marker 的 CUBE_LIST（立方体列表）发布到
// visualization_marker 话题，供 RViz 以 Marker 显示 3D 占据。
// 与 costmap_2d_cloud（点云版本）互为替代方案。
//
// 【在 Nav2 中的位置】
// 纯调试工具，不属于运行时主链路。
//
// 【Marker 消息要点】
// - type=CUBE_LIST：一个 Marker 消息携带一列立方体（points 数组）；
// - scale.x/y/z 设为体素分辨率 → 每个立方体恰好一个体素大小；
// - 颜色取自按状态索引的颜色表（MARKED=蓝色）。
// ============================================================================

#include <string>
#include <vector>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav2_msgs/msg/voxel_grid.hpp"
#include "nav2_voxel_grid/voxel_grid.hpp"
#include "nav2_util/execution_timer.hpp"

// Cell：一个占据体素的世界坐标 + 状态
struct Cell
{
  double x;
  double y;
  double z;
  nav2_voxel_grid::VoxelStatus status;
};
typedef std::vector<Cell> V_Cell;

// 按状态索引的颜色表（下标对应 VoxelStatus 枚举）：
// FREE=0、UNKNOWN=1（绿色）、MARKED=2（蓝色）
float g_colors_r[] = {0.0f, 0.0f, 1.0f};
float g_colors_g[] = {0.0f, 0.0f, 0.0f};
float g_colors_b[] = {0.0f, 1.0f, 0.0f};
float g_colors_a[] = {0.0f, 0.5f, 1.0f};

// 全局缓存与节点/发布器
V_Cell g_cells;
rclcpp::Node::SharedPtr g_node;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub;

// ============================================================================
// voxelCallback()：voxel_grid 回调——收集占据体素并发布 CUBE_LIST Marker
// ============================================================================
void voxelCallback(const nav2_msgs::msg::VoxelGrid::ConstSharedPtr grid)
{
  // 空体素网格直接丢弃
  if (grid->data.empty()) {
    RCLCPP_ERROR(g_node->get_logger(), "Received voxel grid");
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

  // 收集全部占据（MARKED）体素，换算为体素中心的世界坐标
  g_cells.clear();
  uint32_t num_markers = 0;
  for (uint32_t y_grid = 0; y_grid < y_size; ++y_grid) {
    for (uint32_t x_grid = 0; x_grid < x_size; ++x_grid) {
      for (uint32_t z_grid = 0; z_grid < z_size; ++z_grid) {
        // 从位图提取该体素状态
        nav2_voxel_grid::VoxelStatus status =
          nav2_voxel_grid::VoxelGrid::getVoxel(
          x_grid, y_grid,
          z_grid, x_size, y_size, z_size, data);
        if (status == nav2_voxel_grid::MARKED) {
          Cell c;
          c.status = status;
          // 栅格坐标 → 世界坐标（格中心 +0.5 约定）
          c.x = x_origin + (x_grid + 0.5) * x_res;
          c.y = y_origin + (y_grid + 0.5) * y_res;
          c.z = z_origin + (z_grid + 0.5) * z_res;
          g_cells.push_back(c);

          ++num_markers;
        }
      }
    }
  }

  // ---- 组装 CUBE_LIST Marker 消息 ----
  auto m = std::make_unique<visualization_msgs::msg::Marker>();
  m->header.frame_id = frame_id;
  m->header.stamp = stamp;
  m->ns = g_node->get_namespace();   // 命名空间（同 ns+id 的 marker 会被覆盖）
  m->id = 0;
  m->type = visualization_msgs::msg::Marker::CUBE_LIST;   // 立方体列表类型
  m->action = visualization_msgs::msg::Marker::ADD;       // 添加/更新
  m->pose.orientation.w = 1.0;                            // 无旋转
  // 每个立方体的尺寸 = 一个体素的大小
  m->scale.x = x_res;
  m->scale.y = y_res;
  m->scale.z = z_res;
  // 颜色：按 MARKED 状态查表（蓝色）
  m->color.r = g_colors_r[nav2_voxel_grid::MARKED];
  m->color.g = g_colors_g[nav2_voxel_grid::MARKED];
  m->color.b = g_colors_b[nav2_voxel_grid::MARKED];
  m->color.a = g_colors_a[nav2_voxel_grid::MARKED];
  // 填充立方体中心点列表
  m->points.resize(num_markers);
  for (uint32_t i = 0; i < num_markers; ++i) {
    Cell & c = g_cells[i];
    geometry_msgs::msg::Point & p = m->points[i];
    p.x = c.x;
    p.y = c.y;
    p.z = c.z;
  }

  // 发布 Marker
  pub->publish(std::move(m));

  // 记录处理耗时
  timer.end();
  RCLCPP_INFO(
    g_node->get_logger(), "Published %d markers in %f seconds",
    num_markers, timer.elapsed_time_in_seconds());
}

// ============================================================================
// main()：创建节点、Marker 发布器、体素订阅，进入事件循环
// ============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("costmap_2d_marker");

  RCLCPP_DEBUG(g_node->get_logger(), "Starting costmap_2d_marker");

  // Marker 发布器（队列 1：只保留最新一帧）
  pub = g_node->create_publisher<visualization_msgs::msg::Marker>(
    "visualization_marker", 1);

  // 订阅 VoxelLayer 发布的体素栅格
  auto sub = g_node->create_subscription<nav2_msgs::msg::VoxelGrid>(
    "voxel_grid", rclcpp::SystemDefaultsQoS(), voxelCallback);

  // 阻塞事件循环直到节点关闭
  rclcpp::spin(g_node->get_node_base_interface());
}
