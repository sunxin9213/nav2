// Copyright (c) 2018 Intel Corporation
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
// 中文注释：清除代价地图服务（ClearCostmapService）实现文件
//
// 【文件功能】
// 本文件实现"清除代价地图"的三个 ROS 服务（用户可通过
// ros2 service call 或 RViz 工具触发）：
//   1. clear_except_<costmap名>（ClearCostmapExceptRegion）：
//      清除"机器人周围 reset_distance 见方区域以外"的全部内容；
//   2. clear_around_<costmap名>（ClearCostmapAroundRobot）：
//      清除"机器人周围 reset_distance 见方区域内"的内容；
//   3. clear_entirely_<costmap名>（ClearEntireCostmap）：整图复位。
//
// 【在 Nav2 中的位置】
// 由 Costmap2DROS 在配置阶段创建；Nav2 的行为树服务节点
// （如 clear_costmap 服务节点）与 RViz 工具最终都调用到这里。
// 常用于机器人被"幻影障碍"困住时手动恢复。
//
// 【核心机制】
// - 只清除 isClearable()==true 的层（障碍层/体素层等感知层），
//   静态层与过滤器不可清除（静态地图是先验、过滤器有语义）；
// - clearArea(invert) 的 invert 参数区分"清除外围/清内部"；
// - addExtraBounds：把整个层标记为待重绘，使下一更新周期
//   LayeredCostmap 重新向主地图写入该层数据。
// ============================================================================

#include <vector>
#include <string>
#include <algorithm>
#include <memory>

#include "nav2_costmap_2d/clear_costmap_service.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_costmap_2d
{

using std::vector;
using std::string;
using std::shared_ptr;
using std::any_of;
using ClearExceptRegion = nav2_msgs::srv::ClearCostmapExceptRegion;
using ClearAroundRobot = nav2_msgs::srv::ClearCostmapAroundRobot;
using ClearEntirely = nav2_msgs::srv::ClearEntireCostmap;

// ============================================================================
// 构造函数：创建三个清除服务（服务名带上代价地图节点名以区分全局/局部）
// ============================================================================

ClearCostmapService::ClearCostmapService(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  Costmap2DROS & costmap)
: costmap_(costmap)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  // 复位值 = 主地图默认值（通常是 NO_INFORMATION 或 FREE_SPACE）
  reset_value_ = costmap_.getCostmap()->getDefaultValue();

  // 服务 1：清除指定区域以外（保留机器人周围）
  clear_except_service_ = node->create_service<ClearExceptRegion>(
    "clear_except_" + costmap_.getName(),
    std::bind(
      &ClearCostmapService::clearExceptRegionCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // 服务 2：清除机器人周围指定区域
  clear_around_service_ = node->create_service<ClearAroundRobot>(
    "clear_around_" + costmap.getName(),
    std::bind(
      &ClearCostmapService::clearAroundRobotCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // 服务 3：整图复位
  clear_entire_service_ = node->create_service<ClearEntirely>(
    "clear_entirely_" + costmap_.getName(),
    std::bind(
      &ClearCostmapService::clearEntireCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

// 析构：确保服务被注销
ClearCostmapService::~ClearCostmapService()
{
  // make sure services shutdown.
  clear_except_service_.reset();
  clear_around_service_.reset();
  clear_entire_service_.reset();
}

// ============================================================================
// 服务回调 1：清除区域以外（invert=true 表示清除矩形外）
// ============================================================================

void ClearCostmapService::clearExceptRegionCallback(
  const shared_ptr<rmw_request_id_t>/*request_header*/,
  const shared_ptr<ClearExceptRegion::Request> request,
  const shared_ptr<ClearExceptRegion::Response>/*response*/)
{
  RCLCPP_INFO(
    logger_, "%s",
    ("Received request to clear except a region the " + costmap_.getName()).c_str());

  // invert=true：保留机器人周围区域，清除其余部分
  clearRegion(request->reset_distance, true);
}

// ============================================================================
// 服务回调 2：清除机器人周围区域（invert=false 表示清除矩形内）
// ============================================================================

void ClearCostmapService::clearAroundRobotCallback(
  const shared_ptr<rmw_request_id_t>/*request_header*/,
  const shared_ptr<ClearAroundRobot::Request> request,
  const shared_ptr<ClearAroundRobot::Response>/*response*/)
{
  // invert=false：只清除机器人周围区域
  clearRegion(request->reset_distance, false);
}

// ============================================================================
// 服务回调 3：整图复位（调用 LayeredCostmap 的 resetLayers）
// ============================================================================

void ClearCostmapService::clearEntireCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ClearEntirely::Request>/*request*/,
  const std::shared_ptr<ClearEntirely::Response>/*response*/)
{
  RCLCPP_INFO(
    logger_, "%s",
    ("Received request to clear entirely the " + costmap_.getName()).c_str());

  clearEntirely();
}

// ============================================================================
// clearRegion()：按机器人位置与 reset_distance 清除可清除层的区域
// invert=true 清矩形外；false 清矩形内
// ============================================================================

void ClearCostmapService::clearRegion(const double reset_distance, bool invert)
{
  double x, y;

  // 需要先拿到机器人当前位置（清除区域以机器人为中心）
  if (!getPosition(x, y)) {
    RCLCPP_ERROR(
      logger_, "%s",
      "Cannot clear map because robot pose cannot be retrieved.");
    return;
  }

  // 遍历分层地图的全部插件层
  auto layers = costmap_.getLayeredCostmap()->getPlugins();

  for (auto & layer : *layers) {
    // 只清除标记为"可清除"的层（障碍/体素层等感知层）；
    // 静态层与过滤器不可清除
    if (layer->isClearable()) {
      auto costmap_layer = std::static_pointer_cast<CostmapLayer>(layer);
      clearLayerRegion(costmap_layer, x, y, reset_distance, invert);
    }
  }

  // AlexeyMerzlyakov: No need to clear layer region for costmap filters
  // as they are always supposed to be not clearable.
  // 过滤器层始终不可清除，无需处理
}

// ============================================================================
// clearLayerRegion()：对单个层执行区域清除
// 计算世界坐标矩形 → 钳位到地图内 → clearArea → 标记整层待重绘
// ============================================================================

void ClearCostmapService::clearLayerRegion(
  shared_ptr<CostmapLayer> & costmap, double pose_x, double pose_y, double reset_distance,
  bool invert)
{
  // 加层锁，与该层的更新线程互斥
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // 以机器人为中心的正方形（世界坐标）
  double start_point_x = pose_x - reset_distance / 2;
  double start_point_y = pose_y - reset_distance / 2;
  double end_point_x = start_point_x + reset_distance;
  double end_point_y = start_point_y + reset_distance;

  // 世界坐标 → 栅格坐标（钳位到地图边界内）
  int start_x, start_y, end_x, end_y;
  costmap->worldToMapEnforceBounds(start_point_x, start_point_y, start_x, start_y);
  costmap->worldToMapEnforceBounds(end_point_x, end_point_y, end_x, end_y);

  // 执行区域清除（置为 NO_INFORMATION，invert 控制清内/清外）
  costmap->clearArea(start_x, start_y, end_x, end_y, invert);

  // 把整个层范围标记为额外更新边界——
  // 下一周期该层会把数据重新写入主地图（清除才会生效于主地图）
  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
}

// ============================================================================
// clearEntirely()：整图复位——调用 Costmap2DROS::resetLayers
// （各层执行各自的 reset()：障碍层清空观测与地图，静态层重发数据等）
// ============================================================================

void ClearCostmapService::clearEntirely()
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getCostmap()->getMutex()));
  costmap_.resetLayers();
}

// getPosition()：获取机器人当前位置（通过 Costmap2DROS 的 TF 查询）
bool ClearCostmapService::getPosition(double & x, double & y) const
{
  geometry_msgs::msg::PoseStamped pose;
  if (!costmap_.getRobotPose(pose)) {
    return false;
  }

  x = pose.pose.position.x;
  y = pose.pose.position.y;

  return true;
}

}  // namespace nav2_costmap_2d
