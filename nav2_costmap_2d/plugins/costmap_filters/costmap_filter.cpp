/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *********************************************************************/
// ============================================================================
// 中文注释：代价地图过滤器基类（CostmapFilter）实现文件
//
// 【文件功能】
// 本文件实现 Nav2 "代价地图过滤器（Costmap Filters）"体系的公共基类。
// 过滤器是一类特殊的 Layer：它们在地图数据流上叠加"掩码（mask）"实现
// 语义级约束，如禁行区（KeepoutFilter）、限速区（SpeedFilter）、
// 二值通行/禁行（BinaryFilter）。基类统一处理：
//   - 公共参数（enabled / filter_info_topic / transform_tolerance）；
//   - 启停服务（std_srvs/SetBool 的 <层名>/toggle_filter）；
//   - 机器人位姿的记录（updateBounds 阶段存下 latest_pose_）；
//   - 坐标变换与"世界坐标→掩码栅格坐标"换算的公共工具。
//
// 【在 Nav2 中的位置】
// 继承链：Layer → CostmapFilter → KeepoutFilter/SpeedFilter/BinaryFilter。
// 过滤器与 CostmapFilterInfo 消息发布器（如 keepout_filter_node、
// speed_filter_node，位于 nav2_map_server 包）配套使用：
//   1) 过滤器订阅 filter_info_topic 获得 CostmapFilterInfo（含掩码话题名、
//      base/multiplier 等解释参数）；
//   2) 再订阅掩码话题（OccupancyGrid 地图）；
//   3) 在 updateCosts（process()）中按掩码修改主地图或速度限制。
//
// 【生命周期语义】
// on_configure → onInitialize()；on_activate → activate()→initializeFilter()；
// on_deactivate → deactivate()→resetFilter()。即：激活时才建立订阅。
// ============================================================================

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"

#include <exception>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace nav2_costmap_2d
{

// 构造：初始化话题名为空，并创建保护本层的互斥锁
CostmapFilter::CostmapFilter()
: filter_info_topic_(""), mask_topic_("")
{
  access_ = new mutex_t();
}

// 析构：释放互斥锁
CostmapFilter::~CostmapFilter()
{
  delete access_;
}

// ============================================================================
// onInitialize()：声明并读取所有过滤器共有的参数，创建启停服务
// ============================================================================
void CostmapFilter::onInitialize()
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  try {
    // Declare common for all costmap filters parameters
    // 过滤器公共参数：启用开关、过滤器信息话题名、TF 容差
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("filter_info_topic", rclcpp::PARAMETER_STRING);
    declareParameter("transform_tolerance", rclcpp::ParameterValue(0.1));

    // Get parameters
    // 读取参数（层私有命名：<层名>.<参数名>）
    node->get_parameter(name_ + "." + "enabled", enabled_);
    filter_info_topic_ = node->get_parameter(name_ + "." + "filter_info_topic").as_string();
    double transform_tolerance {};
    node->get_parameter(name_ + "." + "transform_tolerance", transform_tolerance);
    transform_tolerance_ = tf2::durationFromSec(transform_tolerance);

    // Costmap Filter enabling service
    // 创建启停服务：std_srvs/SetBool，服务名 "<层名>/toggle_filter"，
    // 运行时可远程开关过滤器而无需改参数重启
    enable_service_ = node->create_service<std_srvs::srv::SetBool>(
      name_ + "/toggle_filter",
      std::bind(
        &CostmapFilter::enableCallback, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger_, "Parameter problem: %s", ex.what());
    throw ex;
  }
}

// activate()：生命周期激活——调用子类的 initializeFilter() 建立订阅
void CostmapFilter::activate()
{
  initializeFilter(filter_info_topic_);
}

// deactivate()：生命周期去激活——调用子类的 resetFilter() 断开订阅
void CostmapFilter::deactivate()
{
  resetFilter();
}

// reset()：复位——先重置再重新初始化（订阅会重建）
void CostmapFilter::reset()
{
  resetFilter();
  initializeFilter(filter_info_topic_);
  current_ = false;
}

// ============================================================================
// updateBounds()：分层更新第一阶段——过滤器不扩展更新边界，
// 只记录机器人最新位姿（供 SpeedFilter 等按位置查掩码用）
// ============================================================================
void CostmapFilter::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * /*min_x*/, double * /*min_y*/, double * /*max_x*/, double * /*max_y*/)
{
  if (!enabled_) {
    return;
  }

  // 保存本周期的机器人位姿（x, y, theta）
  latest_pose_.x = robot_x;
  latest_pose_.y = robot_y;
  latest_pose_.theta = robot_yaw;
}

// ============================================================================
// updateCosts()：分层更新第二阶段——委托给子类实现的 process()
// 对主地图或速度限制施加掩码效果
// ============================================================================
void CostmapFilter::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  process(master_grid, min_i, min_j, max_i, max_j, latest_pose_);
  current_ = true;
}

// ============================================================================
// enableCallback()：启停服务回调（std_srvs/SetBool）
// data=true 启用过滤器，data=false 禁用
// ============================================================================
void CostmapFilter::enableCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  enabled_ = request->data;
  response->success = true;
  if (enabled_) {
    response->message = "Enabled";
  } else {
    response->message = "Disabled";
  }
}

// ============================================================================
// transformPose()：把全局坐标系下的位姿变换到掩码坐标系
// 掩码地图与主地图可能处于不同坐标系（如 mask 在 "map"，主地图在 "odom"），
// 此时用 TF2 把位置点变换过去；同坐标系则直接复制
// ============================================================================
bool CostmapFilter::transformPose(
  const std::string global_frame,
  const geometry_msgs::msg::Pose2D & global_pose,
  const std::string mask_frame,
  geometry_msgs::msg::Pose2D & mask_pose) const
{
  if (mask_frame != global_frame) {
    // Filter mask and current layer are in different frames:
    // Transform (global_pose.x, global_pose.y) point from current layer frame (global_frame)
    // to mask_pose point in mask_frame
    // 坐标系不同：把全局坐标点包装成 PointStamped 做 TF 变换
    geometry_msgs::msg::TransformStamped transform;
    geometry_msgs::msg::PointStamped in, out;
    in.header.stamp = clock_->now();
    in.header.frame_id = global_frame;
    in.point.x = global_pose.x;
    in.point.y = global_pose.y;
    in.point.z = 0;

    try {
      tf_->transform(in, out, mask_frame, transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        logger_,
        "CostmapFilter: failed to get costmap frame (%s) "
        "transformation to mask frame (%s) with error: %s",
        global_frame.c_str(), mask_frame.c_str(), ex.what());
      return false;
    }
    mask_pose.x = out.point.x;
    mask_pose.y = out.point.y;
  } else {
    // Filter mask and current layer are in the same frame:
    // Just use global_pose coordinates
    // 坐标系相同：直接使用全局坐标
    mask_pose = global_pose;
  }

  return true;
}

// ============================================================================
// worldToMask()：世界坐标 → 掩码栅格坐标（带边界检查）
// 与 Costmap2D::worldToMap 思路一致：
//   mx = (wx - origin_x) / resolution，my = (wy - origin_y) / resolution
// 超出掩码范围返回 false
// ============================================================================
bool CostmapFilter::worldToMask(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr filter_mask,
  double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  // 取掩码的几何信息：原点、分辨率、尺寸
  const double origin_x = filter_mask->info.origin.position.x;
  const double origin_y = filter_mask->info.origin.position.y;
  const double resolution = filter_mask->info.resolution;
  const unsigned int size_x = filter_mask->info.width;
  const unsigned int size_y = filter_mask->info.height;

  // 在掩码原点左/下方：越界
  if (wx < origin_x || wy < origin_y) {
    return false;
  }

  // 世界坐标 → 栅格坐标（减原点、除分辨率）
  mx = static_cast<unsigned int>((wx - origin_x) / resolution);
  my = static_cast<unsigned int>((wy - origin_y) / resolution);
  if (mx >= size_x || my >= size_y) {
    return false;
  }

  return true;
}

}  // namespace nav2_costmap_2d
