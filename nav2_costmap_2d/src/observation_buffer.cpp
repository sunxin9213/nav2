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
 *********************************************************************/
// ============================================================================
// 中文注释：观测缓冲区（ObservationBuffer）实现文件
//
// 【文件功能】
// 本文件实现传感器观测的"预处理 + 时间窗缓存"组件。ObstacleLayer/
// VoxelLayer 为每个传感器源创建一个 ObservationBuffer，它负责：
//   1. 把点云（含传感器原点）用 TF2 变换到全局坐标系（如 "map"/"odom"）；
//   2. 按高度上下限过滤点（地面点/天花板点剔除）；
//   3. 把距离参数（标记/清除的 min/max range）附给每条观测；
//   4. 按保留时长 observation_keep_time 缓存多条历史观测
//      （0 表示只保留最新一条）；
//   5. isCurrent() 判断传感器是否按期望频率持续更新（失联检测）。
//
// 【在 Nav2 中的位置】
// ObstacleLayer::onInitialize 中为每个 observation_source 创建；
// 订阅回调（laserScanCallback/pointCloud2Callback）把点云喂给
// bufferCloud()；更新周期里 getMarkingObservations/getClearingObservations
// 通过 getObservations() 取出全部有效观测。
//
// 【核心概念】
// - Observation 结构：origin_（传感器原点，全局系）+ cloud_（过滤后的
//   全局系点云）+ 四个距离参数；
// - observation_list_ 使用 std::list 且新观测 push_front：
//   列表按时间从新到旧排列，purgeStaleObservations 从尾部裁剪即可；
// - tf2_buffer_.transform：TF2 把消息整体变换到目标坐标系，
//   tf_tolerance_ 容忍 TF 略微滞后。
// ============================================================================
#include "nav2_costmap_2d/observation_buffer.hpp"

#include <algorithm>
#include <list>
#include <string>
#include <vector>
#include <chrono>

#include "tf2/convert.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
using namespace std::chrono_literals;

namespace nav2_costmap_2d
{
// ============================================================================
// 构造函数：保存全部配置参数（距离/高度过滤、坐标系、TF 容差等）
// 并从节点取得时钟、logger，初始化"最后更新时间"
// ============================================================================

ObservationBuffer::ObservationBuffer(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  std::string topic_name,
  double observation_keep_time,
  double expected_update_rate,
  double min_obstacle_height, double max_obstacle_height, double obstacle_max_range,
  double obstacle_min_range,
  double raytrace_max_range, double raytrace_min_range, tf2_ros::Buffer & tf2_buffer,
  std::string global_frame,
  std::string sensor_frame,
  tf2::Duration tf_tolerance)
: tf2_buffer_(tf2_buffer),                                          // TF2 缓冲引用
  observation_keep_time_(rclcpp::Duration::from_seconds(observation_keep_time)),   // 观测保留时长
  expected_update_rate_(rclcpp::Duration::from_seconds(expected_update_rate)),     // 期望更新周期（判失联）
  global_frame_(global_frame),                                       // 目标全局坐标系
  sensor_frame_(sensor_frame),                                       // 传感器坐标系（可空=用消息头）
  topic_name_(topic_name),                                           // 传感器话题名（日志用）
  min_obstacle_height_(min_obstacle_height), max_obstacle_height_(max_obstacle_height),   // 高度过滤区间
  obstacle_max_range_(obstacle_max_range), obstacle_min_range_(obstacle_min_range),       // 标记距离区间
  raytrace_max_range_(raytrace_max_range), raytrace_min_range_(                         // 清除射线距离区间
    raytrace_min_range), tf_tolerance_(tf_tolerance)                                     // TF 查询容差
{
  auto node = parent.lock();
  clock_ = node->get_clock();      // 节点时钟（时间相关逻辑使用）
  logger_ = node->get_logger();    // 节点 logger
  last_updated_ = node->now();     // 初始化最后更新时间
}

ObservationBuffer::~ObservationBuffer()
{
}

// ============================================================================
// bufferCloud()：核心入口——处理一帧新点云
// 步骤：新建 Observation → 变换传感器原点 → 变换整片点云 →
// 高度过滤 → 裁剪过期观测
// ============================================================================
void ObservationBuffer::bufferCloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  geometry_msgs::msg::PointStamped global_origin;

  // create a new observation on the list to be populated
  // 在链表头新建一条空观测（新观测总在最前）
  observation_list_.push_front(Observation());

  // check whether the origin frame has been set explicitly
  // or whether we should get it from the cloud
  // 传感器坐标系：显式配置优先，否则用点云消息头的 frame_id
  std::string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

  try {
    // given these observations come from sensors...
    // we'll need to store the origin pt of the sensor
    // ---- 第一步：传感器原点变换到全局坐标系 ----
    // 构造传感器坐标系原点 (0,0,0) 的点
    geometry_msgs::msg::PointStamped local_origin;
    local_origin.header.stamp = cloud.header.stamp;
    local_origin.header.frame_id = origin_frame;
    local_origin.point.x = 0;
    local_origin.point.y = 0;
    local_origin.point.z = 0;
    // TF 变换到全局系（清除射线要从这里发射）
    tf2_buffer_.transform(local_origin, global_origin, global_frame_, tf_tolerance_);
    tf2::convert(global_origin.point, observation_list_.front().origin_);

    // make sure to pass on the raytrace/obstacle range
    // of the observation buffer to the observation
    // ---- 第二步：把距离参数随观测传递（使用处直接读取）----
    observation_list_.front().raytrace_max_range_ = raytrace_max_range_;
    observation_list_.front().raytrace_min_range_ = raytrace_min_range_;
    observation_list_.front().obstacle_max_range_ = obstacle_max_range_;
    observation_list_.front().obstacle_min_range_ = obstacle_min_range_;

    sensor_msgs::msg::PointCloud2 global_frame_cloud;

    // transform the point cloud
    // ---- 第三步：整片点云变换到全局坐标系 ----
    tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_, tf_tolerance_);
    global_frame_cloud.header.stamp = cloud.header.stamp;

    // now we need to remove observations from the cloud that are below
    // or above our height thresholds
    // ---- 第四步：高度过滤 ----
    // 复制变换后点云的元信息（结构/步长等）
    sensor_msgs::msg::PointCloud2 & observation_cloud = *(observation_list_.front().cloud_);
    observation_cloud.height = global_frame_cloud.height;
    observation_cloud.width = global_frame_cloud.width;
    observation_cloud.fields = global_frame_cloud.fields;
    observation_cloud.is_bigendian = global_frame_cloud.is_bigendian;
    observation_cloud.point_step = global_frame_cloud.point_step;
    observation_cloud.row_step = global_frame_cloud.row_step;
    observation_cloud.is_dense = global_frame_cloud.is_dense;

    unsigned int cloud_size = global_frame_cloud.height * global_frame_cloud.width;
    // PointCloud2Modifier：先按满尺寸分配，稍后再收缩到实际保留点数
    sensor_msgs::PointCloud2Modifier modifier(observation_cloud);
    modifier.resize(cloud_size);
    unsigned int point_count = 0;

    // copy over the points that are within our height bounds
    // 逐点检查 z：仅保留 [min_obstacle_height, max_obstacle_height] 内的点，
    // 按字节整块拷贝（point_step 字节为一个点）
    sensor_msgs::PointCloud2Iterator<float> iter_z(global_frame_cloud, "z");
    std::vector<unsigned char>::const_iterator iter_global = global_frame_cloud.data.begin(),
      iter_global_end = global_frame_cloud.data.end();
    std::vector<unsigned char>::iterator iter_obs = observation_cloud.data.begin();
    for (; iter_global != iter_global_end; ++iter_z, iter_global +=
      global_frame_cloud.point_step)
    {
      if ((*iter_z) <= max_obstacle_height_ &&
        (*iter_z) >= min_obstacle_height_)
      {
        std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs);
        iter_obs += global_frame_cloud.point_step;
        ++point_count;
      }
    }

    // resize the cloud for the number of legal points
    // 收缩数组到实际保留的点数，并补齐消息头
    modifier.resize(point_count);
    observation_cloud.header.stamp = cloud.header.stamp;
    observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
  } catch (tf2::TransformException & ex) {
    // if an exception occurs, we need to remove the empty observation from the list
    // TF 失败（例如时间戳太老）：撤销刚插入的空观测并报错
    observation_list_.pop_front();
    RCLCPP_ERROR(
      logger_,
      "TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s",
      sensor_frame_.c_str(),
      cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  // if the update was successful, we want to update the last updated time
  // 成功处理：刷新"最后更新时间"（供 isCurrent 判断失联）
  last_updated_ = clock_->now();

  // we'll also remove any stale observations from the list
  // 顺带裁剪过期观测，控制链表长度
  purgeStaleObservations();
}

// ============================================================================
// getObservations()：取出全部有效观测（先裁剪过期，再拷贝给调用方）
// returns a copy of the observations
// ============================================================================

void ObservationBuffer::getObservations(std::vector<Observation> & observations)
{
  // first... let's make sure that we don't have any stale observations
  // 先清理过期观测
  purgeStaleObservations();

  // now we'll just copy the observations for the caller
  // 把链表内容拷贝到输出 vector（调用方无需加锁遍历链表）
  std::list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
    observations.push_back(*obs_it);
  }
}

// ============================================================================
// purgeStaleObservations()：移除过期观测
// keep_time=0：只保留最新一条（清掉其余全部）；
// 否则从链表尾部（最旧）开始，删除所有超过保留时长的观测
// （列表按新→旧排列，一旦遇到未过期项即可停止）
// ============================================================================

void ObservationBuffer::purgeStaleObservations()
{
  if (!observation_list_.empty()) {
    std::list<Observation>::iterator obs_it = observation_list_.begin();
    // if we're keeping observations for no time... then we'll only keep one observation
    // 不保留历史：只留链表头（最新）一条
    if (observation_keep_time_ == rclcpp::Duration(0.0s)) {
      observation_list_.erase(++obs_it, observation_list_.end());
      return;
    }

    // otherwise... we'll have to loop through the observations to see which ones are stale
    // 按时间从新到旧遍历
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
      Observation & obs = *obs_it;
      // check if the observation is out of date... and if it is,
      // remove it and those that follow from the list
      // 当前观测过期：它以及之后（更旧）的全部删除
      if ((clock_->now() - obs.cloud_->header.stamp) >
        observation_keep_time_)
      {
        observation_list_.erase(obs_it, observation_list_.end());
        return;
      }
    }
  }
}

// ============================================================================
// isCurrent()：传感器是否仍在按期望频率更新（失联检测）
// expected_update_rate=0 表示不检测（恒为 current）；
// 距上次更新超过期望周期则报警并返回 false——
// LayeredCostmap 会因此把整张地图标记为"不最新"
// ============================================================================

bool ObservationBuffer::isCurrent() const
{
  if (expected_update_rate_ == rclcpp::Duration(0.0s)) {
    return true;   // 未配置期望频率：不做检测
  }

  // 距最后更新时间是否仍在期望周期内
  bool current = (clock_->now() - last_updated_) <=
    expected_update_rate_;
  if (!current) {
    RCLCPP_WARN(
      logger_,
      "The %s observation buffer has not been updated for %.2f seconds, "
      "and it should be updated every %.2f seconds.",
      topic_name_.c_str(),
      (clock_->now() - last_updated_).seconds(),
      expected_update_rate_.seconds());
  }
  return current;
}

// resetLastUpdated()：重置最后更新时间（层重新激活/复位时调用，
// 避免停机期间的时间差被误判为传感器失联）
void ObservationBuffer::resetLastUpdated()
{
  last_updated_ = clock_->now();
}
}  // namespace nav2_costmap_2d
