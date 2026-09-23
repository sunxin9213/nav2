// Copyright (c) 2022 Samsung R&D Institute Russia
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
// 文件说明（中文）：
// PointCloud（点云数据源）类的实现。
// 模块定位：nav2_collision_monitor 的数据源之一，处理
// sensor_msgs/PointCloud2 消息（3D 深度相机/3D 激光雷达）。
// getData 遍历点云中的每个三维点，经 TF 变换到本体坐标系后，
// 只保留高度落在 [min_height, max_height] 区间内的点（过滤地面
// 与过高障碍，只保留可能撞上机器人的层），投影为 2D 障碍物点。
// ============================================================================

#include "nav2_collision_monitor/pointcloud.hpp"

#include <functional>

#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

// 构造：委托基类 Source 初始化，数据缓存置空
PointCloud::PointCloud(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id,
    transform_tolerance, source_timeout, base_shift_correction),
  data_(nullptr)
{
  RCLCPP_INFO(logger_, "[%s]: Creating PointCloud", source_name_.c_str());
}

// 析构：释放订阅
PointCloud::~PointCloud()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying PointCloud", source_name_.c_str());
  data_sub_.reset();
}

// ---------------------------------------------------------------------------
// 配置：基类配置 + 读取参数（含高度过滤范围）+ 创建
// PointCloud2 订阅（SensorDataQoS）。
// ---------------------------------------------------------------------------
void PointCloud::configure()
{
  Source::configure();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  getParameters(source_topic);

  rclcpp::QoS pointcloud_qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    source_topic, pointcloud_qos,
    std::bind(&PointCloud::dataCallback, this, std::placeholders::_1));
}

// ---------------------------------------------------------------------------
// 获取障碍物观测点（输出到 data，本体坐标系下）：
//   1. 有效性检查：无数据 / 数据超时则返回空；
//   2. TF 变换（base_shift_correction 决定是否考虑时间差，
//      与 Scan/Range 中相同）；
//   3. 用 PointCloud2Iterator 逐点读取 x/y/z，变换到本体坐标系后
//      做高度过滤（min_height <= z <= max_height），
//      通过的点投影为 2D 障碍物点加入 data。
// ---------------------------------------------------------------------------
void PointCloud::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data) const
{
  // Ignore data from the source if it is not being published yet or
  // not published for a long time
  // 无数据或数据过期：返回空
  if (data_ == nullptr) {
    return;
  }
  if (!sourceValid(data_->header.stamp, curr_time)) {
    return;
  }

  tf2::Transform tf_transform;
  if (base_shift_correction_) {
    // Obtaining the transform to get data from source frame and time where it was received
    // to the base frame and current time
    // 完整时空变换：数据帧+数据时刻 -> 本体帧+当前时刻（经 global_frame）
    if (
      !nav2_util::getTransform(
        data_->header.frame_id, data_->header.stamp,
        base_frame_id_, curr_time, global_frame_id_,
        transform_tolerance_, tf_buffer_, tf_transform))
    {
      return;
    }
  } else {
    // Obtaining the transform to get data from source frame to base frame without time shift
    // considered. Less accurate but much more faster option not dependent on state estimation
    // frames.
    // 简化变换：仅坐标系变换，不考虑时间差（快但不精确）
    if (
      !nav2_util::getTransform(
        data_->header.frame_id, base_frame_id_,
        transform_tolerance_, tf_buffer_, tf_transform))
    {
      return;
    }
  }

  // 点云迭代器：按字段名访问 x/y/z（兼容任意点云布局）
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*data_, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*data_, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*data_, "z");

  // Refill data array with PointCloud points in base frame
  // 遍历所有点：变换到本体坐标系后按高度过滤（滤除地面点与高空点）
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // Transform point coordinates from source frame -> to base frame
    tf2::Vector3 p_v3_s(*iter_x, *iter_y, *iter_z);
    tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

    // Refill data array
    // 只保留高度在 [min_height, max_height] 内的点（2D 投影）
    if (p_v3_b.z() >= min_height_ && p_v3_b.z() <= max_height_) {
      data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
  }
}

// ---------------------------------------------------------------------------
// 读取参数：公共参数 + 专属高度过滤参数：
//   min_height：最小高度（默认 0.05m，滤除地面）
//   max_height：最大高度（默认 0.5m，滤除高于机器人本体的点）
// ---------------------------------------------------------------------------
void PointCloud::getParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  getCommonParameters(source_topic);

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".min_height", rclcpp::ParameterValue(0.05));
  min_height_ = node->get_parameter(source_name_ + ".min_height").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".max_height", rclcpp::ParameterValue(0.5));
  max_height_ = node->get_parameter(source_name_ + ".max_height").as_double();
}

// 订阅回调：缓存最新一帧 PointCloud2 消息
void PointCloud::dataCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  data_ = msg;
}

}  // namespace nav2_collision_monitor
