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
// Range（测距传感器数据源）类的实现。
// 模块定位：nav2_collision_monitor 的数据源之一，处理
// sensor_msgs/Range 消息（红外/超声等单点测距传感器）。
// 由于 Range 传感器只给一个距离值与视场角（field_of_view），
// 无法确定障碍物的确切方位，本类采取保守策略：
// 在视场角范围内按 obstacles_angle 步长生成一圈"潜在障碍物点"，
// 全部位于测得的距离处，供安全区检测使用。
// ============================================================================

#include "nav2_collision_monitor/range.hpp"

#include <math.h>
#include <cmath>
#include <functional>

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

// 构造：委托基类 Source 初始化，数据缓存置空
Range::Range(
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
  RCLCPP_INFO(logger_, "[%s]: Creating Range", source_name_.c_str());
}

// 析构：释放订阅
Range::~Range()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Range", source_name_.c_str());
  data_sub_.reset();
}

// ---------------------------------------------------------------------------
// 配置：先做基类配置（动态参数回调），读取参数后创建
// sensor_msgs/Range 订阅（QoS 用 SensorDataQoS：best-effort、浅深度）。
// ---------------------------------------------------------------------------
void Range::configure()
{
  Source::configure();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  getParameters(source_topic);

  rclcpp::QoS range_qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<sensor_msgs::msg::Range>(
    source_topic, range_qos,
    std::bind(&Range::dataCallback, this, std::placeholders::_1));
}

// ---------------------------------------------------------------------------
// 获取障碍物观测点（输出到 data，本体坐标系下）：
//   1. 有效性检查：无数据 / 数据超时（sourceValid）直接返回空；
//   2. 距离有效性：range 超出 [min_range, max_range] 则忽略；
//   3. TF 变换（两种模式）：
//      - base_shift_correction=true：考虑机器人运动的完整变换
//        （数据帧+数据时刻 -> 本体帧+当前时刻，经 global_frame），
//        精确但依赖状态估计；
//      - false：仅做静态帧间变换，快但不补偿运动；
//   4. 在视场角 [-fov/2, +fov/2] 内按 obstacles_angle 步长采样，
//      生成位于测距值处的一圈点并变换到本体坐标系。
// ---------------------------------------------------------------------------
void Range::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data) const
{
  // Ignore data from the source if it is not being published yet or
  // not being published for a long time
  // 无数据或数据过期：返回空
  if (data_ == nullptr) {
    return;
  }
  if (!sourceValid(data_->header.stamp, curr_time)) {
    return;
  }

  // Ignore data, if its range is out of scope of range sensor abilities
  // 距离超出传感器量程：忽略本帧
  if (data_->range < data_->min_range || data_->range > data_->max_range) {
    RCLCPP_DEBUG(
      logger_,
      "[%s]: Data range %fm is out of {%f..%f} sensor span. Ignoring...",
      source_name_.c_str(), data_->range, data_->min_range, data_->max_range);
    return;
  }

  tf2::Transform tf_transform;
  if (base_shift_correction_) {
    // Obtaining the transform to get data from source frame and time where it was received
    // to the base frame and current time
    // 完整时空变换：数据坐标系+数据时间 -> 本体坐标系+当前时间
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
    // 简化变换：仅坐标系间变换，不考虑时间差（快但不精确）
    if (
      !nav2_util::getTransform(
        data_->header.frame_id, base_frame_id_,
        transform_tolerance_, tf_buffer_, tf_transform))
    {
      return;
    }
  }

  // Calculate poses and refill data array
  // 在视场角范围内按步长生成潜在障碍物点（极坐标 -> 直角坐标）
  float angle;
  for (
    angle = -data_->field_of_view / 2;
    angle < data_->field_of_view / 2;
    angle += obstacles_angle_)
  {
    // Transform point coordinates from source frame -> to base frame
    // 传感器坐标系下的点（距离 r，方位角 angle）
    tf2::Vector3 p_v3_s(
      data_->range * std::cos(angle),
      data_->range * std::sin(angle),
      0.0);
    // 变换到本体坐标系
    tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

    // Refill data array
    data.push_back({p_v3_b.x(), p_v3_b.y()});
  }

  // Make sure that last (field_of_view / 2) point will be in the data array
  // 补上视场角末端点（循环步进可能跳过边界角）
  angle = data_->field_of_view / 2;

  // Transform point coordinates from source frame -> to base frame
  tf2::Vector3 p_v3_s(
    data_->range * std::cos(angle),
    data_->range * std::sin(angle),
    0.0);
  tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

  // Refill data array
  data.push_back({p_v3_b.x(), p_v3_b.y()});
}

// ---------------------------------------------------------------------------
// 读取参数：公共参数（话题/使能）+ 专属参数 obstacles_angle
// （生成潜在障碍物点的角步长，默认 1 度）。
// ---------------------------------------------------------------------------
void Range::getParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  getCommonParameters(source_topic);

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".obstacles_angle", rclcpp::ParameterValue(M_PI / 180));
  obstacles_angle_ = node->get_parameter(source_name_ + ".obstacles_angle").as_double();
}

// 订阅回调：缓存最新一帧 Range 消息
void Range::dataCallback(sensor_msgs::msg::Range::ConstSharedPtr msg)
{
  data_ = msg;
}

}  // namespace nav2_collision_monitor
