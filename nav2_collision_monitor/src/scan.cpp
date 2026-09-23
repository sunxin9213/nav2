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
// Scan（激光扫描数据源）类的实现。
// 模块定位：nav2_collision_monitor 的数据源之一，处理
// sensor_msgs/LaserScan 消息（2D 激光雷达）。
// getData 把每个有效距离测量（range_min <= r <= range_max）
// 从极坐标（角度 = angle_min + i*angle_increment）转为直角坐标点，
// 并经 TF 变换到机器人本体坐标系，供安全区碰撞检测使用。
// ============================================================================

#include "nav2_collision_monitor/scan.hpp"

#include <cmath>
#include <functional>

namespace nav2_collision_monitor
{

// 构造：委托基类 Source 初始化，数据缓存置空
Scan::Scan(
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
  RCLCPP_INFO(logger_, "[%s]: Creating Scan", source_name_.c_str());
}

// 析构：释放订阅
Scan::~Scan()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Scan", source_name_.c_str());
  data_sub_.reset();
}

// ---------------------------------------------------------------------------
// 配置：基类配置 + 读取公共参数（激光雷达无专属参数）+ 创建
// LaserScan 订阅（SensorDataQoS）。
// ---------------------------------------------------------------------------
void Scan::configure()
{
  Source::configure();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  // Laser scanner has no own parameters
  getCommonParameters(source_topic);

  rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    source_topic, scan_qos,
    std::bind(&Scan::dataCallback, this, std::placeholders::_1));
}

// ---------------------------------------------------------------------------
// 获取障碍物观测点（输出到 data，本体坐标系下）：
//   1. 有效性检查：无数据 / 数据超时则返回空；
//   2. TF 变换（base_shift_correction 决定是否考虑时间差/机器人运动，
//      与 Range::getData 中相同）；
//   3. 遍历所有距离测量：量程内的测量按
//      angle = angle_min + i * angle_increment 转为直角坐标点，
//      变换到本体坐标系后加入 data。
// ---------------------------------------------------------------------------
void Scan::getData(
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

  // Calculate poses and refill data array
  // 遍历激光束：有效距离按极坐标转直角坐标并变换到本体坐标系
  float angle = data_->angle_min;
  for (size_t i = 0; i < data_->ranges.size(); i++) {
    if (data_->ranges[i] >= data_->range_min && data_->ranges[i] <= data_->range_max) {
      // Transform point coordinates from source frame -> to base frame
      tf2::Vector3 p_v3_s(
        data_->ranges[i] * std::cos(angle),
        data_->ranges[i] * std::sin(angle),
        0.0);
      tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

      // Refill data array
      data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
    angle += data_->angle_increment;  // 相邻激光束的角增量
  }
}

// 订阅回调：缓存最新一帧 LaserScan 消息
void Scan::dataCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  data_ = msg;
}

}  // namespace nav2_collision_monitor
