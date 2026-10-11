// Copyright (c) 2024 GoesM
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
// 【文件概况】ROS 消息合法性校验函数族（header-only 重载集）
// ----------------------------------------------------------------------------
// 概述: 为 nav2 常用消息类型提供 validateMsg() 重载，统一拦截畸形消息:
//   1) 数值检查: 拒绝 NaN/Inf 等损坏值
//   2) 逻辑检查: 如 OccupancyGrid 的 data 长度必须等于 width*height
//   3) 递归结构: 含子消息时先递归校验子消息，再校验消息自身
// 主要内容:
//   - 基础类型: double / double 数组(N) / Time / Header
//   - 几何类型: Point / Quaternion(含模长=1检查) / Pose /
//     PoseWithCovariance / PoseWithCovarianceStamped
//   - 地图类型: MapMetaData(尺寸非 0) / OccupancyGrid(data 长度一致)
// 使用场景: 接收外部输入（如 RViz 下发的位姿、加载的地图）时先校验再使用:
//           if (!validateMsg(msg)) { /* 拒绝并记录日志 */ }
// ============================================================================

#ifndef  NAV2_UTIL__VALIDATE_MESSAGES_HPP_
#define  NAV2_UTIL__VALIDATE_MESSAGES_HPP_

#include <cmath>
#include <iostream>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


// @brief Validation Check
//  Check recieved message is safe or not for the nav2-system
//  For each msg-type known in nav2, we could check it as following:
//  if(!validateMsg()) RCLCPP_ERROR(,"malformed msg. Rejecting.")
//
//  Workflow of validateMsg():
//     if here's a sub-msg-type in the recieved msg,
//        the content of sub-msg would be checked as sub-msg-type
//     then, check the whole recieved msg.
//
//  Following conditions are involved in check:
//     1> Value Check: to avoid damaged value like like `nan`, `INF`, empty string and so on
//     2> Logic Check: to avoid value with bad logic,
//             like the size of `map` should be equal to `height*width`
//     3> Any other needed condition could be joint here in future

namespace nav2_util
{

// 基础重载：校验单个 double 值无 NaN/Inf
bool validateMsg(const double & num)
{
  /*  @brief double/float value check
   *  if here'a need to check message validation
   *  it should be avoid to use double value like `nan`, `inf`
   *  otherwise, we regard it as an invalid message
   */
  if (std::isinf(num)) {return false;}
  if (std::isnan(num)) {return false;}
  return true;
}

// 数组版本：逐元素递归校验（如协方差矩阵 covariance 字段）
template<size_t N>
bool validateMsg(const std::array<double, N> & msg)
{
  /*  @brief value check for double-array
   *     like the field `covariance` used in the msg-type:
   *       geometry_msgs::msg::PoseWithCovarianceStamped
   */
  for (const auto & element : msg) {
    if (!validateMsg(element)) {return false;}
  }

  return true;
}

const int NSEC_PER_SEC = 1e9;  // 1 second = 1e9 nanosecond
// 时间戳版本：纳秒字段必须小于 1e9（否则不是一个合法时刻）
bool validateMsg(const builtin_interfaces::msg::Time & msg)
{
  if (msg.nanosec >= NSEC_PER_SEC) {
    return false;                                      // invalid nanosec-stamp
  }
  return true;
}

// Header 版本：先校验子字段 stamp，再要求 frame_id 非空
bool validateMsg(const std_msgs::msg::Header & msg)
{
  //  check sub-type
  if (!validateMsg(msg.stamp)) {return false;}

  /*  @brief frame_id check
   *  if here'a need to check message validation
   *  it should at least have a non-empty frame_id
   *  otherwise, we regard it as an invalid message
   */
  if (msg.frame_id.empty()) {return false;}
  return true;
}

// Point 版本：三分量逐一做数值校验
bool validateMsg(const geometry_msgs::msg::Point & msg)
{
  //  check sub-type
  if (!validateMsg(msg.x)) {return false;}
  if (!validateMsg(msg.y)) {return false;}
  if (!validateMsg(msg.z)) {return false;}
  return true;
}

const double epsilon = 1e-4;
// 四元数版本：除分量数值校验外，还检查模长近似为 1（单位四元数）
bool validateMsg(const geometry_msgs::msg::Quaternion & msg)
{
  //  check sub-type
  if (!validateMsg(msg.x)) {return false;}
  if (!validateMsg(msg.y)) {return false;}
  if (!validateMsg(msg.z)) {return false;}
  if (!validateMsg(msg.w)) {return false;}

  // x^2+y^2+z^2+w^2 应等于 1，允许 1e-4 的误差
  if (abs(msg.x * msg.x + msg.y * msg.y + msg.z * msg.z + msg.w * msg.w - 1.0) >= epsilon) {
    return false;
  }

  return true;
}

// Pose 版本：位置 + 姿态分别校验
bool validateMsg(const geometry_msgs::msg::Pose & msg)
{
  // check sub-type
  if (!validateMsg(msg.position)) {return false;}
  if (!validateMsg(msg.orientation)) {return false;}
  return true;
}

// PoseWithCovariance 版本：位姿 + 36 元素协方差数组
bool validateMsg(const geometry_msgs::msg::PoseWithCovariance & msg)
{
  // check sub-type
  if (!validateMsg(msg.pose)) {return false;}
  if (!validateMsg(msg.covariance)) {return false;}

  return true;
}

// PoseWithCovarianceStamped 版本：消息头 + 带协方差的位姿
bool validateMsg(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  // check sub-type
  if (!validateMsg(msg.header)) {return false;}
  if (!validateMsg(msg.pose)) {return false;}
  return true;
}


// Function to verify map meta information
// 地图元信息版本：原点位姿、分辨率数值合法，且宽高不为 0
bool validateMsg(const nav_msgs::msg::MapMetaData & msg)
{
  // check sub-type
  if (!validateMsg(msg.origin)) {return false;}
  if (!validateMsg(msg.resolution)) {return false;}

  // logic check
  // 1> we don't need an empty map
  if (msg.height == 0 || msg.width == 0) {return false;}
  return true;
}

// for msg-type like map, costmap and others as `OccupancyGrid`
// 占据栅格版本：除子字段校验外，还检查数据长度 = 宽 * 高（逻辑一致性）
bool validateMsg(const nav_msgs::msg::OccupancyGrid & msg)
{
  // check sub-type
  if (!validateMsg(msg.header)) {return false;}
  // msg.data :  @todo any check for it ?
  if (!validateMsg(msg.info)) {return false;}

  // check logic
  if (msg.data.size() != msg.info.width * msg.info.height) {
    return false;                                                          // check map-size
  }
  return true;
}


}  //  namespace nav2_util


#endif  // NAV2_UTIL__VALIDATE_MESSAGES_HPP_
