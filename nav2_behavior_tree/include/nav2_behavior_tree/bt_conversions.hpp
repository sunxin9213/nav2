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

// =============================================================================
// 【文件说明】bt_conversions.hpp
// 【所属模块】nav2_behavior_tree —— Nav2 的行为树核心模块
//
// 本文件为 BehaviorTree.CPP 框架扩展了"字符串 -> ROS2 消息类型"的转换模板
// 特化（convertFromString）。
//
// 【在 Nav2 导航系统中的位置和作用】
// 行为树 XML 中所有端口值都是字符串（如 "1.0;2.0;0.0"）。当端口声明的
// 类型是 geometry_msgs 的 Point/Quaternion/PoseStamped 或
// std::chrono::milliseconds 时，框架需要一个"从字符串解析出该类型"的函数。
// BehaviorTree.CPP 通过模板 convertFromString<T> 提供了内置类型（int、
// double、string 等）的解析；本文件对其做模板特化（template<> 全特化），
// 使上述 ROS2 类型也能直接在 XML 中以字符串形式书写。
//
// 【XML 使用示例】
//   <PoseStamped goal="1.0;2.0;0.0;0.0;0.0;0.7;0.7"/>   <!-- 9 个分号分隔字段 -->
//   <Point position="1.0;2.0;0.0"/>                     <!-- 3 个字段 -->
//   <RateController hz="1.0"/>                          <!-- 毫秒端口："100" -->
//
// 【注意】PoseStamped 有 9 个字段：时间戳(nanoseconds)、frame_id，再加
// 位置 3 个 + 四元数 4 个浮点数。
// =============================================================================

#ifndef NAV2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
#define NAV2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_

#include <string>

#include "rclcpp/time.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace BT
{

// The follow templates are required when using these types as parameters
// in our BT XML files. They parse the strings in the XML into their corresponding
// data type.
// （下面的模板特化用于把这些 ROS2 类型作为 XML 端口参数使用：
//  把 XML 中的字符串解析成对应的 C++ 数据类型。）

/**
 * @brief Parse XML string to geometry_msgs::msg::Point
 *        （把 XML 字符串解析为 Point：3 个分号分隔的浮点数 "x;y;z"）
 * @param key XML string
 *        （XML 中的原始字符串）
 * @return geometry_msgs::msg::Point
 */
template<>
inline geometry_msgs::msg::Point convertFromString(const StringView key)
{
  // three real numbers separated by semicolons
  // 按分号切分字符串；Point 必须恰好 3 个字段 (x, y, z)。
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3) {
    // 字段数不对：抛出异常，BehaviorTree.CPP 会把它包装成节点构造错误。
    throw std::runtime_error("invalid number of fields for point attribute)");
  } else {
    // 逐字段转换为 double 并填入消息。
    geometry_msgs::msg::Point position;
    position.x = BT::convertFromString<double>(parts[0]);
    position.y = BT::convertFromString<double>(parts[1]);
    position.z = BT::convertFromString<double>(parts[2]);
    return position;
  }
}

/**
 * @brief Parse XML string to geometry_msgs::msg::Quaternion
 *        （把 XML 字符串解析为 Quaternion：4 个分号分隔的浮点数 "x;y;z;w"）
 * @param key XML string
 * @return geometry_msgs::msg::Quaternion
 */
template<>
inline geometry_msgs::msg::Quaternion convertFromString(const StringView key)
{
  // four real numbers separated by semicolons
  // 按分号切分；Quaternion 必须恰好 4 个字段 (x, y, z, w)。
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 4) {
    // 字段数错误时抛异常。
    throw std::runtime_error("invalid number of fields for orientation attribute)");
  } else {
    // 逐字段转换并填充四元数。
    geometry_msgs::msg::Quaternion orientation;
    orientation.x = BT::convertFromString<double>(parts[0]);
    orientation.y = BT::convertFromString<double>(parts[1]);
    orientation.z = BT::convertFromString<double>(parts[2]);
    orientation.w = BT::convertFromString<double>(parts[3]);
    return orientation;
  }
}

/**
 * @brief Parse XML string to geometry_msgs::msg::PoseStamped
 *        （把 XML 字符串解析为 PoseStamped：9 个分号分隔字段
 *         "时间戳;frame_id;x;y;z;qx;qy;qz;qw"）
 * @param key XML string
 * @return geometry_msgs::msg::PoseStamped
 */
template<>
inline geometry_msgs::msg::PoseStamped convertFromString(const StringView key)
{
  // 7 real numbers separated by semicolons
  // 按分号切分；PoseStamped 共 9 个字段：
  // [0]=时间戳(纳秒整数) [1]=frame_id [2..4]=位置 x/y/z [5..8]=四元数 x/y/z/w。
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 9) {
    // 字段数错误时抛异常。
    throw std::runtime_error("invalid number of fields for PoseStamped attribute)");
  } else {
    // 依序解析并填充消息的各个字段。
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = rclcpp::Time(BT::convertFromString<int64_t>(parts[0]));
    pose_stamped.header.frame_id = BT::convertFromString<std::string>(parts[1]);
    pose_stamped.pose.position.x = BT::convertFromString<double>(parts[2]);
    pose_stamped.pose.position.y = BT::convertFromString<double>(parts[3]);
    pose_stamped.pose.position.z = BT::convertFromString<double>(parts[4]);
    pose_stamped.pose.orientation.x = BT::convertFromString<double>(parts[5]);
    pose_stamped.pose.orientation.y = BT::convertFromString<double>(parts[6]);
    pose_stamped.pose.orientation.z = BT::convertFromString<double>(parts[7]);
    pose_stamped.pose.orientation.w = BT::convertFromString<double>(parts[8]);
    return pose_stamped;
  }
}

/**
 * @brief Parse XML string to std::chrono::milliseconds
 *        （把 XML 字符串解析为毫秒时长：一个无符号整数，如 "500"）
 * @param key XML string
 * @return std::chrono::milliseconds
 */
template<>
inline std::chrono::milliseconds convertFromString<std::chrono::milliseconds>(const StringView key)
{
  // 直接把字符串转成无符号长整数并包成毫秒时长
  // （XML 中写 <节点 server_timeout="20"/> 即 20ms）。
  return std::chrono::milliseconds(std::stoul(key.data()));
}

}  // namespace BT

#endif  // NAV2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
