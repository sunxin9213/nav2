// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Steven Macenski
// Copyright (c) 2019 Samsung Research America
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
// 【文件概况】机器人位姿与 TF 变换工具
// ----------------------------------------------------------------------------
// 概述: 封装 tf2_ros::Buffer 的常用查询操作，统一异常捕获与日志输出，
//       全部以 bool 返回值代替抛异常，方便调用方做错误处理。
// 主要内容:
//   1. getCurrentPose():             取机器人当前位姿（base_link→map）
//   2. transformPoseInTargetFrame(): 任意位姿变换到目标坐标系
//   3. getTransform() ×2:            取坐标系间变换（最新版/跨时刻桥接版）
//   4. validateTwist():              校验 Twist 六个分量无 NaN/Inf
// 使用场景: 所有需要查询机器人位姿的服务器（controller/planner/behaviors）
//           与传感器数据源坐标系转换（collision monitor 等）。
// ============================================================================

#ifndef NAV2_UTIL__ROBOT_UTILS_HPP_
#define NAV2_UTIL__ROBOT_UTILS_HPP_

#include <string>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_util
{
// ==================== 机器人位姿与 TF 工具函数 ====================
// 以下函数封装了 tf2_ros::Buffer 的常用操作，统一了异常处理与日志输出。

/**
* @brief get the current pose of the robot
* @param global_pose Pose to transform
* @param tf_buffer TF buffer to use for the transformation
* @param global_frame Frame to transform into
* @param robot_frame Frame to transform from
* @param transform_timeout TF Timeout to use for transformation
* @return bool Whether it could be transformed successfully
*/
// 获取机器人当前位姿：把 robot_frame(默认 base_link) 中的单位位姿
// 变换到 global_frame(默认 map) 中，超时默认 0.1s
bool getCurrentPose(
  geometry_msgs::msg::PoseStamped & global_pose,
  tf2_ros::Buffer & tf_buffer, const std::string global_frame = "map",
  const std::string robot_frame = "base_link", const double transform_timeout = 0.1,
  const rclcpp::Time stamp = rclcpp::Time());

/**
* @brief get an arbitrary pose in a target frame
* @param input_pose Pose to transform
* @param transformed_pose Output transformation
* @param tf_buffer TF buffer to use for the transformation
* @param target_frame Frame to transform into
* @param transform_timeout TF Timeout to use for transformation
* @return bool Whether it could be transformed successfully
*/
// 把任意位姿从其自身坐标系变换到目标坐标系，失败时返回 false
bool transformPoseInTargetFrame(
  const geometry_msgs::msg::PoseStamped & input_pose,
  geometry_msgs::msg::PoseStamped & transformed_pose,
  tf2_ros::Buffer & tf_buffer, const std::string target_frame,
  const double transform_timeout = 0.1);

/**
 * @brief Obtains a transform from source_frame_id at source_time ->
 * to target_frame_id at target_time time
 * @param source_frame_id Source frame ID to convert from
 * @param source_time Source timestamp to convert from
 * @param target_frame_id Target frame ID to convert to
 * @param target_time Target time to interpolate to
 * @param transform_tolerance Transform tolerance
 * @param tf_transform Output source->target transform
 * @return True if got correct transform, otherwise false
 */

/**
 * @brief Obtains a transform from source_frame_id -> to target_frame_id
 * @param source_frame_id Source frame ID to convert from
 * @param target_frame_id Target frame ID to convert to
 * @param transform_tolerance Transform tolerance
 * @param tf_buffer TF buffer to use for the transformation
 * @param tf_transform Output source->target transform
 * @return True if got correct transform, otherwise false
 */
// 获取 source->target 的最新 TF 变换（简单版本，不带时间参数）
bool getTransform(
  const std::string & source_frame_id,
  const std::string & target_frame_id,
  const tf2::Duration & transform_tolerance,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  tf2::Transform & tf2_transform);

/**
 * @brief Obtains a transform from source_frame_id at source_time ->
 * to target_frame_id at target_time time
 * @param source_frame_id Source frame ID to convert from
 * @param source_time Source timestamp to convert from
 * @param target_frame_id Target frame ID to convert to
 * @param target_time Current node time to interpolate to
 * @param fixed_frame_id The frame in which to assume the transform is constant in time
 * @param transform_tolerance Transform tolerance
 * @param tf_buffer TF buffer to use for the transformation
 * @param tf_transform Output source->target transform
 * @return True if got correct transform, otherwise false
 */
// 高级版本：源/目标可指定不同时间戳，经 fixed_frame 桥接完成跨时刻变换
bool getTransform(
  const std::string & source_frame_id,
  const rclcpp::Time & source_time,
  const std::string & target_frame_id,
  const rclcpp::Time & target_time,
  const std::string & fixed_frame_id,
  const tf2::Duration & transform_tolerance,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  tf2::Transform & tf2_transform);

/**
 * @brief Validates a twist message contains no nans or infs
 * @param msg Twist message to validate
 * @return True if valid, false if contains unactionable values
 */
// 校验 Twist 速度消息六个分量均无 NaN/Inf
bool validateTwist(const geometry_msgs::msg::Twist & msg);

}  // end namespace nav2_util

#endif  // NAV2_UTIL__ROBOT_UTILS_HPP_
