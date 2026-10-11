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
// 【文件概况】机器人位姿与 TF 工具函数的实现
// ----------------------------------------------------------------------------
// 概述: 见 robot_utils.hpp。本文件实现:
//   1. getCurrentPose(): 构造机器人系单位位姿 → 复用
//      transformPoseInTargetFrame 原地变换得到全局位姿
//   2. transformPoseInTargetFrame(): tf_buffer.transform + 五类 TF 异常
//      （Lookup/Connectivity/Extrapolation/Timeout/Transform）的分类捕获
//   3. getTransform() ×2: 最新变换查询 / 跨时刻（经 fixed_frame 桥接）查询，
//      结果由 TransformStamped 转为 tf2::Transform 数学类型
//   4. validateTwist(): 六个分量逐一做 NaN/Inf 检查
// ============================================================================

#include <string>
#include <cmath>
#include <memory>

#include "nav2_util/robot_utils.hpp"
#include "rclcpp/logger.hpp"

namespace nav2_util
{

bool getCurrentPose(
  geometry_msgs::msg::PoseStamped & global_pose,
  tf2_ros::Buffer & tf_buffer, const std::string global_frame,
  const std::string robot_frame, const double transform_timeout,
  const rclcpp::Time stamp)
{
  // 技巧：先在机器人坐标系里构造"单位位姿"（原点、无旋转），
  // 再把它变换到全局坐标系，变换结果恰好就是机器人在全局系下的位姿
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  global_pose.header.frame_id = robot_frame;
  global_pose.header.stamp = stamp;

  // 输入输出复用同一个变量，原地完成坐标变换
  return transformPoseInTargetFrame(
    global_pose, global_pose, tf_buffer, global_frame, transform_timeout);
}

bool transformPoseInTargetFrame(
  const geometry_msgs::msg::PoseStamped & input_pose,
  geometry_msgs::msg::PoseStamped & transformed_pose,
  tf2_ros::Buffer & tf_buffer, const std::string target_frame,
  const double transform_timeout)
{
  // 静态 logger：避免每次调用都重新构造日志对象
  static rclcpp::Logger logger = rclcpp::get_logger("transformPoseInTargetFrame");

  try {
    // tf_buffer.transform 内部完成查询与坐标变换，超时由 transform_timeout 决定
    transformed_pose = tf_buffer.transform(
      input_pose, target_frame,
      tf2::durationFromSec(transform_timeout));
    return true;
  } catch (tf2::LookupException & ex) {
    // 目标坐标系不存在（尚未注册到 TF 树）
    RCLCPP_ERROR(
      logger,
      "No Transform available Error looking up target frame: %s\n", ex.what());
  } catch (tf2::ConnectivityException & ex) {
    // 两个坐标系之间在 TF 树上不连通
    RCLCPP_ERROR(
      logger,
      "Connectivity Error looking up target frame: %s\n", ex.what());
  } catch (tf2::ExtrapolationException & ex) {
    // 请求的时间点超出 TF 缓冲可插值范围
    RCLCPP_ERROR(
      logger,
      "Extrapolation Error looking up target frame: %s\n", ex.what());
  } catch (tf2::TimeoutException & ex) {
    // 等待变换超时
    RCLCPP_ERROR(
      logger,
      "Transform timeout with tolerance: %.4f", transform_timeout);
  } catch (tf2::TransformException & ex) {
    // 其他 TF 异常的兜底捕获
    RCLCPP_ERROR(
      logger, "Failed to transform from %s to %s",
      input_pose.header.frame_id.c_str(), target_frame.c_str());
  }

  return false;
}

bool getTransform(
  const std::string & source_frame_id,
  const std::string & target_frame_id,
  const tf2::Duration & transform_tolerance,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  tf2::Transform & tf2_transform)
{
  geometry_msgs::msg::TransformStamped transform;
  tf2_transform.setIdentity();  // initialize by identical transform

  if (source_frame_id == target_frame_id) {
    // We are already in required frame
    // 源与目标坐标系相同：无需变换，直接返回单位变换
    return true;
  }

  try {
    // Obtaining the transform to get data from source to target frame
    // 查询最新可用的变换（TimePointZero 表示最新），target 在前表示求 target<-source
    transform = tf_buffer->lookupTransform(
      target_frame_id, source_frame_id,
      tf2::TimePointZero, transform_tolerance);
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("getTransform"),
      "Failed to get \"%s\"->\"%s\" frame transform: %s",
      source_frame_id.c_str(), target_frame_id.c_str(), e.what());
    return false;
  }

  // Convert TransformStamped to TF2 transform
  // 把 ROS 消息类型转换为 tf2 内部数学类型，便于后续直接做矩阵运算
  tf2::fromMsg(transform.transform, tf2_transform);
  return true;
}

bool getTransform(
  const std::string & source_frame_id,
  const rclcpp::Time & source_time,
  const std::string & target_frame_id,
  const rclcpp::Time & target_time,
  const std::string & fixed_frame_id,
  const tf2::Duration & transform_tolerance,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  tf2::Transform & tf2_transform)
{
  geometry_msgs::msg::TransformStamped transform;
  tf2_transform.setIdentity();  // initialize by identical transform

  try {
    // Obtaining the transform to get data from source to target frame.
    // This also considers the time shift between source and target.
    // 高级重载：源和目标可以处于不同时刻，
    // 通过 fixed_frame（变换在此系内假设时不变）桥接两段时间戳的变换
    transform = tf_buffer->lookupTransform(
      target_frame_id, target_time,
      source_frame_id, source_time,
      fixed_frame_id, transform_tolerance);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("getTransform"),
      "Failed to get \"%s\"->\"%s\" frame transform: %s",
      source_frame_id.c_str(), target_frame_id.c_str(), ex.what());
    return false;
  }

  // Convert TransformStamped to TF2 transform
  tf2::fromMsg(transform.transform, tf2_transform);
  return true;
}

bool validateTwist(const geometry_msgs::msg::Twist & msg)
{
  // 逐个分量检查线速度与角速度是否为 Inf 或 NaN，
  // 任何一个分量非法都判定消息不可用（避免速度控制器输出失控值）
  if (std::isinf(msg.linear.x) || std::isnan(msg.linear.x)) {
    return false;
  }

  if (std::isinf(msg.linear.y) || std::isnan(msg.linear.y)) {
    return false;
  }

  if (std::isinf(msg.linear.z) || std::isnan(msg.linear.z)) {
    return false;
  }

  if (std::isinf(msg.angular.x) || std::isnan(msg.angular.x)) {
    return false;
  }

  if (std::isinf(msg.angular.y) || std::isnan(msg.angular.y)) {
    return false;
  }

  if (std::isinf(msg.angular.z) || std::isnan(msg.angular.z)) {
    return false;
  }

  return true;
}

}  // end namespace nav2_util
