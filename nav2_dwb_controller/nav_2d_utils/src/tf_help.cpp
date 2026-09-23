/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <memory>
#include <string>
#include "nav_2d_utils/tf_help.hpp"

namespace nav_2d_utils
{

// 带异常处理的坐标变换（3D 位姿版本）：
//   - 目标坐标系相同 -> 直接拷贝，免去 TF 查询；
//   - 正常路径：tf->transform 完成变换；
//   - 外推异常（ExtrapolationException，位姿时间戳超出 TF 缓存范围）：
//     改用最新变换（TimePointZero）手动变换，但要先检查位姿时间与
//     变换时间的差值是否在容忍度以内，超龄则报错返回 false；
//   - 其他变换异常：打印错误返回 false。
bool transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  rclcpp::Duration & transform_tolerance
)
{
  // 免变换优化：源坐标系与目标坐标系相同时直接原样拷贝
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    // 标准 TF 变换：tf2_ros::Buffer::transform 会自动寻找合适时间戳的变换
    tf->transform(in_pose, out_pose, frame);
    return true;
  } catch (tf2::ExtrapolationException & ex) {
    // 外推异常：TF 树中没有覆盖 in_pose 时间戳的变换数据
    // 回退策略：查询最新的可用变换（TimePointZero = 最新）
    auto transform = tf->lookupTransform(
      frame,                        // 目标坐标系
      in_pose.header.frame_id,      // 源坐标系
      tf2::TimePointZero            // 取最新变换而非按时间戳匹配
    );
    // 检查"数据时延"：位姿时间戳 - 变换时间戳 是否超过容忍度
    if (
      (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
      transform_tolerance)
    {
      // 超过容忍度：数据过于陈旧，拒绝变换
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Transform data too old when converting from %s to %s",
        in_pose.header.frame_id.c_str(),
        frame.c_str()
      );
      // 打印数据时间与变换时间，便于排查 TF 树哪个环节延迟过大
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Data time: %ds %uns, Transform time: %ds %uns",
        in_pose.header.stamp.sec,
        in_pose.header.stamp.nanosec,
        transform.header.stamp.sec,
        transform.header.stamp.nanosec
      );
      return false;
    } else {
      // 时延在容忍度内：用最新变换手动执行变换（tf2::doTransform）
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    }
  } catch (tf2::TransformException & ex) {
    // 其他 TF 异常（坐标系不存在、TF 树断链等）：报错并返回失败
    RCLCPP_ERROR(
      rclcpp::get_logger("tf_help"),
      "Exception in transformPose: %s",
      ex.what()
    );
    return false;
  }
  return false;
}

// 带异常处理的坐标变换（2D 位姿版本）：
// 借助 conversions 的转换函数：2D -> 3D -> TF 变换 -> 2D
bool transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const nav_2d_msgs::msg::Pose2DStamped & in_pose,
  nav_2d_msgs::msg::Pose2DStamped & out_pose,
  rclcpp::Duration & transform_tolerance
)
{
  // 2D 位姿先转为 3D PoseStamped（yaw 构造四元数）
  geometry_msgs::msg::PoseStamped in_3d_pose = pose2DToPoseStamped(in_pose);
  geometry_msgs::msg::PoseStamped out_3d_pose;

  // 复用上面的 3D 版本完成实际变换
  bool ret = transformPose(tf, frame, in_3d_pose, out_3d_pose, transform_tolerance);
  if (ret) {
    // 成功后再压平回 2D（四元数重新提取 yaw）
    out_pose = poseStampedToPose2D(out_3d_pose);
  }
  return ret;
}
}  // namespace nav_2d_utils
