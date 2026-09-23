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

#ifndef NAV_2D_UTILS__TF_HELP_HPP_
#define NAV_2D_UTILS__TF_HELP_HPP_

#include <string>
#include <memory>
#include "tf2_ros/buffer.h"
#include "nav_2d_utils/conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav_2d_utils
{
/**
 * @brief Transform a PoseStamped from one frame to another while catching exceptions
 *
 * Also returns immediately if the frames are equal.
 * @param tf Smart pointer to TFListener
 * @param frame Frame to transform the pose into
 * @param in_pose Pose to transform
 * @param out_pose Place to store the resulting transformed pose
 * @return True if successful transform
 */
// ---------------------------------------------------------------------------
// TF2 坐标变换辅助函数（ROS2 TF API 学习素材）：
// 对 tf2_ros::Buffer::transform 的安全封装，解决两个痛点：
//   1. 免变换优化：目标坐标系与位姿坐标系相同时直接拷贝返回；
//   2. 异常处理：tf2 变换可能抛出多种异常（外推/超时/不存在），
//      这里统一捕获：
//      - ExtrapolationException（变换数据时间不匹配）：改用最新可用
//        变换（TimePointZero）并检查数据时延是否在容忍范围内；
//      - 其他 TransformException：打印错误返回 false。
// 成功返回 true 并写 out_pose；失败返回 false。
// 提供两个重载：PoseStamped（3D 带戳位姿）与 Pose2DStamped（2D 版本，
// 内部先转 3D 变换再转回 2D）。
// ---------------------------------------------------------------------------
// 将 PoseStamped 从其自身坐标系变换到目标 frame（带异常捕获与时延检查）
bool transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  rclcpp::Duration & transform_tolerance
);

/**
 * @brief Transform a Pose2DStamped from one frame to another while catching exceptions
 *
 * Also returns immediately if the frames are equal.
 * @param tf Smart pointer to TFListener
 * @param frame Frame to transform the pose into
 * @param in_pose Pose to transform
 * @param out_pose Place to store the resulting transformed pose
 * @return True if successful transform
 */
// 2D 版本：先转成 PoseStamped 完成 TF 变换，再转回 Pose2DStamped
bool transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const nav_2d_msgs::msg::Pose2DStamped & in_pose,
  nav_2d_msgs::msg::Pose2DStamped & out_pose,
  rclcpp::Duration & transform_tolerance
);

}  // namespace nav_2d_utils

#endif  // NAV_2D_UTILS__TF_HELP_HPP_
