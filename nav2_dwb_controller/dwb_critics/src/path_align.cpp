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

#include "dwb_critics/path_align.hpp"
#include <vector>
#include <string>
#include "dwb_critics/alignment_util.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/parameters.hpp"

namespace dwb_critics
{

// 中文说明（文件级）：
// 路径对齐（Path Align）critic：惩罚机器人“前探点”到全局路径的距离，
// 使机器人行进方向与路径切向对齐（而非仅中心点贴路径）。
// 继承 PathDistCritic（共用路径距离场），但评分时取轨迹位姿沿朝向
// 前方 forward_point_distance 处的前探点查询距离，
// 从而把“朝向”信息引入评分。接近目标时自动失效（zero_scale）。

// 初始化：复用父类初始化并读取前探距离参数
void PathAlignCritic::onInit()
{
  PathDistCritic::onInit();
  // 对齐类 critic 不因障碍抛异常（避免过度剔除轨迹）
  stop_on_failure_ = false;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // 前探点距离：沿机器人朝向前伸出的评价点（“鼻尖”）
  forward_point_distance_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".forward_point_distance", 0.325);
}

// 周期准备：接近目标时禁用本 critic，否则复用父类的路径距离场构建
bool PathAlignCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  double dx = pose.x - goal.x;
  double dy = pose.y - goal.y;
  double sq_dist = dx * dx + dy * dy;
  if (sq_dist > forward_point_distance_ * forward_point_distance_) {
    zero_scale_ = false;  // 距目标尚远：正常参与评分
  } else {
    // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
    //（接近目标时保持鼻尖贴路径会导致行为不稳定，故禁用）
    zero_scale_ = true;
    return true;
  }

  // 复用父类：构建全局路径的曼哈顿距离场
  return PathDistCritic::prepare(pose, vel, goal, global_plan);
}

// 获取有效权重：禁用时为 0，否则按地图分辨率缩放
double PathAlignCritic::getScale() const
{
  if (zero_scale_) {
    return 0.0;
  } else {
    return costmap_->getResolution() * 0.5 * scale_;
  }
}

// 单点评分：用前探点（而非中心点）查询路径距离
double PathAlignCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  return PathDistCritic::scorePose(getForwardPose(pose, forward_point_distance_));
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::PathAlignCritic, dwb_core::TrajectoryCritic)
