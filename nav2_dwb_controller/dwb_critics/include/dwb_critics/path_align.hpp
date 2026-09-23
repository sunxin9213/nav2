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
#ifndef DWB_CRITICS__PATH_ALIGN_HPP_
#define DWB_CRITICS__PATH_ALIGN_HPP_

#include <vector>
#include <string>
#include "dwb_critics/path_dist.hpp"

// ============================================================================
// 模块说明：路径对齐评价器（PathAlignCritic）
// ----------------------------------------------------------------------------
// 继承自 PathDistCritic（路径距离评价器），同样以全局路径为源点构建
// BFS 距离场，但打分对象不同：
//   - PathDistCritic ：评估轨迹终点（机器人中心）到全局路径的距离；
//   - PathAlignCritic：评估机器人前方 forward_point_distance 米处的
//     "虚拟点"到全局路径的距离。
// 效果：当机器人朝向与路径方向一致时，其前方的点仍在路径附近，代价低；
// 当机器人朝向偏离路径时，前方的点会远离路径，代价升高。因此该评价器
// 隐式地将机器人"拉直"到与全局路径对齐的航向上。
// 终点特判（zero_scale_）：当机器人接近路径末端时，前向虚拟点可能越过
// 终点（路径尽头没有格子可对齐），此时本评价器权重置零，避免对不存在
// 的路径段强行对齐。
// ============================================================================

namespace dwb_critics
{
/**
 * @class PathAlignCritic
 * @brief Scores trajectories based on how far from the global path the front of the robot ends up.
 *
 * This uses the costmap grid as a proxy for calculating which way the robot should be facing relative
 * to the global path. Instead of scoring how far the center of the robot is away from the global path,
 * this critic calculates how far a point forward_point_distance in front of the robot is from the global
 * path. This biases the planner toward trajectories that line up with the global plan.
 *
 * When the robot is near the end of the path, the scale of this critic is set to zero. When the projected
 * point is past the global goal, we no longer want this critic to try to align to a part of the global path
 * that isn't there.
 */
// 路径对齐评价器：以机器人前方虚拟点到全局路径的距离作为朝向对齐的代理度量
class PathAlignCritic : public PathDistCritic
{
public:
  // 构造：默认不对齐禁用（zero_scale_ 为 false），前向点距离 0.0
  PathAlignCritic()
  : zero_scale_(false), forward_point_distance_(0.0) {}
  // 初始化：调用父类 onInit 并声明 forward_point_distance 参数
  void onInit() override;
  // 准备：调用父类 prepare 构建路径距离场；若接近路径末端则置 zero_scale_
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  // 重写 getScale：接近终点时返回 0（禁用本评价器），否则调用父类实现
  double getScale() const override;
  // 打分：将位姿沿朝向前移 forward_point_distance_ 后查路径距离场
  double scorePose(const geometry_msgs::msg::Pose2D & pose) override;

protected:
  bool zero_scale_;               // 接近路径末端时的"零权重"标志
  double forward_point_distance_; // 机器人前方虚拟点的距离（米）
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS__PATH_ALIGN_HPP_
