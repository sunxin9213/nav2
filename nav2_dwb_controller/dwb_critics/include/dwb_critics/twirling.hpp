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

#ifndef DWB_CRITICS__TWIRLING_HPP_
#define DWB_CRITICS__TWIRLING_HPP_

#include "dwb_core/trajectory_critic.hpp"

// ============================================================================
// 模块说明：旋转惩罚评价器（TwirlingCritic）
// ----------------------------------------------------------------------------
// 对轨迹的角速度施加纯粹惩罚，抑制机器人"边走边自旋"（twirling）的行为。
// 背景：
//   - 差速机器人：平移必然伴随旋转，此评价器基本不起作用；
//   - 全向（holonomic / 麦轮）机器人：可以边平移边任意自旋，规划器有时
//     会选出"一路转着圈去目标"的怪异轨迹。本评价器通过 |wz| * scale 的
//     代价让规划器倾向"保持朝向、直接平移"的轨迹。
// 打分公式：scoreTrajectory 直接返回轨迹角速度 theta 的绝对值（再乘
// 评价器统一权重 scale），无其他逻辑。
// ============================================================================

namespace dwb_critics
{
/**
 * @class TwirlingCritic
 * @brief Penalize trajectories with rotational velocities
 *
 * This class provides a cost based on how much a robot "twirls" on its way to the goal. With
 * differential-drive robots, there isn't a choice, but with holonomic or near-holonomic robots,
 * sometimes a robot spins more than you'd like on its way to a goal. This class provides a way
 * to assign a penalty purely to rotational velocities.
 */
// 旋转惩罚评价器：全向底盘专属，惩罚轨迹角速度以避免边走边转圈
class TwirlingCritic : public dwb_core::TrajectoryCritic
{
public:
  // 初始化：调用基类 onInit（本评价器无额外参数）
  void onInit() override;
  // 打分：返回轨迹角速度的绝对值（转得越快罚分越高）
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
};
}  // namespace dwb_critics

#endif  // DWB_CRITICS__TWIRLING_HPP_
