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

#ifndef DWB_CRITICS__PREFER_FORWARD_HPP_
#define DWB_CRITICS__PREFER_FORWARD_HPP_

#include <string>
#include "dwb_core/trajectory_critic.hpp"

// ============================================================================
// 模块说明：偏好前进评价器（PreferForwardCritic）
// ----------------------------------------------------------------------------
// 惩罚"倒车"与"原地大转向"的轨迹，鼓励机器人优先向前行驶。
// 适用场景：差速/前轮转向机器人倒车控制性差、传感器（如激光雷达）通常
// 朝前安装，倒车既不安全也缺乏感知。
// 三种打分情形（scoreTrajectory 实现）：
//   1) 轨迹 x 速度为负（倒车）              → 返回固定罚分 penalty_；
//   2) x 速度低且角速度也低（近原地蹭动）    → 返回固定罚分 penalty_；
//   3) 其他（正常前进，可能有转向）           → 返回 |theta| * theta_scale_，
//      即按角速度大小惩罚"过度旋转"的轨迹。
// 参数（均在 onInit 中读取）：
//   penalty_      倒车/蹭动的固定罚分（默认 1.0）；
//   strafe_x_     "低速 x"的判断阈值（默认 0.1 m/s）；
//   strafe_theta_ "低速角速度"的判断阈值（默认 0.2 rad/s）；
//   theta_scale_  正常轨迹下角速度的缩放系数（默认 10.0）。
// ============================================================================

namespace dwb_critics
{

/**
 * @class PreferForwardCritic
 * @brief Penalize trajectories with move backwards and/or turn too much
 *
 * Has three different scoring conditions:
 * 1) If the trajectory's x velocity is negative, return the penalty
 * 2) If the trajectory's x is low and the theta is also low, return the penalty.
 * 3) Otherwise, return a scaled version of the trajectory's theta.
 */
// 偏好前进评价器：惩罚倒车和原地蹭动，抑制过度旋转
class PreferForwardCritic : public dwb_core::TrajectoryCritic
{
public:
  // 构造：penalty=1.0、strafe_x=0.1、strafe_theta=0.2、theta_scale=10.0
  PreferForwardCritic()
  : penalty_(1.0), strafe_x_(0.1), strafe_theta_(0.2), theta_scale_(10.0) {}
  // 初始化：从参数服务器读取上述四个参数
  void onInit() override;
  // 打分：倒车或低速蹭动返回 penalty_，正常前进返回 |theta|*theta_scale_
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

private:
  double penalty_, strafe_x_, strafe_theta_, theta_scale_;  // 罚分/低速阈值/角速度缩放
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS__PREFER_FORWARD_HPP_
