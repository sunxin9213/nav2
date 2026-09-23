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
#ifndef DWB_CRITICS__ROTATE_TO_GOAL_HPP_
#define DWB_CRITICS__ROTATE_TO_GOAL_HPP_

#include <string>
#include <vector>
#include "dwb_core/trajectory_critic.hpp"

// ============================================================================
// 模块说明：原地转向目标评价器（RotateToGoalCritic）
// ----------------------------------------------------------------------------
// 实现"到点后先停稳、再原地旋转到目标朝向"的终端控制逻辑。
// 历史背景：该功能原为 DWA Local Planner 内置的 LatchedStopRotate 控制器，
// Nav2 中为保持评价器框架的一致性而重构为一个评价器。
// 三阶段状态机（以当前位姿到目标点的线性距离与 xy_goal_tolerance 比较）：
//   阶段 1（远离目标）：距离 > xy_goal_tolerance，本评价器直接返回 0 分，
//     不干预（让其他评价器主导正常行驶）；
//   阶段 2（刚进入容差窗口且仍有线速度）：只允许"比当前速度更慢"的轨迹，
//     使机器人在加速度限制内减速刹停。分数 = 当前线速度平方 *
//     slowing_factor（默认 5.0）+ scoreRotation 结果；
//   阶段 3（已基本停稳）：任何带线速度的轨迹都判为非法（负分），
//     只允许纯旋转轨迹，按 scoreRotation 打分。
// scoreRotation（可重写）默认返回轨迹中某个位姿与目标朝向的最短角差：
//   - lookahead_time < 0（默认）：取轨迹最后一个位姿（与 DWA 行为一致）；
//   - lookahead_time >= 0       ：按该前瞻时间投影出一个更早的位姿。
//     前瞻时间比 sim_time 短时，评价器对"冲过头（overshoot）"不那么敏感，
//     机器人会以更快转速转向更久。
// ============================================================================

namespace dwb_critics
{

/**
 * @class RotateToGoalCritic
 * @brief Forces the commanded trajectories to only be rotations if within a certain distance window
 *
 * This used to be built in to the DWA Local Planner as the LatchedStopRotate controller,
 * but has been moved to a critic for consistency.
 *
 * The critic has three distinct phases.
 *  1) If the current pose is outside xy_goal_tolerance LINEAR distance from the goal pose, this critic
 *     will just return score 0.0.
 *  2) If within the xy_goal_tolerance and the robot is still moving with non-zero linear motion, this critic
 *     will only allow trajectories that are slower than the current speed in order to stop the robot (within
 *     the robot's acceleration limits). The returned score will be the robot's linear speed squared, multiplied
 *     by the slowing_factor parameter (default 5.0) added to the result of scoreRotation.
 *  3) If within the xy_goal_tolerance and the robot has sufficiently small linear motion, this critic will
 *     score trajectories that have linear movement as invalid and score the rest based on the result of the
 *     scoreRotation method
 *
 * The scoreRotation method can be overriden, but the default behavior is to return the shortest angular distance
 * between the goal pose and a pose from the trajectory. Which pose depends on the lookahead_time parameter.
 *  * If the lookahead_time parameter is negative, the pose evaluated will be the last pose in the trajectory,
 *    which is the same as DWA's behavior. This is the default.
 *  * Otherwise, a new pose will be projected using the dwb_local_planner::projectPose. By using a lookahead
 *    time shorter than sim_time, the critic will be less concerned about overshooting the goal yaw and thus will
 *    continue to turn faster for longer.
 */
// 到点旋转评价器：目标附近"减速-刹停-纯旋转"三阶段终端控制
class RotateToGoalCritic : public dwb_core::TrajectoryCritic
{
public:
  // 初始化：读取 xy_goal_tolerance、slowing_factor、lookahead_time 等参数
  void onInit() override;
  // 复位：清空窗口/旋转状态标志（换目标时重新经历三阶段）
  void reset() override;
  // 每周期准备：计算到目标距离、判断是否进入容差窗口、更新状态标志
  void prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  // 打分：按三阶段状态机（远离=0 分 / 减速=速度平方*因子 / 旋转=角差）打分
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  /**
   * @brief Assuming that this is an actual rotation when near the goal, score the trajectory.
   *
   * This (easily overridden) method assumes that the critic is in the third phase (as described above)
   * and returns a numeric score for the trajectory relative to the goal yaw.
   * @param traj Trajectory to score
   * @return numeric score
   */
  // 旋转打分（虚拟，便于派生类定制）：返回轨迹位姿朝向与目标朝向的最短角差
  virtual double scoreRotation(const dwb_msgs::msg::Trajectory2D & traj);

private:
  bool in_window_;               // 是否已进入 xy_goal_tolerance 容差窗口（阶段 2/3）
  bool rotating_;                // 是否已进入纯旋转阶段（阶段 3）
  double goal_yaw_;              // 目标点的朝向角（prepare 时记录）
  double xy_goal_tolerance_;     // 目标位置容差（米）
  double xy_goal_tolerance_sq_;  ///< Cached squared tolerance（容差平方，避免开方）
  double current_xy_speed_sq_;        // 当前线速度平方（阶段 2 判断减速用）
  double stopped_xy_velocity_sq_;     // 判定"已停稳"的线速度平方阈值
  double slowing_factor_;             // 减速阶段的罚分放大系数（默认 5.0）
  double lookahead_time_;             // 旋转打分的前瞻时间（负值=取轨迹末位姿）
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS__ROTATE_TO_GOAL_HPP_
