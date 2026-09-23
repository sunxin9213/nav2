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

#ifndef DWB_CRITICS__OSCILLATION_HPP_
#define DWB_CRITICS__OSCILLATION_HPP_

#include <vector>
#include <string>
#include <chrono>
#include "dwb_core/trajectory_critic.hpp"

using namespace std::chrono_literals;  // NOLINT

// ============================================================================
// 模块说明：振荡评价器（OscillationCritic）
// ----------------------------------------------------------------------------
// 检测机器人是否在"来回打摆"（oscillation）：即速度符号在 x/y/theta 三个
// 维度上频繁翻转（正->负->正），而期间又没有移动足够远或等待足够久。
// 防振荡的意义：DWB 采样式局部规划器可能在狭窄区域里选出的轨迹反复切换
// 方向（前进-后退-前进），导致机器人卡死。本评价器一旦检测到振荡苗头，
// 就把"会延续振荡方向"的轨迹判为非法（负分），强迫机器人沿单一方向解围。
//
// 判定规则（以 x 维度为例）：
//   场景 1：前进 1 米后倒退 2 毫米——倒退使 x 符号翻转（正->负）被记录。
//   若此时再前进，符号再次翻转（负->正），且移动量/时间未超过复位阈值，
//   则视为振荡：正向速度轨迹被赋予 oscillation_score（非法），只允许继续
//   倒退，直到满足复位条件。
//   场景 2：前进 1 米后倒退 1 米——倒退后累计移动超过 oscillation_reset_dist，
//   不算振荡，所有轨迹均合法。
//
// 复位机制（resetAvailable）：
//   - 距离复位：当前位姿到 prev_stationary_pose_（上次符号翻转时记录的
//     停驻位姿）的线性距离 > oscillation_reset_dist_，或角度差 >
//     oscillation_reset_angle_；
//   - 时间复位：距上次复位时间超过 oscillation_reset_time_。
//   满足任一条件即清除全部符号翻转记录，恢复自由选择。
//
// x_only_threshold_：当 |vx| 超过该阈值时只检查 x 维度（高速时 y/theta
// 的符号变化通常无害）；为负则始终检查全部三个维度。
// ============================================================================

namespace dwb_critics
{

/**
 * @class OscillationCritic
 * @brief Checks to see whether the sign of the commanded velocity flips frequently
 *
 * This critic figures out if the commanded trajectories are oscillating by seeing
 * if one of the dimensions (x,y,theta) flips from positive to negative and then back
 * (or vice versa) without moving sufficiently far or waiting sufficiently long.
 *
 * Scenario 1: Robot moves one meter forward, and then two millimeters backward.
 * Another forward motion would be considered oscillating, since the x dimension would then
 * flip from positive to negative and then back to negative. Hence, when scoring different
 * trajectories, positive velocity commands will get the oscillation_score (-5.0, or invalid)
 * and only negative velocity commands will be considered valid.

 * Scenario 2: Robot moves one meter forward, and then one meter backward.
 * The robot has thus moved one meter since flipping the sign of the x direction, which
 * is greater than our oscillation_reset_dist, so its not considered oscillating, so all
 * trajectories are considered valid.
 *
 * Note: The critic will only check oscillations in the x dimension while it exceeds
 * a particular value (x_only_threshold_). If it dips below that magnitude, it will
 * also check for oscillations in the y and theta dimensions. If x_only_threshold_ is
 * negative, then the critic will always check all dimensions.
 *
 * Implementation Details:
 * The critic saves the robot's current position when it prepares, and what the actual
 * commanded velocity was during the debrief step. Upon debriefing, if the sign of any of
 * dimensions has flipped since the last command, the position is saved as prev_stationary_pose_.
 *
 * If the linear or angular distance from prev_stationary_pose_ to the current pose exceeds
 * the limits, the oscillation flags are reset so the previous sign change is no longer remembered.
 * This assumes that oscillation_reset_dist_ or oscillation_reset_angle_ are positive. Otherwise,
 * it uses a time based delay reset function.
 */
class OscillationCritic : public dwb_core::TrajectoryCritic
{
public:
  // 构造：复位时间默认 0 秒（onInit 中从参数读取）
  OscillationCritic()
  : oscillation_reset_time_(0s) {}
  // 初始化：读取 oscillation_reset_dist / angle / time、x_only_threshold 等参数
  void onInit() override;
  // 每周期准备：保存当前位姿；若满足复位条件则清除全部符号翻转记录
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  // 打分：对会被判为"延续振荡"的速度符号返回非法负分，否则返回 0 分
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  // 复位：清空三个维度趋势跟踪器的符号记录
  void reset() override;
  // 事后回告（DWB 每周期执行实际选中速度后调用）：更新各维度符号趋势，
  // 若发生新的翻转则记录当前位姿为 prev_stationary_pose_
  void debrief(const nav_2d_msgs::msg::Twist2D & cmd_vel) override;

private:
  /**
   * @class CommandTrend
   * @brief Helper class for performing the same logic on the x,y and theta dimensions
   */
  // 命令趋势跟踪器：对单一维度记录速度符号翻转状态（x/y/theta 各一份实例）
  class CommandTrend
  {
public:
    CommandTrend();          // 初始状态：sign_=ZERO，无方向限制
    void reset();            // 清空符号记录，恢复初始状态

    /**
     * @brief update internal flags based on the commanded velocity
     * @param velocity commanded velocity for the dimension this trend is tracking
     * @return true if the sign has flipped
     */
    // 根据实际执行的命令速度更新内部符号状态；若与上次相比发生翻转返回 true
    bool update(double velocity);

    /**
     * @brief Check to see whether the proposed velocity would be considered oscillating
     * @param velocity the velocity to evaluate
     * @return true if the sign has flipped more than once
     */
    // 判断候选速度是否会延续振荡：若已翻转且方向受限则返回 true
    bool isOscillating(double velocity);

    /**
     * @brief Check whether we are currently tracking a flipped sign
     * @return True if the sign has flipped
     */
    // 是否正在跟踪一次未复位的符号翻转
    bool hasSignFlipped();

private:
    // Simple Enum for Tracking
    // 速度符号枚举：零 / 正 / 负
    // cppcheck-suppress syntaxError
    enum class Sign { ZERO, POSITIVE, NEGATIVE };

    Sign sign_;                            // 上次命令速度的符号
    bool positive_only_, negative_only_;   // 翻转后施加的方向限制标志（只许正/只许负）
  };

  /**
   * @brief Given a command that has been selected, track each component's sign for oscillations
   * @param cmd_vel The command velocity selected by the algorithm
   * @return True if the sign on any of the components flipped
   */
  bool setOscillationFlags(const nav_2d_msgs::msg::Twist2D & cmd_vel);

  /**
   * @brief Return true if the robot has travelled far enough or waited long enough
   */
  // 复位条件判定：移动距离/转角超过阈值，或距上次复位时间超过设定时长
  bool resetAvailable();

  // x/y/theta 三个维度的符号趋势跟踪器
  CommandTrend x_trend_, y_trend_, theta_trend_;
  double oscillation_reset_dist_;        // 距离复位阈值（米）
  double oscillation_reset_angle_;       // 角度复位阈值（弧度）
  double x_only_threshold_;             // 仅检查 x 维度的速度阈值（负值=始终全检）
  rclcpp::Duration oscillation_reset_time_;  // 时间复位阈值

  // Cached square parameter
  double oscillation_reset_dist_sq_;     // 缓存的距离阈值平方（避免每次比较开方）

  // Saved positions
  geometry_msgs::msg::Pose2D pose_, prev_stationary_pose_;  // 当前位姿 / 上次翻转时停驻位姿
  // Saved timestamp
  rclcpp::Time prev_reset_time_;        // 上次复位时间戳
  rclcpp::Clock::SharedPtr clock_;      // 时钟句柄（ROS 时间源）
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS__OSCILLATION_HPP_
