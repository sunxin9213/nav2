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

#ifndef DWB_PLUGINS__STANDARD_TRAJ_GENERATOR_HPP_
#define DWB_PLUGINS__STANDARD_TRAJ_GENERATOR_HPP_

#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dwb_core/trajectory_generator.hpp"
#include "dwb_plugins/velocity_iterator.hpp"
#include "dwb_plugins/kinematic_parameters.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dwb_plugins
{

/**
 * @class StandardTrajectoryGenerator
 * @brief Standard DWA-like trajectory generator.
 * （标准 DWA 风格轨迹生成器：枚举速度空间中的候选（由 VelocityIterator
 *  决定，如 XYThetaIterator 的三轴笛卡尔积），对每个候选速度
 *  在 sim_time_ 时间窗内按恒加速/匀速模型前向推演出一条轨迹，
 *  交给评价器打分。）
 */
class StandardTrajectoryGenerator : public dwb_core::TrajectoryGenerator
{
public:
  // Standard TrajectoryGenerator interface
  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & nh,
    const std::string & plugin_name) override;  // 初始化迭代器与推演参数
  void startNewIteration(const nav_2d_msgs::msg::Twist2D & current_velocity) override;
  // 以当前速度开始新一轮速度枚举
  bool hasMoreTwists() override;          // 速度空间是否枚举完毕
  nav_2d_msgs::msg::Twist2D nextTwist() override;  // 下一个候选速度

  // 由候选速度生成完整轨迹（含位姿序列与速度序列）
  dwb_msgs::msg::Trajectory2D generateTrajectory(
    const geometry_msgs::msg::Pose2D & start_pose,
    const nav_2d_msgs::msg::Twist2D & start_vel,
    const nav_2d_msgs::msg::Twist2D & cmd_vel) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  // 限速：转发给运动学参数处理器（按比例/绝对值缩放最大速度）
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    if (kinematics_handler_) {
      kinematics_handler_->setSpeedLimit(speed_limit, percentage);
    }
  }

protected:
  /**
   * @brief Initialize the VelocityIterator pointer. Put in its own function for easy overriding
   */
  // 创建速度迭代器实例（默认 XYThetaIterator，子类可重写换采样策略）
  virtual void initializeIterator(const nav2_util::LifecycleNode::SharedPtr & nh);

  /**
   * @brief Calculate the velocity after a set period of time, given the desired velocity and acceleration limits
   *
   * @param cmd_vel Desired velocity
   * @param start_vel starting velocity
   * @param dt amount of time in seconds
   * @return new velocity after dt seconds
   */
  // 轨迹内每步的速度更新：从 start_vel 向 cmd_vel 按加/减速上限逼近
  virtual nav_2d_msgs::msg::Twist2D computeNewVelocity(
    const nav_2d_msgs::msg::Twist2D & cmd_vel, const nav_2d_msgs::msg::Twist2D & start_vel,
    const double dt);

  /**
   * @brief Use the robot's kinematic model to predict new positions for the robot
   *
   * @param start_pose Starting pose
   * @param vel Actual robot velocity (assumed to be within acceleration limits)
   * @param dt amount of time in seconds
   * @return New pose after dt seconds
   */
  // 运动学模型推演位姿：x' = x + vx·cos(θ)·dt - vy·sin(θ)·dt 等
  virtual geometry_msgs::msg::Pose2D computeNewPosition(
    const geometry_msgs::msg::Pose2D start_pose, const nav_2d_msgs::msg::Twist2D & vel,
    const double dt);


  /**
   * @brief Compute an array of time deltas between the points in the generated trajectory.
   *
   * @param cmd_vel The desired command velocity
   * @return vector of the difference between each time step in the generated trajectory
   *
   * If we are discretizing by time, the returned vector will be the same constant time_granularity
   * for all cmd_vels. Otherwise, you will get times based on the linear/angular granularity.
   *
   * Right now the vector contains a single value repeated many times, but this method could be overridden
   * to allow for dynamic spacing
   */
  // 计算轨迹点之间的时间步长数组：
  // 按时间离散时为常量 time_granularity；否则按线/角粒度换算
  virtual std::vector<double> getTimeSteps(const nav_2d_msgs::msg::Twist2D & cmd_vel);

  KinematicsHandler::Ptr kinematics_handler_;           // 运动学参数管理器
  std::shared_ptr<VelocityIterator> velocity_iterator_;  // 速度采样迭代器

  double sim_time_;   // 轨迹推演时间窗（秒）

  // Sampling Parameters
  bool discretize_by_time_;  // 是否按时间（而非按距离）离散轨迹点

  /// @brief If discretizing by time, the amount of time between each point in the traj
  double time_granularity_;   // 时间离散粒度

  /// @brief If not discretizing by time, the amount of linear space between points
  double linear_granularity_;  // 线距离离散粒度

  /// @brief If not discretizing by time, the amount of angular space between points
  double angular_granularity_;  // 角度离散粒度

  /// @brief the name of the overlying plugin ID
  std::string plugin_name_;   // 插件名（参数命名空间）

  /// @brief Option to limit velocity in the trajectory generator by using current velocity
  bool limit_vel_cmd_in_traj_;  // 是否将轨迹内的指令速度限制在当前速度附近

  /* Backwards Compatibility Parameter: include_last_point
   *
   * dwa had an off-by-one error built into it.
   * It generated N trajectory points, where N = ceil(sim_time / time_delta).
   * If for example, sim_time=3.0 and time_delta=1.5, it would generate trajectories with 2 points, which
   * indeed were time_delta seconds apart. However, the points would be at t=0 and t=1.5, and thus the
   * actual sim_time was much less than advertised.
   *
   * This is remedied by adding one final point at t=sim_time, but only if include_last_point_ is true.
   *
   * Nothing I could find actually used the time_delta variable or seemed to care that the trajectories
   * were not projected out as far as they intended.
   */
  // 兼容开关：是否补上 t=sim_time 的末端点，修正旧 DWA 的少一点问题
  bool include_last_point_;
};


}  // namespace dwb_plugins

#endif  // DWB_PLUGINS__STANDARD_TRAJ_GENERATOR_HPP_
