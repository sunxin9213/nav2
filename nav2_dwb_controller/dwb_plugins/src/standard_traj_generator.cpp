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

#include "dwb_plugins/standard_traj_generator.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include <memory>
#include "dwb_plugins/xy_theta_iterator.hpp"
#include "nav_2d_utils/parameters.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"

namespace dwb_plugins
{

// 中文说明（文件级）：
// 标准轨迹生成器：DWB 的默认 TrajectoryGenerator 插件。
// 算法流程（动态窗口法 DWA/DWB）：
// 1. 由 XYThetaIterator 在加速度可达的速度空间内离散采样目标速度；
// 2. 对每个采样速度，从当前位姿/速度出发按加速度限制向前仿真
//    sim_time 秒（computeNewVelocity 渐近收敛到目标速度，
//    computeNewPosition 按运动学积分位姿），得到候选轨迹；
// 3. 轨迹交由各 critic 评分，选最优轨迹输出其速度指令。
// 轨迹离散方式支持按时间或按距离两种粒度。

// 初始化：创建运动学处理器与速度迭代器，读取仿真参数
void StandardTrajectoryGenerator::initialize(
  const nav2_util::LifecycleNode::SharedPtr & nh,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  // 运动学参数（速度/加速度限值）处理器
  kinematics_handler_ = std::make_shared<KinematicsHandler>();
  kinematics_handler_->initialize(nh, plugin_name_);
  initializeIterator(nh);

  // sim_time：每条候选轨迹的前向仿真时长（秒）
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".sim_time", rclcpp::ParameterValue(1.7));
  // 是否按时间粒度离散轨迹（false 时按距离/角度粒度）
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".discretize_by_time", rclcpp::ParameterValue(false));

  // 时间粒度：按时间离散时相邻轨迹点的时间间隔
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".time_granularity", rclcpp::ParameterValue(0.5));
  // 线距离粒度：按距离离散时相邻点的最大平移距离
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".linear_granularity", rclcpp::ParameterValue(0.5));
  // 角距离粒度：相邻点的最大角度变化
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".angular_granularity", rclcpp::ParameterValue(0.025));
  // 是否在轨迹末尾额外附加终点位姿
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".include_last_point", rclcpp::ParameterValue(true));

  // 是否将加速度受限后的首步实际速度作为轨迹速度记录
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".limit_vel_cmd_in_traj", rclcpp::ParameterValue(false));

  /*
   * If discretize_by_time, then sim_granularity represents the amount of time that should be between
   *  two successive points on the trajectory.
   *
   * If discretize_by_time is false, then sim_granularity is the maximum amount of distance between
   *  two successive points on the trajectory, and angular_sim_granularity is the maximum amount of
   *  angular distance between two successive points.
   */
  nh->get_parameter(plugin_name + ".sim_time", sim_time_);
  nh->get_parameter(plugin_name + ".discretize_by_time", discretize_by_time_);
  nh->get_parameter(plugin_name + ".time_granularity", time_granularity_);
  nh->get_parameter(plugin_name + ".linear_granularity", linear_granularity_);
  nh->get_parameter(plugin_name + ".angular_granularity", angular_granularity_);
  nh->get_parameter(plugin_name + ".include_last_point", include_last_point_);
  nh->get_parameter(plugin_name + ".limit_vel_cmd_in_traj", limit_vel_cmd_in_traj_);
}

// 创建速度采样迭代器（默认 XYThetaIterator，可被子类替换）
void StandardTrajectoryGenerator::initializeIterator(
  const nav2_util::LifecycleNode::SharedPtr & nh)
{
  velocity_iterator_ = std::make_shared<XYThetaIterator>();
  velocity_iterator_->initialize(nh, kinematics_handler_, plugin_name_);
}

// 开始新一轮采样（每个控制周期调用一次）
void StandardTrajectoryGenerator::startNewIteration(
  const nav_2d_msgs::msg::Twist2D & current_velocity)
{
  velocity_iterator_->startNewIteration(current_velocity, sim_time_);
}

// 是否还有未处理的采样速度
bool StandardTrajectoryGenerator::hasMoreTwists()
{
  return velocity_iterator_->hasMoreTwists();
}

// 取下一个采样速度
nav_2d_msgs::msg::Twist2D StandardTrajectoryGenerator::nextTwist()
{
  return velocity_iterator_->nextTwist();
}

// 计算轨迹的仿真时间步序列（决定轨迹点的时间分布）
std::vector<double> StandardTrajectoryGenerator::getTimeSteps(
  const nav_2d_msgs::msg::Twist2D & cmd_vel)
{
  std::vector<double> steps;
  if (discretize_by_time_) {
    // 按时间离散：总时长 / 时间粒度 = 步数
    steps.resize(ceil(sim_time_ / time_granularity_));
  } else {  // discretize by distance
    double vmag = hypot(cmd_vel.x, cmd_vel.y);

    // the distance the robot would travel in sim_time if it did not change velocity
    //（若速度不变，仿真时长内将走过的直线距离）
    double projected_linear_distance = vmag * sim_time_;

    // the angle the robot would rotate in sim_time
    //（仿真时长内将旋转的角度）
    double projected_angular_distance = fabs(cmd_vel.theta) * sim_time_;

    // Pick the maximum of the two
    //（按“距离粒度”与“角度粒度”中更细的取步数）
    int num_steps = ceil(
      std::max(
        projected_linear_distance / linear_granularity_,
        projected_angular_distance / angular_granularity_));
    steps.resize(num_steps);
  }
  if (steps.size() == 0) {
    steps.resize(1);  // 至少 1 步
  }
  // 均分仿真时长到各步
  std::fill(steps.begin(), steps.end(), sim_time_ / steps.size());
  return steps;
}

// 生成单条候选轨迹：从起始位姿/速度出发，按加速度限制向前仿真
dwb_msgs::msg::Trajectory2D StandardTrajectoryGenerator::generateTrajectory(
  const geometry_msgs::msg::Pose2D & start_pose,
  const nav_2d_msgs::msg::Twist2D & start_vel,
  const nav_2d_msgs::msg::Twist2D & cmd_vel)
{
  dwb_msgs::msg::Trajectory2D traj;
  traj.velocity = cmd_vel;  // 轨迹对应的目标速度指令
  //  simulate the trajectory
  geometry_msgs::msg::Pose2D pose = start_pose;
  nav_2d_msgs::msg::Twist2D vel = start_vel;
  double running_time = 0.0;
  std::vector<double> steps = getTimeSteps(cmd_vel);
  traj.poses.push_back(start_pose);  // 轨迹首点为起始位姿
  bool first_vel = false;
  for (double dt : steps) {
    //  calculate velocities
    //（按加速度限制向目标速度收敛一步，模拟真实动力学）
    vel = computeNewVelocity(cmd_vel, vel, dt);
    if (!first_vel && limit_vel_cmd_in_traj_) {
      // 记录受加速度限制后的首步实际速度作为轨迹速度
      traj.velocity = vel;
      first_vel = true;
    }

    //  update the position of the robot using the velocities passed in
    //（用当前速度积分得到新位姿）
    pose = computeNewPosition(pose, vel, dt);

    traj.poses.push_back(pose);
    traj.time_offsets.push_back(rclcpp::Duration::from_seconds(running_time));
    running_time += dt;
  }  //  end for simulation steps

  if (include_last_point_) {
    // 附加最终位姿点（带总时长偏移）
    traj.poses.push_back(pose);
    traj.time_offsets.push_back(rclcpp::Duration::from_seconds(running_time));
  }

  return traj;
}

/**
 * change vel using acceleration limits to converge towards sample_target-vel
 *（按加速度限制将 start_vel 向目标 cmd_vel 收敛一步：
 *  projectVelocity 计算一步内可达的速度变化量并裁剪）
 */
nav_2d_msgs::msg::Twist2D StandardTrajectoryGenerator::computeNewVelocity(
  const nav_2d_msgs::msg::Twist2D & cmd_vel,
  const nav_2d_msgs::msg::Twist2D & start_vel, const double dt)
{
  KinematicParameters kinematics = kinematics_handler_->getKinematics();
  nav_2d_msgs::msg::Twist2D new_vel;
  // 三轴分别投影：当前速度 + 加/减速度限制 * dt，并夹紧到目标速度
  new_vel.x = projectVelocity(
    start_vel.x, kinematics.getAccX(),
    kinematics.getDecelX(), dt, cmd_vel.x);
  new_vel.y = projectVelocity(
    start_vel.y, kinematics.getAccY(),
    kinematics.getDecelY(), dt, cmd_vel.y);
  new_vel.theta = projectVelocity(
    start_vel.theta,
    kinematics.getAccTheta(), kinematics.getDecelTheta(),
    dt, cmd_vel.theta);
  return new_vel;
}

// 运动学积分：由当前位姿与速度计算一步后的新位姿
geometry_msgs::msg::Pose2D StandardTrajectoryGenerator::computeNewPosition(
  const geometry_msgs::msg::Pose2D start_pose,
  const nav_2d_msgs::msg::Twist2D & vel, const double dt)
{
  geometry_msgs::msg::Pose2D new_pose;
  // 机体速度旋转到世界系：x' = vx*cos(θ) + vy*cos(θ+π/2)，y' 同理（欧拉积分）
  new_pose.x = start_pose.x +
    (vel.x * cos(start_pose.theta) + vel.y * cos(M_PI_2 + start_pose.theta)) * dt;
  new_pose.y = start_pose.y +
    (vel.x * sin(start_pose.theta) + vel.y * sin(M_PI_2 + start_pose.theta)) * dt;
  new_pose.theta = start_pose.theta + vel.theta * dt;
  return new_pose;
}

}  // namespace dwb_plugins

PLUGINLIB_EXPORT_CLASS(
  dwb_plugins::StandardTrajectoryGenerator,
  dwb_core::TrajectoryGenerator)
