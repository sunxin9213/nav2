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

#ifndef DWB_PLUGINS__LIMITED_ACCEL_GENERATOR_HPP_
#define DWB_PLUGINS__LIMITED_ACCEL_GENERATOR_HPP_

#include <memory>
#include <string>

#include "dwb_plugins/standard_traj_generator.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dwb_plugins
{
/**
 * @class LimitedAccelGenerator
 * @brief Limits the acceleration in the generated trajectories to a fraction of the simulated time.
 * （受限加速度轨迹生成器：继承标准生成器，但把速度采样窗口限制在
 *  "当前速度 ± 加速度上限 × acceleration_time_" 内，即速度指令
 *  在一个控制周期内物理可达。与标准生成器的区别：
 *  标准：先取可达速度，再在轨迹推演中继续加速；
 *  受限：采样窗口本身就受限，轨迹内速度保持恒定。）
 */
class LimitedAccelGenerator : public StandardTrajectoryGenerator
{
public:
  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & nh,
    const std::string & plugin_name) override;  // 初始化基类并读取 sim_period
  void startNewIteration(const nav_2d_msgs::msg::Twist2D & current_velocity) override;
  // 以当前速度为中心、加速度限制为半径开始采样（时间窗=acceleration_time_）

protected:
  /**
   * @brief Calculate the velocity after a set period of time, given the desired velocity and acceleration limits
   *
   * Unlike the StandardTrajectoryGenerator, the velocity remains constant in the LimitedAccelGenerator
   *
   * @param cmd_vel Desired velocity
   * @param start_vel starting velocity
   * @param dt amount of time in seconds
   * @return cmd_vel
   */
  // 直接返回 cmd_vel：速度可行性已由受限采样窗口保证
  nav_2d_msgs::msg::Twist2D computeNewVelocity(
    const nav_2d_msgs::msg::Twist2D & cmd_vel,
    const nav_2d_msgs::msg::Twist2D & start_vel,
    const double dt) override;
  double acceleration_time_;   // 加速时间窗（默认为控制周期 1/controller_frequency）
  std::string plugin_name_;    // 插件名（参数命名空间）
};
}  // namespace dwb_plugins

#endif  // DWB_PLUGINS__LIMITED_ACCEL_GENERATOR_HPP_
