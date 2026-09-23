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

#ifndef DWB_CORE__TRAJECTORY_GENERATOR_HPP_
#define DWB_CORE__TRAJECTORY_GENERATOR_HPP_

#include <vector>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "dwb_msgs/msg/trajectory2_d.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dwb_core
{

/**
 * @class TrajectoryGenerator
 * @brief Interface for iterating through possible velocities and creating trajectories
 *
 * This class defines the plugin interface for two separate but related components.
 *
 * First, this class provides an iterator interface for exploring all of the velocities
 * to search, given the current velocity.
 *
 * Second, the class gives an independent interface for creating a trajectory from a twist,
 * i.e. projecting it out in time and space.
 *
 * Both components rely heavily on the robot's kinematic model, and can share many parameters,
 * which is why they are grouped into a singular class.
 * （轨迹生成器插件接口：组合两个职责——
 *  1) 枚举候选速度（迭代器接口）；
 *  2) 由候选速度前向推演出轨迹。
 *  二者都强依赖机器人运动学模型，故合并为一个插件类。）
 */
class TrajectoryGenerator
{
public:
  typedef std::shared_ptr<dwb_core::TrajectoryGenerator> Ptr;

  virtual ~TrajectoryGenerator() {}

  /**
   * @brief Initialize parameters as needed
   * @param nh NodeHandle to read parameters from
   */
  virtual void initialize(
    const nav2_util::LifecycleNode::SharedPtr & nh,
    const std::string & plugin_name) = 0;
  virtual void reset() {}  // 复位内部状态（新任务时调用）
  /**
   * @brief Start a new iteration based on the current velocity
   * @param current_velocity
   */
  // 以当前速度为中心开始新一轮速度枚举
  virtual void startNewIteration(const nav_2d_msgs::msg::Twist2D & current_velocity) = 0;

  /**
   * @brief Test to see whether there are more twists to test
   * @return True if more twists, false otherwise
   */
  virtual bool hasMoreTwists() = 0;  // 速度空间是否枚举完毕

  /**
   * @brief Return the next twist and advance the iteration
   * @return The Twist!
   */
  virtual nav_2d_msgs::msg::Twist2D nextTwist() = 0;  // 取下一个候选速度

  /**
   * @brief Get all the twists for an iteration.
   *
   * Note: Resets the iterator if one is in process
   *
   * @param current_velocity
   * @return all the twists
   */
  // 便利方法：一次性取出一轮迭代的所有候选速度
  virtual std::vector<nav_2d_msgs::msg::Twist2D> getTwists(
    const nav_2d_msgs::msg::Twist2D & current_velocity)
  {
    std::vector<nav_2d_msgs::msg::Twist2D> twists;
    startNewIteration(current_velocity);  // 重新开始（会丢弃进行中的迭代）
    while (hasMoreTwists()) {
      twists.push_back(nextTwist());
    }
    return twists;
  }

  /**
   * @brief Given a cmd_vel in the robot's frame and initial conditions, generate a Trajectory2D
   * @param start_pose Current robot location
   * @param start_vel Current robot velocity
   * @param cmd_vel The desired command velocity
   */
  // 由候选速度在 sim_time 窗口内推演完整轨迹
  virtual dwb_msgs::msg::Trajectory2D generateTrajectory(
    const geometry_msgs::msg::Pose2D & start_pose,
    const nav_2d_msgs::msg::Twist2D & start_vel,
    const nav_2d_msgs::msg::Twist2D & cmd_vel) = 0;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  // 限速接口：由速度限制过滤器触发（绝对值或百分比模式）
  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
};

}  // namespace dwb_core

#endif  // DWB_CORE__TRAJECTORY_GENERATOR_HPP_
