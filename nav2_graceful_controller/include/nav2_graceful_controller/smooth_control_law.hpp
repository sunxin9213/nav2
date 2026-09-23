// Copyright (c) 2023 Alberto J. Tudela Roldán
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_GRACEFUL_CONTROLLER__SMOOTH_CONTROL_LAW_HPP_
#define NAV2_GRACEFUL_CONTROLLER__SMOOTH_CONTROL_LAW_HPP_

#include <algorithm>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nav2_graceful_controller
{

/**
 * @class nav2_graceful_controller::SmoothControlLaw
 * @brief Smooth control law for graceful motion based on "A smooth control law for graceful motion"
 * (Jong Jin Park and Benjamin Kuipers).
 *（平滑控制律：Graceful Controller 的核心速度计算组件）
 *
 * 中文说明：
 * 基于 Park & Kuipers 的平滑控制律理论，在自极坐标 (r, φ, δ) 中
 * 用双时间尺度（慢子系统 r-φ、快子系统 δ）反馈控制计算曲率，
 * 实现位姿（位置+朝向）的渐近收敛。实现见 src/smooth_control_law.cpp。
 */
class SmoothControlLaw
{
public:
  /**
   * @brief Constructor for nav2_graceful_controller::SmoothControlLaw
   *
   * @param k_phi Ratio of the rate of change in phi to the rate of change in r.
   * @param k_delta Constant factor applied to the heading error feedback.
   * @param beta Constant factor applied to the path curvature: dropping velocity.
   * @param lambda Constant factor applied to the path curvature for sharpness.
   * @param slowdown_radius Radial threshold applied to the slowdown rule.
   * @param v_linear_min Minimum linear velocity.
   * @param v_linear_max Maximum linear velocity.
   * @param v_angular_max Maximum angular velocity.
   */
  SmoothControlLaw(
    double k_phi, double k_delta, double beta, double lambda, double slowdown_radius,
    double v_linear_min, double v_linear_max, double v_angular_max);

  /**
   * @brief Destructor for nav2_graceful_controller::SmoothControlLaw
   */
  ~SmoothControlLaw() = default;

  /**
   * @brief Set the values that define the curvature.
   *
   * @param k_phi Ratio of the rate of change in phi to the rate of change in r.
   * @param k_delta Constant factor applied to the heading error feedback.
   * @param beta Constant factor applied to the path curvature: dropping velocity.
   * @param lambda Constant factor applied to the path curvature for sharpness.
   */
  void setCurvatureConstants(
    const double k_phi, const double k_delta, const double beta, const double lambda);

  /**
   * @brief Set the slowdown radius
   *
   * @param slowdown_radius Radial threshold applied to the slowdown rule.
   */
  void setSlowdownRadius(const double slowdown_radius);

  /**
   * @brief Update the velocity limits.
   *
   * @param v_linear_min The minimum absolute velocity in the linear direction.
   * @param v_linear_max The maximum absolute velocity in the linear direction.
   * @param v_angular_max The maximum absolute velocity in the angular direction.
   */
  void setSpeedLimit(
    const double v_linear_min, const double v_linear_max, const double v_angular_max);

  /**
   * @brief Compute linear and angular velocities command using the curvature.
   *
   * @param target Pose of the target in the robot frame.
   * @param current Current pose of the robot in the robot frame.
   * @param backward If true, the robot is moving backwards. Defaults to false.
   * @return Velocity command.
   */
  geometry_msgs::msg::Twist calculateRegularVelocity(
    const geometry_msgs::msg::Pose & target,
    const geometry_msgs::msg::Pose & current,
    const bool & backward = false);

  /**
   * @brief Compute linear and angular velocities command using the curvature.
   *
   * @param target Pose of the target in the robot frame.
   * @param backward If true, the robot is moving backwards. Defaults to false.
   * @return Velocity command.
   */
  geometry_msgs::msg::Twist calculateRegularVelocity(
    const geometry_msgs::msg::Pose & target, const bool & backward = false);

  /**
   * @brief Calculate the next pose of the robot by generating a velocity command using the
   * curvature and the current pose.
   *
   * @param dt Time step.
   * @param target Pose of the target in the robot frame.
   * @param current Current pose of the robot in the robot frame.
   * @param backward If true, the robot is moving backwards. Defaults to false.
   * @return geometry_msgs::msg::Pose
   */
  geometry_msgs::msg::Pose calculateNextPose(
    const double dt,
    const geometry_msgs::msg::Pose & target,
    const geometry_msgs::msg::Pose & current,
    const bool & backward = false);

protected:
  /**
   * @brief Calculate the path curvature using a Lyapunov-based feedback control law from
   * "A smooth control law for graceful motion" (Jong Jin Park and Benjamin Kuipers).
   *（核心曲率计算：κ = -1/r * [k_delta*(δ - atan(-k_phi*φ)) + 反馈项]）
   *
   * @param r Distance between the robot and the target.（到目标距离）
   * @param phi Orientation of target with respect to the line of sight from the robot to the target.（目标朝向相对视线角）
   * @param delta Steering angle of the robot.（机器人朝向相对视线角）
   * @return The curvature（路径曲率）
   */
  double calculateCurvature(double r, double phi, double delta);

  /**
   * @brief Ratio of the rate of change in phi to the rate of change in r. Controls the convergence
   * of the slow subsystem.
   *（慢子系统增益：φ 与 r 变化率之比）
   *
   * If this value is equal to zero, the controller will behave as a pure waypoint follower.
   * A high value offers extreme scenario of pose-following where theta is reduced much faster than r.
   *（为 0 时退化为纯航点跟随；过大时朝向收敛远快于距离收敛）
   *
   * Note: This variable is called k1 in earlier versions of the paper.（论文旧版记作 k1）
   */
  double k_phi_;

  /**
   * @brief Constant factor applied to the heading error feedback. Controls the convergence of the
   * fast subsystem.
   *（快子系统增益：朝向误差反馈系数）
   *
   * The bigger the value, the robot converge faster to the reference heading.
   *（值越大朝向收敛越快）
   *
   * Note: This variable is called k2 in earlier versions of the paper.（论文旧版记作 k2）
   */
  double k_delta_;

  /**
   * @brief Constant factor applied to the path curvature. This value must be positive.
   * Determines how fast the velocity drops when the curvature increases.
   *（速度-曲率衰减系数：曲率增大时速度下降的快慢，必须为正）
   */
  double beta_;

  /**
   * @brief Constant factor applied to the path curvature. This value must be greater or equal to 1.
   * Determines the sharpness of the curve: higher lambda implies sharper curves.
   *（速度-曲率衰减指数：越大曲线越锐利，必须 ≥ 1）
   */
  double lambda_;

  /**
   * @brief Radial threshold applied to the slowdown rule.
   *（减速半径：接近目标开始线性减速的距离）
   */
  double slowdown_radius_;

  /**
   * @brief Minimum linear velocity.（最小线速度）
   */
  double v_linear_min_;

  /**
   * @brief Maximum linear velocity.（最大线速度）
   */
  double v_linear_max_;

  /**
   * @brief Maximum angular velocity.（最大角速度）
   */
  double v_angular_max_;
};

}  // namespace nav2_graceful_controller

#endif  // NAV2_GRACEFUL_CONTROLLER__SMOOTH_CONTROL_LAW_HPP_
