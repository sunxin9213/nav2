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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_graceful_controller/ego_polar_coords.hpp"
#include "nav2_graceful_controller/smooth_control_law.hpp"

namespace nav2_graceful_controller
{

// 中文说明（文件级）：
// 平滑控制律（Smooth Control Law）：Graceful Controller 的核心算法，
// 源自 Yoshihiro Kusanto 等 2021 年提出的混合 A* 后处理控制器。
// 在自极坐标系（ego-polar coordinates，见 ego_polar_coords.hpp）中
// 用非线性反馈控制律直接计算曲率与速度：
//   κ = -1/r * [k_delta*(δ - atan(-k_phi*φ)) + (1 + k_phi/(1+(k_phi*φ)²))*sin(δ)]
// 其中 r=到目标距离，φ=目标方位角，δ=朝向差。
// 该控制律可证明收敛（Lyapunov），且输出平滑无抖动；
// 速度按曲率自适应衰减 v = vmax/(1+β|κ|^λ)，接近目标再按 r/slowdown_radius
// 线性减速以消除奇点。

// 构造：保存曲率常数与速度限制
SmoothControlLaw::SmoothControlLaw(
  double k_phi, double k_delta, double beta, double lambda, double slowdown_radius,
  double v_linear_min, double v_linear_max, double v_angular_max)
: k_phi_(k_phi), k_delta_(k_delta), beta_(beta), lambda_(lambda), slowdown_radius_(slowdown_radius),
  v_linear_min_(v_linear_min), v_linear_max_(v_linear_max), v_angular_max_(v_angular_max)
{
}

// 设置曲率控制律的四个增益常数（动态参数回调时更新）
void SmoothControlLaw::setCurvatureConstants(
  double k_phi, double k_delta, double beta, double lambda)
{
  k_phi_ = k_phi;
  k_delta_ = k_delta;
  beta_ = beta;
  lambda_ = lambda;
}

// 设置减速半径（接近目标开始减速的距离）
void SmoothControlLaw::setSlowdownRadius(double slowdown_radius)
{
  slowdown_radius_ = slowdown_radius;
}

// 设置速度上下限（速度限制器回调时更新）
void SmoothControlLaw::setSpeedLimit(
  const double v_linear_min, const double v_linear_max, const double v_angular_max)
{
  v_linear_min_ = v_linear_min;
  v_linear_max_ = v_linear_max;
  v_angular_max_ = v_angular_max;
}

// 计算常规速度指令：由目标点与当前位姿得出平滑的 (v, w)
geometry_msgs::msg::Twist SmoothControlLaw::calculateRegularVelocity(
  const geometry_msgs::msg::Pose & target, const geometry_msgs::msg::Pose & current,
  const bool & backward)
{
  // Convert the target to polar coordinates
  //（把目标位姿转换到以当前位姿为基准的自极坐标 (r, φ, δ)）
  auto ego_coords = EgocentricPolarCoordinates(target, current, backward);
  // Calculate the curvature
  //（按平滑控制律计算路径曲率 κ）
  double curvature = calculateCurvature(ego_coords.r, ego_coords.phi, ego_coords.delta);
  // Invert the curvature if the robot is moving backwards
  //（倒车时曲率取反，保证几何一致性）
  curvature = backward ? -curvature : curvature;

  // Adjust the linear velocity as a function of the path curvature to
  // slowdown the controller as it approaches its target
  //（速度按曲率自适应衰减：曲率越大速度越低）
  double v = v_linear_max_ / (1.0 + beta_ * std::pow(fabs(curvature), lambda_));

  // Slowdown when the robot is near the target to remove singularity
  //（接近目标时按 r/slowdown_radius 线性减速，消除 r→0 的奇点）
  v = std::min(v_linear_max_ * (ego_coords.r / slowdown_radius_), v);

  // Set some small v_min when far away from origin to promote faster
  // turning motion when the curvature is very high
  //（夹紧到 [vmin, vmax]：高曲率时保持最小速度促进快速转向）
  v = std::clamp(v, v_linear_min_, v_linear_max_);

  // Set the velocity to negative if the robot is moving backwards
  //（倒车时速度取负）
  v = backward ? -v : v;

  // Compute the angular velocity
  //（角速度 = 曲率 * 线速度）
  double w = curvature * v;
  // Bound angular velocity between [-max_angular_vel, max_angular_vel]
  //（角速度裁剪到限值）
  double w_bound = std::clamp(w, -v_angular_max_, v_angular_max_);
  // And linear velocity to follow the curvature
  //（角速度被裁剪时，按 w/κ 反算线速度，保证轨迹仍沿期望曲率）
  v = (curvature != 0.0) ? (w_bound / curvature) : v;

  // Return the velocity command
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = v;
  cmd_vel.angular.z = w_bound;
  return cmd_vel;
}

// 无当前位姿版本：默认当前位姿为原点（用于从原点出发的仿真）
geometry_msgs::msg::Twist SmoothControlLaw::calculateRegularVelocity(
  const geometry_msgs::msg::Pose & target, const bool & backward)
{
  return calculateRegularVelocity(target, geometry_msgs::msg::Pose(), backward);
}

// 用控制律前向仿真一步：得到 dt 时间后的下一位姿（欧拉积分）
geometry_msgs::msg::Pose SmoothControlLaw::calculateNextPose(
  const double dt,
  const geometry_msgs::msg::Pose & target,
  const geometry_msgs::msg::Pose & current,
  const bool & backward)
{
  // 计算当前位姿的速度指令
  geometry_msgs::msg::Twist vel = calculateRegularVelocity(target, current, backward);
  geometry_msgs::msg::Pose next;
  double yaw = tf2::getYaw(current.orientation);
  // 沿朝向平移（差速运动学）
  next.position.x = current.position.x + vel.linear.x * dt * cos(yaw);
  next.position.y = current.position.y + vel.linear.x * dt * sin(yaw);
  // 旋转更新朝向
  yaw += vel.angular.z * dt;
  next.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  return next;
}

// 平滑控制律曲率计算（核心公式）
double SmoothControlLaw::calculateCurvature(double r, double phi, double delta)
{
  // Calculate the proportional term of the control law as the product of the gain and the error:
  // difference between the actual steering angle and the virtual control for the slow subsystem
  //（比例项：实际转向角 δ 与慢子系统虚拟控制 atan(-k_phi*φ) 之差 * 增益）
  double prop_term = k_delta_ * (delta - std::atan(-k_phi_ * phi));
  // Calculate the feedback control law for the steering
  //（反馈项：含 φ 修正的 sin(δ) 反馈，保证收敛性）
  double feedback_term = (1.0 + (k_phi_ / (1.0 + std::pow(k_phi_ * phi, 2)))) * sin(delta);
  // Calculate the path curvature
  //（合成曲率：两项之和除以距离 r 并取负）
  return -1.0 / r * (prop_term + feedback_term);
}

}  // namespace nav2_graceful_controller
