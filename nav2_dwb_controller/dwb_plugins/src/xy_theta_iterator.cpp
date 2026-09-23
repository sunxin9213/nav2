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

#include "dwb_plugins/xy_theta_iterator.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "nav_2d_utils/parameters.hpp"
#include "nav2_util/node_utils.hpp"

#define EPSILON 1E-5

// 中文说明（文件级）：
// XYTheta 速度采样迭代器：DWB 局部规划器的核心速度采样组件。
// 在 [最小速度, 最大速度] 区间内按 vx_samples/vy_samples/vtheta_samples
// 的数量对三个自由度做离散网格采样，通过三层嵌套一维迭代器
//（先 theta、再 y、最后 x，类似里程表进位）遍历所有速度组合，
// 并跳过不满足运动学约束（超速/低速死区/全零速度）的无效组合。
// 每个有效速度组合即一条候选轨迹的目标速度。

namespace dwb_plugins
{
// 初始化：读取三个自由度的采样数量参数
void XYThetaIterator::initialize(
  const nav2_util::LifecycleNode::SharedPtr & nh,
  KinematicsHandler::Ptr kinematics,
  const std::string & plugin_name)
{
  kinematics_handler_ = kinematics;  // 保存运动学参数处理器

  // 声明并读取采样数参数：x/y 方向线速度与角速度各采多少个样本
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".vx_samples", rclcpp::ParameterValue(20));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".vy_samples", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".vtheta_samples", rclcpp::ParameterValue(20));

  nh->get_parameter(plugin_name + ".vx_samples", vx_samples_);
  nh->get_parameter(plugin_name + ".vy_samples", vy_samples_);
  nh->get_parameter(plugin_name + ".vtheta_samples", vtheta_samples_);
}

// 开始新一轮采样：以当前速度为基准，按加速度限制构建三个一维迭代器
void XYThetaIterator::startNewIteration(
  const nav_2d_msgs::msg::Twist2D & current_velocity,
  double dt)
{
  KinematicParameters kinematics = kinematics_handler_->getKinematics();
  // 每个一维迭代器在 [当前速度 - 减速*dt, 当前速度 + 加速*dt] 与
  // 绝对速度限值取交集后均匀取 vx_samples_ 个样本
  x_it_ = std::make_shared<OneDVelocityIterator>(
    current_velocity.x,
    kinematics.getMinX(), kinematics.getMaxX(),
    kinematics.getAccX(), kinematics.getDecelX(),
    dt, vx_samples_);
  y_it_ = std::make_shared<OneDVelocityIterator>(
    current_velocity.y,
    kinematics.getMinY(), kinematics.getMaxY(),
    kinematics.getAccY(), kinematics.getDecelY(),
    dt, vy_samples_);
  th_it_ = std::make_shared<OneDVelocityIterator>(
    current_velocity.theta,
    kinematics.getMinTheta(), kinematics.getMaxTheta(),
    kinematics.getAccTheta(), kinematics.getDecelTheta(),
    dt, vtheta_samples_);
  // 若初始组合无效（如全零速度），推进到首个有效组合
  if (!isValidVelocity()) {
    iterateToValidVelocity();
  }
}

// 判断某速度组合是否满足运动学约束
bool XYThetaIterator::isValidSpeed(double x, double y, double theta)
{
  KinematicParameters kinematics = kinematics_handler_->getKinematics();
  double vmag_sq = x * x + y * y;  // 合成线速度平方
  // 约束 1：合成速度不得超过最大速度
  if (kinematics.getMaxSpeedXY() >= 0.0 && vmag_sq > kinematics.getMaxSpeedXY_SQ() + EPSILON) {
    return false;
  }
  // 约束 2：速度不得落入“低速死区”（低于最小平移速度且低于最小角速度）
  if (kinematics.getMinSpeedXY() >= 0.0 && vmag_sq + EPSILON < kinematics.getMinSpeedXY_SQ() &&
    kinematics.getMinSpeedTheta() >= 0.0 && fabs(theta) + EPSILON < kinematics.getMinSpeedTheta())
  {
    return false;
  }
  // 约束 3：完全静止的组合（既不平移也不旋转）不是有效候选
  if (vmag_sq == 0.0 && th_it_->getVelocity() == 0.0) {
    return false;
  }
  return true;
}

// 判断迭代器当前指向的组合是否有效
bool XYThetaIterator::isValidVelocity()
{
  return isValidSpeed(
    x_it_->getVelocity(), y_it_->getVelocity(),
    th_it_->getVelocity());
}

// 是否还有未遍历的速度组合（以外层 x 迭代器为准）
bool XYThetaIterator::hasMoreTwists()
{
  return x_it_ && !x_it_->isFinished();
}

// 取出当前有效速度组合，并将迭代器推进到下一个有效组合
nav_2d_msgs::msg::Twist2D XYThetaIterator::nextTwist()
{
  nav_2d_msgs::msg::Twist2D velocity;
  velocity.x = x_it_->getVelocity();
  velocity.y = y_it_->getVelocity();
  velocity.theta = th_it_->getVelocity();

  // 预先推进到下一个有效组合，供下次调用直接读取
  iterateToValidVelocity();

  return velocity;
}

// 推进三层迭代器直到下一个有效速度组合（类似里程表进位：theta 最快，x 最慢）
void XYThetaIterator::iterateToValidVelocity()
{
  bool valid = false;
  while (!valid && hasMoreTwists()) {
    ++(*th_it_);  // 角速度 +1 位
    if (th_it_->isFinished()) {
      th_it_->reset();  // 角速度回卷，y 方向 +1 位
      ++(*y_it_);
      if (y_it_->isFinished()) {
        y_it_->reset();  // y 回卷，x 方向 +1 位
        ++(*x_it_);
      }
    }
    valid = isValidVelocity();  // 检查新组合是否有效
  }
}

}  // namespace dwb_plugins
