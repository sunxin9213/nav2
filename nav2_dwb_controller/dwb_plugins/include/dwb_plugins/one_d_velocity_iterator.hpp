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

#ifndef DWB_PLUGINS__ONE_D_VELOCITY_ITERATOR_HPP_
#define DWB_PLUGINS__ONE_D_VELOCITY_ITERATOR_HPP_

#include <algorithm>
#include <cmath>

namespace dwb_plugins
{

const double EPSILON = 1E-5;  // 浮点比较容差

/**
 * @brief Given initial conditions and a time, figure out the end velocity
 *
 * @param v0 Initial velocity
 * @param accel The acceleration rate
 * @param decel The decceleration rate
 * @param dt Delta time - amount of time to project into the future
 * @param target target velocity
 * @return The velocity dt seconds after v0.
 */
// 速度前瞻：按匀加速模型把初速度 v0 推演 dt 秒后的速度。
// 朝 target 加速时用 accel（其绝对值），背离时用 decel，
// 且不会越过 target（一步到 target 即止）
inline double projectVelocity(double v0, double accel, double decel, double dt, double target)
{
  double v1;
  if (v0 < target) {
    v1 = v0 + accel * dt;   // 需要加速：v0 向上逼近 target
    return std::min(target, v1);  // 不越过 target
  } else {
    v1 = v0 + decel * dt;   // 需要减速：decel 通常为负
    return std::max(target, v1);  // 不低于 target
  }
}

/**
 * @class OneDVelocityIterator
 * @brief An iterator for generating a number of samples in a range
 *
 * In its simplest usage, this gives us N (num_samples) different velocities that are reachable
 * given our current velocity. However, there is some fancy logic around zero velocities and
 * the min/max velocities
 *
 * If the current velocity is 2 m/s, and the acceleration limit is 1 m/ss and the acc_time is 1 s,
 * this class would provide velocities between 1 m/s and 3 m/s.
 *
 *
 *
 */
// 一维速度采样迭代器：DWB 速度空间采样的基础构件。
// 给定当前速度与加速度限制，枚举时间窗 acc_time 内可达的 num_samples 个
// 速度值（均匀间隔）。特殊逻辑：确保 0 速度一定出现在采样序列中，
// 使机器人始终有"停下来"这个候选选项
class OneDVelocityIterator
{
public:
  /**
   * @brief Constructor for the velocity iterator
   *
   * @param current Current velocity
   * @param min Minimum velocity allowable
   * @param max Maximum velocity allowable
   * @param acc_limit Acceleration Limit
   * @param decel_limit Deceleration Limit
   * @param num_samples The number of samples to return
   */
  OneDVelocityIterator(
    double current, double min, double max, double acc_limit, double decel_limit, double acc_time,
    int num_samples)
  {
    // 当前速度先裁剪到 [min, max] 合法范围内
    if (current < min) {
      current = min;
    } else if (current > max) {
      current = max;
    }
    // 分别向 max/min 两个方向做时间窗内可达性推演，得到采样上下界
    max_vel_ = projectVelocity(current, acc_limit, decel_limit, acc_time, max);
    min_vel_ = projectVelocity(current, acc_limit, decel_limit, acc_time, min);
    reset();

    // 上下界几乎重合：退化为单点迭代（步长设 1 使 isFinished 立即判定）
    if (fabs(min_vel_ - max_vel_) < EPSILON) {
      increment_ = 1.0;
      return;
    }
    num_samples = std::max(2, num_samples);  // 至少 2 个样本

    // e.g. for 4 samples, split distance in 3 even parts
    // 均匀步长：区间长度 / (num_samples - 1)
    increment_ = (max_vel_ - min_vel_) / std::max(1, (num_samples - 1));
  }

  /**
   * @brief Get the next velocity available
   */
  double getVelocity() const
  {
    // 本次迭代临时返回 0（保证"停止"候选被枚举到）
    if (return_zero_now_) {return 0.0;}
    return current_;
  }

  /**
   * @brief Increment the iterator
   */
  OneDVelocityIterator & operator++()
  {
    // 跨越零点的时机插入一个精确的 0 速度样本（仅一次）
    if (return_zero_ && current_ < 0.0 && current_ + increment_ > 0.0 &&
      current_ + increment_ <= max_vel_ + EPSILON)
    {
      return_zero_now_ = true;   // 本次取值为 0
      return_zero_ = false;      // 后续不再插入
    } else {
      current_ += increment_;    // 正常前进一个步长
      return_zero_now_ = false;
    }
    return *this;
  }

  /**
   * @brief Reset back to the first velocity
   */
  void reset()
  {
    current_ = min_vel_;     // 从下界开始
    return_zero_ = true;     // 允许插入零速度样本
    return_zero_now_ = false;
  }

  /**
   * If we have returned all the velocities for this iteration
   */
  bool isFinished() const
  {
    return current_ > max_vel_ + EPSILON;  // 超过上界即遍历结束
  }

private:
  bool return_zero_, return_zero_now_;  // 零速度插入标志（计划中/本次生效）
  double min_vel_, max_vel_;            // 本轮采样可达的下/上速度界
  double current_;                      // 当前采样值
  double increment_;                    // 相邻样本的步长
};
}  // namespace dwb_plugins

#endif  // DWB_PLUGINS__ONE_D_VELOCITY_ITERATOR_HPP_
