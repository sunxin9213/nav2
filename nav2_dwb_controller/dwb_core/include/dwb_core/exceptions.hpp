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
#ifndef DWB_CORE__EXCEPTIONS_HPP_
#define DWB_CORE__EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>
#include <memory>

#include "nav2_core/exceptions.hpp"

namespace dwb_core
{

/**
 * @class PlannerTFException
 * @brief Thrown when the planner cannot complete its operation due to TF errors
 * （TF 变换失败异常：坐标变换不可用/超时时抛出，
 *  继承 PlannerException 以纳入 Nav2 统一的异常处理体系。）
 */
class PlannerTFException : public nav2_core::PlannerException
{
public:
  explicit PlannerTFException(const std::string description)
  : nav2_core::PlannerException(description) {}
};

/**
 * @class IllegalTrajectoryException
 * @brief Thrown when one of the critics encountered a fatal error
 * （非法轨迹异常：评价器判定轨迹不可行（如碰撞）时抛出。
 *  携带评价器名称，便于 IllegalTrajectoryTracker 分类统计。）
 */
class IllegalTrajectoryException : public nav2_core::PlannerException
{
public:
  IllegalTrajectoryException(const std::string critic_name, const std::string description)
  : nav2_core::PlannerException(description), critic_name_(critic_name) {}
  std::string getCriticName() const {return critic_name_;}  // 返回判非法的评价器名

protected:
  std::string critic_name_;  // 判定失败的评价器名称
};

}  // namespace dwb_core

#endif  // DWB_CORE__EXCEPTIONS_HPP_
