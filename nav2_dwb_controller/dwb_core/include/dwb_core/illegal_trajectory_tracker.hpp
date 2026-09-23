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

#ifndef DWB_CORE__ILLEGAL_TRAJECTORY_TRACKER_HPP_
#define DWB_CORE__ILLEGAL_TRAJECTORY_TRACKER_HPP_

#include <map>
#include <utility>
#include <string>
#include "dwb_core/exceptions.hpp"
#include "nav2_core/exceptions.hpp"

namespace dwb_core
{
// 非法轨迹统计器：在每轮评分循环中统计合法/非法轨迹数量，
// 并按 (评价器名, 失败原因) 分类计数。当所有轨迹都非法时，
// 用其数据构造 NoLegalTrajectoriesException 上抛，帮助诊断
// 速度采样空间为何全军覆没
class IllegalTrajectoryTracker
{
public:
  IllegalTrajectoryTracker()
  : legal_count_(0), illegal_count_(0) {}

  void addIllegalTrajectory(const IllegalTrajectoryException & e);  // 记录一条非法轨迹
  void addLegalTrajectory();  // 记录一条合法轨迹

  // 各失败原因占全部轨迹的百分比映射
  std::map<std::pair<std::string, std::string>, double> getPercentages() const;

  std::string getMessage() const;  // 汇总消息（合法占比或全失败警告）

protected:
  // (评价器名, 异常信息) -> 出现次数
  std::map<std::pair<std::string, std::string>, unsigned int> counts_;
  unsigned int legal_count_, illegal_count_;  // 合法/非法轨迹总数
};

/**
 * @class NoLegalTrajectoriesException
 * @brief Thrown when all the trajectories explored are illegal
 * （所有候选轨迹均非法时抛出，携带统计器用于输出诊断信息。
 *  继承 PlannerException，上层 Nav2 会将其归为规划/控制失败。）
 */
class NoLegalTrajectoriesException
  : public nav2_core::PlannerException
{
public:
  explicit NoLegalTrajectoriesException(const IllegalTrajectoryTracker & tracker)
  : PlannerException(tracker.getMessage()),
    tracker_(tracker) {}
  IllegalTrajectoryTracker tracker_;  // 失败统计快照
};

}  // namespace dwb_core

#endif  // DWB_CORE__ILLEGAL_TRAJECTORY_TRACKER_HPP_
