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
#ifndef DWB_CRITICS__GOAL_ALIGN_HPP_
#define DWB_CRITICS__GOAL_ALIGN_HPP_

#include <vector>
#include <string>
#include "dwb_critics/goal_dist.hpp"

// ============================================================================
// 模块说明：目标对齐评价器（GoalAlignCritic）
// ----------------------------------------------------------------------------
// 继承自 GoalDistCritic（目标距离评价器），核心思想一致：
//   在全局路径上找到"距离机器人最远且仍在代价地图内"的位姿，将其作为
//   临时目标点，并把该点在地图栅格上标记为最低代价值。
// 区别在于打分对象不同：
//   - GoalDistCritic    ：评估轨迹终点（机器人中心）到临时目标的距离；
//   - GoalAlignCritic   ：评估机器人前方 forward_point_distance 米处的
//     "虚拟点"到临时目标的距离。
// 通过给"机器人前方的点"打分，可以隐式约束机器人的朝向——只有当机器人
// 朝着临时目标方向前进时，其前方的点才会靠近该目标，从而获得较低代价。
// 因此它是对"机器人应该朝哪个方向看"的一种代理计算。
// ============================================================================

namespace dwb_critics
{

/**
 * @class GoalAlignCritic
 * @brief Scores trajectories based on whether the robot ends up pointing toward the eventual goal
 *
 * Similar to GoalDistCritic, this critic finds the pose from the global path farthest from the robot
 * that is still on the costmap and then evaluates how far the front of the robot is from that point.
 * This works as a proxy to calculating which way the robot should be pointing.
 */
// 目标对齐评价器：以机器人前方虚拟点到临时目标的距离作为朝向偏差的代理度量
class GoalAlignCritic : public GoalDistCritic
{
public:
  // 构造：前向点距离默认 0.0，onInit 中从参数读取
  GoalAlignCritic()
  : forward_point_distance_(0.0) {}
  // 初始化：先调用父类 GoalDistCritic::onInit，再声明 forward_point_distance 参数
  void onInit() override;
  // 每个评分周期前的准备：调用父类 prepare（生成目标距离场），并记录当前位姿
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  // 打分：将位姿沿朝向前移 forward_point_distance_ 后再查地图距离场取值
  double scorePose(const geometry_msgs::msg::Pose2D & pose) override;

protected:
  double forward_point_distance_;  // 机器人前方虚拟点到机器人中心的距离（米）
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS__GOAL_ALIGN_HPP_
