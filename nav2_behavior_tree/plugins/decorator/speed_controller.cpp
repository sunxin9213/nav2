// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

// =============================================================================
// 【文件说明】speed_controller.cpp
// 【所属模块】nav2_behavior_tree/plugins/decorator —— Nav2 行为树的装饰节点插件
//
// 实现 SpeedController（速度自适应控制器）装饰节点：根据机器人当前
// 速度动态调节子节点的执行频率——速度快时高频执行（及时响应），
// 速度慢时低频执行（省计算）。适用场景：目标点很远、路径很长时，
// 低速巡航阶段不必频繁重规划。
//
// 【频率计算模型】
// 机器人速度从 min_speed 到 max_speed 线性映射到执行频率
// min_rate ~ max_rate：
//   速度 <= min_speed -> 频率 = min_rate（最低频）
//   速度 >= max_speed -> 频率 = max_rate（最高频）
//   中间线性插值。
//
// 【BehaviorTree.CPP 与 ROS2 API 要点】
// - 装饰节点：与 RateController 类似的"周期到才 tick 子节点"逻辑，
//  但周期随速度动态更新（updatePeriod）。
// - 速度来源：黑板上的 nav2_util::OdomSmoother（里程计平滑器，
//  由 bt_navigator 放入黑板 "odom_smoother" 键），读取平滑后的速度。
// - 输入端口：min_rate/max_rate（Hz）、min_speed/max_speed（m/s）；
//  目标变化（黑板 "goal"/"goals"）时重置为最高频。
// - XML 使用示例：
//   <SpeedController min_rate="0.1" max_rate="1.0"
//                    min_speed="0.0" max_speed="0.5">
//     <ComputePathToPose .../>
//   </SpeedController>
// =============================================================================

#include <string>
#include <memory>
#include <vector>
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/decorator/speed_controller.hpp"

namespace nav2_behavior_tree
{

// 构造函数：读取频率/速度区间端口，获取里程计平滑器。
SpeedController::SpeedController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_tick_(false),
  period_(1.0),
  min_rate_(0.1),
  max_rate_(1.0),
  min_speed_(0.0),
  max_speed_(0.5)
{
  // 从黑板取共享 ROS2 节点。
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // 读取四个区间参数：执行频率下/上限（Hz）、速度下/上限（m/s）。
  getInput("min_rate", min_rate_);
  getInput("max_rate", max_rate_);
  getInput("min_speed", min_speed_);
  getInput("max_speed", max_speed_);

  // 参数合法性校验：频率必须为正，否则配置错误直接抛异常
  // （使该 BT 节点构造失败，问题在树加载阶段就暴露）。
  if (min_rate_ <= 0.0 || max_rate_ <= 0.0) {
    std::string err_msg = "SpeedController node cannot have rate <= 0.0";
    RCLCPP_FATAL(node_->get_logger(), "%s", err_msg.c_str());
    throw BT::BehaviorTreeException(err_msg);
  }

  // 预计算频率区间和速度区间的跨度，供线性插值使用。
  d_rate_ = max_rate_ - min_rate_;
  d_speed_ = max_speed_ - min_speed_;

  // 读取里程计话题名参数（默认 "odom"），
  // 并从黑板获取共享的里程计平滑器（内部订阅 odom 平滑速度）。
  std::string odom_topic;
  node_->get_parameter_or("odom_topic", odom_topic, std::string("odom"));
  odom_smoother_ = config().blackboard->get<std::shared_ptr<nav2_util::OdomSmoother>>(
    "odom_smoother");
}

// tick 主函数：按当前速度决定是否透传 tick 给子节点。
inline BT::NodeStatus SpeedController::tick()
{
  // 节点从 IDLE 启动（新一轮执行）：快照当前目标，
  // 周期重置为最高频（1/max_rate），准备开始累计。
  if (status() == BT::NodeStatus::IDLE) {
    // Reset since we're starting a new iteration of
    // the speed controller (moving from IDLE to RUNNING)
    config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", goals_);
    config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", goal_);
    period_ = 1.0 / max_rate_;
    start_ = node_->now();
    first_tick_ = true;
  }

  // 读取黑板上的当前目标。
  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  // 目标发生变化（新导航任务/新目标点）：重置周期为最高频并刷新快照
  // （新目标需要立即响应，不能用低频慢慢来）。
  if (goal_ != current_goal || goals_ != current_goals) {
    // Reset state and set period to max since we have a new goal
    period_ = 1.0 / max_rate_;
    start_ = node_->now();
    first_tick_ = true;
    goal_ = current_goal;
    goals_ = current_goals;
  }

  setStatus(BT::NodeStatus::RUNNING);

  // 计算距本轮开始以来的流逝时间。
  auto elapsed = node_->now() - start_;

  // The child gets ticked the first time through and any time the period has
  // expired. In addition, once the child begins to run, it is ticked each time
  // 'til completion
  // （透传条件：首次、子节点 RUNNING、或周期已到。）
  if (first_tick_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    elapsed.seconds() >= period_)
  {
    first_tick_ = false;

    // update period if the last period is exceeded
    // 周期已到：根据当前速度重新计算周期（速度快则周期短），
    // 并重置计时起点。
    if (elapsed.seconds() >= period_) {
      updatePeriod();
      start_ = node_->now();
    }

    // tick 子节点并透传其状态。
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  // 周期未到且子节点未在运行：保持当前状态（RUNNING）。
  return status();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "SpeedController" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SpeedController>("SpeedController");
}
