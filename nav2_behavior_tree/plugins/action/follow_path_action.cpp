// Copyright (c) 2018 Intel Corporation
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
// 【文件说明】follow_path_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 FollowPath 行为树节点：调用 nav2_controller 的
// "follow_path" ROS2 Action（nav2_msgs::action::FollowPath），
// 让控制器（DWB/MPPI/RPP 等）跟踪黑板上的全局路径，
// 持续向速度平滑器发布速度指令直到到达目标。
//
// 【在 Nav2 导航流程中的位置】
// 典型导航树中位于 ComputePathToPose 之后：读取黑板 "path" 键开始跟踪；
// 由于跟踪是长时间动作（返回 RUNNING），常用 RateController/SpeedController
// 装饰器控制其上游重规划频率。
//
// 【行为树核心概念】
// - 输入端口：path（要跟踪的路径）、controller_id（控制器选择，如
//   "FollowPath"）、goal_checker_id（目标到达判定器选择）。
// - 本类的亮点是 on_wait_for_result 钩子：动作运行期间持续监测黑板上的
//   路径/控制器 ID 是否变化，变化则更新 goal_ 并置 goal_updated_=true，
//   基类 tick() 会在下个周期自动重发新 goal——实现"边走边重新规划"的
//   运行时目标更新，无需先取消再重发。
// - XML 使用示例：
//   <FollowPath path="{path}" controller_id="FollowPath"/>
// =============================================================================

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/follow_path_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：委托基类创建 "follow_path" action 客户端，无额外初始化。
FollowPathAction::FollowPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::FollowPath>(xml_tag_name, action_name, conf)
{
}

// on_tick 钩子：动作开始前读取输入端口填充 goal_。
void FollowPathAction::on_tick()
{
  // 要跟踪的全局路径（通常来自黑板 "path" 键，即规划结果）。
  getInput("path", goal_.path);
  // 控制器 ID：XML 中可动态选择用哪个控制器插件执行跟踪。
  getInput("controller_id", goal_.controller_id);
  // 目标到达判定器 ID：决定"到达目标"的判据（容差、速度阈值等）。
  getInput("goal_checker_id", goal_.goal_checker_id);
}

// on_wait_for_result 钩子：跟踪过程中每个等待周期被调用（feedback 为
// 服务器反馈，本节点不使用）；用于检测输入变化并触发 goal 更新。
void FollowPathAction::on_wait_for_result(
  std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback>/*feedback*/)
{
  // ---- 检测路径是否被上游更新（重新规划出了新路径）----
  nav_msgs::msg::Path new_path;
  getInput("path", new_path);

  // Check if it is not same with the current one
  // 新路径存在（非空）且与当前不同：更新 goal 并标记 goal_updated_，
  // the action server on the next loop iteration
  // 基类 tick() 检测到该标志后会向服务器重发新 goal。
  if (goal_.path != new_path && new_path != nav_msgs::msg::Path()) {
    goal_.path = new_path;
    goal_updated_ = true;
  }

  // ---- 检测控制器 ID 是否变化（运行中切换控制器）----
  std::string new_controller_id;
  getInput("controller_id", new_controller_id);

  if (goal_.controller_id != new_controller_id) {
    goal_.controller_id = new_controller_id;
    goal_updated_ = true;
  }

  // ---- 检测目标判定器 ID 是否变化 ----
  std::string new_goal_checker_id;
  getInput("goal_checker_id", new_goal_checker_id);

  if (goal_.goal_checker_id != new_goal_checker_id) {
    goal_.goal_checker_id = new_goal_checker_id;
    goal_updated_ = true;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "FollowPath" 标签绑定到本类构建器。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::FollowPathAction>(
        name, "follow_path", config);
    };

  factory.registerBuilder<nav2_behavior_tree::FollowPathAction>(
    "FollowPath", builder);
}
