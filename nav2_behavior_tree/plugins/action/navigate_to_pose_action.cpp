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
// 【文件说明】navigate_to_pose_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 NavigateToPose 行为树节点：调用 bt_navigator 的
// "navigate_to_pose" ROS2 Action（nav2_msgs::action::NavigateToPose），
// 让机器人导航到单个目标点。这是 Nav2 最顶层的导航入口节点，
// 通常位于导航树的根/主分支，整棵子树的执行就对应一次导航任务。
//
// 【行为树核心概念】
// - 本节点继承 BtActionNode<NavigateToPose>（动作节点基类，见
//   bt_action_node.hpp），tick 语义（发送 goal -> RUNNING -> 结果钩子）
//   全部由基类实现，本类只负责端口到 goal 字段的映射。
// - 端口：goal（输入，目标位姿 PoseStamped）、behavior_tree（输入，
//   可选的自定义子树 XML 路径，让 bt_navigator 加载自定义导航树）。
// - XML 使用示例：
//   <NavigateToPose goal="{goal}" behavior_tree="my_tree.xml"/>
//
// 【注册机制】文件末尾的 BT_REGISTER_NODES 宏导出 C 函数，
// BehaviorTreeFactory 动态加载本 .so 时调用它，把 "NavigateToPose"
// 标签绑定到本类的构建器；之后 XML 中的 <NavigateToPose .../> 才能被
// 实例化。Groot 亦据此显示节点端口（见 nav2_tree_nodes.xml）。
// =============================================================================

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：直接委托给基类 BtActionNode<NavigateToPose>，
// 基类会创建 "navigate_to_pose" action 客户端并等待服务器上线；
// 本类无需额外初始化（goal 字段在 on_tick 中填充）。
NavigateToPoseAction::NavigateToPoseAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
}

// on_tick 钩子：每次动作开始前从端口读取输入并填充 goal_。
void NavigateToPoseAction::on_tick()
{
  // 读取目标位姿到 goal_.pose；读取失败（端口未连接/黑板无值）时
  // 记错误日志并直接返回——保留上次的 goal（通常会导致导航到旧目标）。
  if (!getInput("goal", goal_.pose)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "NavigateToPoseAction: goal not provided");
    return;
  }
  // 可选端口：允许指定自定义行为树 XML 文件，
  // bt_navigator 将用它替换默认导航树来执行本次任务。
  getInput("behavior_tree", goal_.behavior_tree);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库被加载时调用，向 factory 注册构建器。
BT_REGISTER_NODES(factory)
{
  // 构建器 lambda：XML 实例化节点时被调用，
  // 固定传入 action 服务名 "navigate_to_pose"。
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::NavigateToPoseAction>(
        name, "navigate_to_pose", config);
    };

  // 把 XML 标签名 "NavigateToPose" 与构建器绑定注册。
  factory.registerBuilder<nav2_behavior_tree::NavigateToPoseAction>(
    "NavigateToPose", builder);
}
