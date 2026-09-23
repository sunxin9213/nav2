// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Florian Gramss
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
// 【文件说明】behavior_tree_engine.cpp
// 【所属模块】nav2_behavior_tree —— Nav2 的行为树核心模块
//
// 本文件实现 BehaviorTreeEngine（行为树引擎），是 Nav2 加载和驱动行为树
// 执行的核心。类的职责说明见 behavior_tree_engine.hpp 文件头。
//
// 【在 Nav2 导航系统中的位置和作用】
// 三个职责：
//   1. 构造函数：通过 BT::SharedLibrary 动态加载插件库（.so），
//      每个库的 BT_REGISTER_NODES 宏把节点构建器注册进 factory_；
//   2. run()：BT 执行主循环——按固定周期 tick 树根，直到树返回
//      SUCCESS/FAILURE 或外部取消；这是整个导航任务的"心跳"；
//   3. createTreeFromText/File + haltAllActions：建树与复位。
// =============================================================================

#include "nav2_behavior_tree/behavior_tree_engine.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/utils/shared_library.h"

namespace nav2_behavior_tree
{

// ---- 构造函数：动态加载全部 BT 插件库并注册节点类型 ----
BehaviorTreeEngine::BehaviorTreeEngine(
  const std::vector<std::string> & plugin_libraries)
{
  // BT::SharedLibrary 是 BehaviorTree.CPP 对 dlopen/LoadLibrary 的跨平台封装。
  BT::SharedLibrary loader;
  // 逐个加载插件库：getOSName() 把库名（如 "nav2_wait_action_bt_node"）
  // 补全为平台相关文件名（libXXX.so / XXX.dll）；
  // registerFromPlugin() 会调用库内导出的 BT_REGISTER_NODES 注册函数，
  // 把该库所有节点类型（Builder）登记进 factory_，
  // 之后 createTreeFromText 才能按 XML 标签实例化它们。
  for (const auto & p : plugin_libraries) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }
}

// ---- 主执行循环：周期性 tick 树直到结束/取消/异常 ----
BtStatus
BehaviorTreeEngine::run(
  BT::Tree * tree,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  // 壁钟节拍器：保证循环以 loopTimeout 为周期执行
  // （tick 太快会空转浪费 CPU，太慢则响应不及时）。
  rclcpp::WallRate loopRate(loopTimeout);
  // 初始状态为 RUNNING：进入循环至少 tick 一次。
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // Loop until something happens with ROS or the node completes
  try {
    // 循环条件：ROS2 正常运行 且 根节点仍返回 RUNNING。
    while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
      // 每轮先检查取消：外部（action 客户端发了取消请求）要求中断时，
      // halt 根节点（递归清理所有 RUNNING 节点）并返回 CANCELED。
      if (cancelRequested()) {
        tree->rootNode()->halt();
        return BtStatus::CANCELED;
      }

      // tick 根节点一个周期：整棵树按控制节点规则递归执行。
      result = tree->tickRoot();

      // 执行用户的每周期回调（发布反馈、检查抢占、刷新日志等）。
      onLoop();

      // 按节拍休眠；sleep() 返回 false 表示本轮实际耗时已超过周期
      // （某节点单次 tick 阻塞过久），记录警告提示调大 bt_loop_duration。
      if (!loopRate.sleep()) {
        RCLCPP_WARN(
          rclcpp::get_logger("BehaviorTreeEngine"),
          "Behavior Tree tick rate %0.2f was exceeded!",
          1.0 / (loopRate.period().count() * 1.0e-9));
      }
    }
  } catch (const std::exception & ex) {
    // 树执行中抛出异常（如 BT 节点内部的逻辑错误）：
    // 记录错误并按失败处理，不让异常逃逸导致整个服务器崩溃。
    RCLCPP_ERROR(
      rclcpp::get_logger("BehaviorTreeEngine"),
      "Behavior tree threw exception: %s. Exiting with failure.", ex.what());
    return BtStatus::FAILED;
  }

  // 循环结束：根节点 SUCCESS 映射为 SUCCEEDED，其余（FAILURE）映射为 FAILED。
  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

// ---- 从 XML 文本创建行为树（委托给 BehaviorTreeFactory）----
BT::Tree
BehaviorTreeEngine::createTreeFromText(
  const std::string & xml_string,
  BT::Blackboard::Ptr blackboard)
{
  // factory 按 XML 标签名查找已注册的节点构建器，逐个实例化并组装成树；
  // blackboard 由调用方传入，便于在换树时保留旧数据（goal、路径等）。
  return factory_.createTreeFromText(xml_string, blackboard);
}

// ---- 从 XML 文件创建行为树 ----
BT::Tree
BehaviorTreeEngine::createTreeFromFile(
  const std::string & file_path,
  BT::Blackboard::Ptr blackboard)
{
  // 与 createTreeFromText 等价，只是输入是文件路径。
  return factory_.createTreeFromFile(file_path, blackboard);
}

// In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
// （要重复运行一棵行为树，必须能把所有节点复位到初始状态）
void
BehaviorTreeEngine::haltAllActions(BT::TreeNode * root_node)
{
  // 空指针保护：树不存在时直接返回。
  if (!root_node) {
    return;
  }

  // 先 halt 根节点：BT 的 halt() 本应递归传播到子节点。
  root_node->halt();

  // but, just in case...
  // 保险起见：递归遍历全树，对任何仍处于 RUNNING 状态的节点
  // 单独再 halt 一次（防止个别控制节点未正确传播 halt）。
  auto visitor = [](BT::TreeNode * node) {
      if (node->status() == BT::NodeStatus::RUNNING) {
        node->halt();
      }
    };
  BT::applyRecursiveVisitor(root_node, visitor);
}

}  // namespace nav2_behavior_tree
