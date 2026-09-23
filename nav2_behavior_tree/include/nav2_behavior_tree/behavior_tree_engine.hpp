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
// 【文件说明】behavior_tree_engine.hpp
// 【所属模块】nav2_behavior_tree —— Nav2 的行为树核心模块
//
// 本文件声明了 BehaviorTreeEngine 类——行为树引擎，是 Nav2 加载和运行
// 行为树的入口。
//
// 【在 Nav2 导航系统中的位置和作用】
// nav2_bt_navigator 等顶层节点通过本引擎完成三件事：
//   1. 构造时动态加载全部 BT 插件库（.so，每个插件库通过
//      BT_REGISTER_NODES 宏向 factory 注册节点类型）；
//   2. 从 XML 文本/文件创建行为树（createTreeFromText/File）；
//   3. 以固定周期驱动树执行（run）：每轮 tick 根节点一次，直到树返回
//      SUCCESS/FAILURE，或外部请求取消（cancelRequested 返回 true）。
//
// 【BehaviorTree.CPP 核心概念】
// - BT::BehaviorTreeFactory：节点类型的"注册表 + 树工厂"。插件库被
//   BT::SharedLibrary 动态加载后，其中的注册函数把节点构建器登记进
//   factory；之后 factory 才能按 XML 中的标签名实例化节点。
// - BT::Tree：一棵已实例化的树；调用 tree.rootNode()->executeTick()
//   即执行一个周期。
// - 树是"状态机 + 递归 tick"模型：控制节点（Sequence/Fallback/自定义）
//   按自身规则 tick 子节点并聚合出整树状态。
// - BtStatus 是 Nav2 对树执行结果的高层封装（成功/失败/取消），
//   供 BtActionServer 转换为 ROS2 action 结果。
// =============================================================================

#ifndef NAV2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
#define NAV2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @enum nav2_behavior_tree::BtStatus
 * @brief An enum class representing BT execution status
 *        （行为树整体执行状态枚举：成功 / 失败 / 被取消）
 */
enum class BtStatus { SUCCEEDED, FAILED, CANCELED };

/**
 * @class nav2_behavior_tree::BehaviorTreeEngine
 * @brief A class to create and handle behavior trees
 *        （行为树引擎：加载插件、建树、驱动执行）
 */
class BehaviorTreeEngine
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::BehaviorTreeEngine
   *        （构造函数：逐个动态加载 BT 插件库，把节点类型注册进 factory）
   * @param plugin_libraries vector of BT plugin library names to load
   *        （要加载的插件库名称列表，如 "nav2_compute_path_to_pose_action_bt_node"）
   */
  explicit BehaviorTreeEngine(
    const std::vector<std::string> & plugin_libraries);
  virtual ~BehaviorTreeEngine() {}

  /**
   * @brief Function to execute a BT at a specific rate
   *        （主执行循环：周期性 tick 整棵树直到结束或被取消）
   * @param tree BT to execute
   *        （要执行的行为树）
   * @param onLoop Function to execute on each iteration of BT execution
   *        （每轮迭代附加执行的回调，如发布反馈、检查抢占）
   * @param cancelRequested Function to check if cancel was requested during BT execution
   *        （取消检查函数：返回 true 时中断执行并返回 CANCELED）
   * @param loopTimeout Time period for each iteration of BT execution
   *        （每轮迭代的时间预算，默认 10ms，超时仅记录警告不终止）
   * @return nav2_behavior_tree::BtStatus Status of BT execution
   *        （树执行结果：SUCCEEDED / FAILED / CANCELED）
   */
  BtStatus run(
    BT::Tree * tree,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

  /**
   * @brief Function to create a BT from a XML string
   *        （从 XML 字符串创建行为树）
   * @param xml_string XML string representing BT
   *        （描述树结构的 XML 文本）
   * @param blackboard Blackboard for BT
   *        （树使用的黑板（通常复用旧黑板，保留 goal 等已有数据））
   * @return BT::Tree Created behavior tree
   */
  BT::Tree createTreeFromText(
    const std::string & xml_string,
    BT::Blackboard::Ptr blackboard);

  /**
   * @brief Function to create a BT from an XML file
   *        （从 XML 文件创建行为树：读文件后转调 createTreeFromText）
   * @param file_path Path to BT XML file
   *        （XML 文件路径）
   * @param blackboard Blackboard for BT
   *        （树使用的黑板）
   * @return BT::Tree Created behavior tree
   */
  BT::Tree createTreeFromFile(
    const std::string & file_path,
    BT::Blackboard::Ptr blackboard);

  /**
   * @brief Function to explicitly reset all BT nodes to initial state
   *        （递归 halt 整棵树：把所有节点复位到初始 IDLE 状态，
   *         触发各节点 halt() 清理逻辑（如取消进行中的 action））
   * @param root_node Pointer to BT root node
   *        （树的根节点）
   */
  void haltAllActions(BT::TreeNode * root_node);

protected:
  // 节点工厂：持有全部已注册节点类型，负责按 XML 实例化树。
  BT::BehaviorTreeFactory factory_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
