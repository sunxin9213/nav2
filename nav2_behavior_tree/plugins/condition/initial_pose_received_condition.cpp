// Copyright (c) 2019 Intel Corporation
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
// 【文件说明】initial_pose_received_condition.cpp
// 【所属模块】nav2_behavior_tree/plugins/condition —— Nav2 行为树的条件节点插件
//
// 实现 InitialPoseReceived 行为树节点：检查黑板上的
// "initial_pose_received" 布尔键——初始位姿（rviz 2D Pose Estimate 或
// 里程计）是否已被接收。这是 AMCL 启动导航的前置条件：
// 没有初始位姿，粒子滤波无法开始定位。
//
// 【行为树核心概念：简单条件（SimpleCondition）】
// - 本节点演示了 BehaviorTree.CPP 的"极简条件"写法：不需要定义类，
//   一个普通函数 + factory.registerSimpleCondition() 即可注册。
//   函数签名：BT::NodeStatus fn(BT::TreeNode&)。
// - 该函数由外部（bt_navigator 的 AMCL 相关逻辑）通过往黑板写
//   "initial_pose_received" 键来驱动。
// - XML 使用示例：
//   <InitialPoseReceived/>
// =============================================================================

#include "nav2_behavior_tree/plugins/condition/initial_pose_received_condition.hpp"

namespace nav2_behavior_tree
{

// 简单条件函数：从黑板读取 "initial_pose_received" 布尔值并映射为节点状态。
// true  -> SUCCESS（初始位姿已就绪，导航可以开始）；
// false -> FAILURE（还在等待初始位姿）。
BT::NodeStatus initialPoseReceived(BT::TreeNode & tree_node)
{
  auto initPoseReceived = tree_node.config().blackboard->get<bool>("initial_pose_received");
  return initPoseReceived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：用 registerSimpleCondition 注册"简单条件"——
// 不需要自定义类，直接绑定一个函数（内部包装为 SimpleConditionNode）。
BT_REGISTER_NODES(factory)
{
  factory.registerSimpleCondition(
    "InitialPoseReceived",
    std::bind(&nav2_behavior_tree::initialPoseReceived, std::placeholders::_1));
}
