// Copyright (c) 2019 Samsung Research America
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
// 【文件说明】clear_costmap_service.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 本文件实现三个"清代价地图"服务节点（均继承 BtServiceNode，调用 ROS2
// Service 而非 Action）：
//   1. ClearEntireCostmap       —— 清空整个代价地图；
//   2. ClearCostmapExceptRegion —— 清空机器人周围指定半径以外的区域
//                                   （保留机器人近旁的障碍信息）；
//   3. ClearCostmapAroundRobot  —— 清空机器人周围指定半径以内的区域
//                                   （清掉近旁可能过期的障碍）。
//
// 【在 Nav2 导航流程中的位置】
// 三个节点都是典型恢复行为：规划失败时常常是因为代价地图里残留了过期的
// 障碍（如已离开的人/物），清图后重新规划往往能成功。它们出现在默认导航
// 树的 RecoveryBranch 中。
//
// 【行为树核心概念】
// - 服务节点语义：调用一次服务，等待应答（SUCCESS/FAILURE），
//   没有反馈流（区别于 Action 节点）。
// - 服务名端口 service_name 必须在 XML 中指定，例如：
//   <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
//   <ClearCostmapAroundRobot service_name="local_costmap/clear_around_local_costmap"
//                            reset_distance="1.0"/>
// - 三个节点都在 on_tick 中递增黑板 number_recoveries（恢复行为计数）。
// =============================================================================

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/clear_costmap_service.hpp"

namespace nav2_behavior_tree
{

// ---- ClearEntireCostmap：清空整个代价地图 ----
// 构造函数：委托基类创建服务客户端（服务名由 XML 端口 service_name 提供）。
ClearEntireCostmapService::ClearEntireCostmapService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::ClearEntireCostmap>(service_node_name, conf)
{
}

// on_tick 钩子：无需填充请求字段（Empty 请求），只递增恢复计数。
void ClearEntireCostmapService::on_tick()
{
  // 清图属于恢复行为：递增黑板 number_recoveries。
  increment_recovery_count();
}

// ---- ClearCostmapExceptRegion：清空机器人周围 reset_distance 以外的区域 ----
ClearCostmapExceptRegionService::ClearCostmapExceptRegionService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::ClearCostmapExceptRegion>(service_node_name, conf)
{
}

// on_tick 钩子：读取保留半径并递增恢复计数。
void ClearCostmapExceptRegionService::on_tick()
{
  // reset_distance：机器人近旁保留障碍信息的半径（米），
  // 半径以外的障碍全部清除。
  getInput("reset_distance", request_->reset_distance);
  increment_recovery_count();
}

// ---- ClearCostmapAroundRobot：清空机器人周围 reset_distance 以内的区域 ----
ClearCostmapAroundRobotService::ClearCostmapAroundRobotService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::ClearCostmapAroundRobot>(service_node_name, conf)
{
}

// on_tick 钩子：读取清除半径并递增恢复计数。
void ClearCostmapAroundRobotService::on_tick()
{
  // reset_distance：机器人周围要清除障碍的半径（米），
  // 用于剔除近旁过期的虚假障碍。
  getInput("reset_distance", request_->reset_distance);
  increment_recovery_count();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：一个动态库注册三个节点类型
// （registerNodeType 使用默认构建器，适合构造签名标准的节点）。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ClearEntireCostmapService>("ClearEntireCostmap");
  factory.registerNodeType<nav2_behavior_tree::ClearCostmapExceptRegionService>(
    "ClearCostmapExceptRegion");
  factory.registerNodeType<nav2_behavior_tree::ClearCostmapAroundRobotService>(
    "ClearCostmapAroundRobot");
}
