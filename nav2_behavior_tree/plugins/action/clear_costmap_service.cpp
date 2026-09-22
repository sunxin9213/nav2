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

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/clear_costmap_service.hpp"

namespace nav2_behavior_tree
{

ClearEntireCostmapService::ClearEntireCostmapService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
// 实现该逻辑，并处理相关状态转换。
: BtServiceNode<nav2_msgs::srv::ClearEntireCostmap>(service_node_name, conf)
{
}

// 在每次 tick 时读取输入并准备下一步执行状态。
void ClearEntireCostmapService::on_tick()
{
// 实现该逻辑，并处理相关状态转换。
  increment_recovery_count();
}

ClearCostmapExceptRegionService::ClearCostmapExceptRegionService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
// 实现该逻辑，并处理相关状态转换。
: BtServiceNode<nav2_msgs::srv::ClearCostmapExceptRegion>(service_node_name, conf)
{
}

// 在每次 tick 时读取输入并准备下一步执行状态。
void ClearCostmapExceptRegionService::on_tick()
{
  getInput("reset_distance", request_->reset_distance);
// 实现该逻辑，并处理相关状态转换。
  increment_recovery_count();
}

ClearCostmapAroundRobotService::ClearCostmapAroundRobotService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
// 实现该逻辑，并处理相关状态转换。
: BtServiceNode<nav2_msgs::srv::ClearCostmapAroundRobot>(service_node_name, conf)
{
}

// 在每次 tick 时读取输入并准备下一步执行状态。
void ClearCostmapAroundRobotService::on_tick()
{
  getInput("reset_distance", request_->reset_distance);
// 实现该逻辑，并处理相关状态转换。
  increment_recovery_count();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 将当前节点类型注册到 BehaviorTreeFactory，使 XML 能直接实例化该节点。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ClearEntireCostmapService>("ClearEntireCostmap");
  factory.registerNodeType<nav2_behavior_tree::ClearCostmapExceptRegionService>(
    "ClearCostmapExceptRegion");
  factory.registerNodeType<nav2_behavior_tree::ClearCostmapAroundRobotService>(
    "ClearCostmapAroundRobot");
}
