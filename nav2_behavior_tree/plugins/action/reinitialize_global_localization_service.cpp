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
// 【文件说明】reinitialize_global_localization_service.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 ReinitializeGlobalLocalization 行为树节点：调用 AMCL 的
// "reinitialize_global_localization" ROS2 Service（std_srvs::srv::Empty），
// 让 AMCL 丢弃当前位姿估计，在整个地图上重新播撒粒子做全局重定位
// （机器人"迷路"后找回自己）。
//
// 【在 Nav2 导航流程中的位置】
// 属于恢复行为：当定位丢失（如被搬动、 kidnapped robot 问题）导致
// 导航持续失败时，先全局重定位再重新导航。
//
// 【行为树核心概念】
// - 继承 BtServiceNode<std_srvs::srv::Empty>：Empty 服务的请求和响应
//   都没有字段，因此构造函数和 on_tick 都无需填充任何数据，
//   只需在 XML 中指定服务名。
// - XML 使用示例：
//   <ReinitializeGlobalLocalization service_name="reinitialize_global_localization"/>
// =============================================================================

#include <string>
#include "nav2_behavior_tree/plugins/action/reinitialize_global_localization_service.hpp"

namespace nav2_behavior_tree
{

// 构造函数：委托基类创建服务客户端。
// Empty 服务无请求字段，构造体为空。
ReinitializeGlobalLocalizationService::ReinitializeGlobalLocalizationService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<std_srvs::srv::Empty>(service_node_name, conf)
{}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "ReinitializeGlobalLocalization" 标签
// 注册到 BehaviorTreeFactory（使用默认构建器）。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ReinitializeGlobalLocalizationService>(
    "ReinitializeGlobalLocalization");
}
