// Copyright (c) 2022 Joshua Wallace
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
// 【文件说明】assisted_teleop_cancel_node.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 CancelAssistedTeleop 行为树节点：取消 nav2_behaviors 服务器上
// 正在执行的 "assisted_teleop"（辅助遥控）Action。
// 继承 BtCancelActionNode<AssistedTeleop>，取消逻辑全部由基类实现
// （见 bt_cancel_action_node.hpp），本类只在注册时绑定 Action 类型
// AssistedTeleop 和服务名 "assisted_teleop"。
//
// 【在 Nav2 导航流程中的位置】
// 用于结束辅助遥控模式、恢复自主导航。
//
// 【XML 使用示例】
//   <CancelAssistedTeleop server_name="assisted_teleop"/>
// =============================================================================

#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/assisted_teleop_cancel_node.hpp"

namespace nav2_behavior_tree
{

// 构造函数：委托基类创建 "assisted_teleop" action 客户端；
// 基类 tick() 负责发送批量取消请求并等待应答。
AssistedTeleopCancel::AssistedTeleopCancel(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtCancelActionNode<nav2_msgs::action::AssistedTeleop>(xml_tag_name, action_name, conf)
{
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "CancelAssistedTeleop" 标签绑定到本类构建器。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::AssistedTeleopCancel>(
        name, "assisted_teleop", config);
    };

  factory.registerBuilder<nav2_behavior_tree::AssistedTeleopCancel>(
    "CancelAssistedTeleop", builder);
}
