// Copyright (c) 2022 Neobotix GmbH
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
// 【文件说明】spin_cancel_node.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 CancelSpin 行为树节点：取消 nav2_behaviors 服务器上正在执行的
// "spin"（原地旋转）Action。继承 BtCancelActionNode<Spin>，
// 全部取消逻辑（创建客户端、发送批量取消请求、等待应答）由基类完成，
// 本类只需在注册时绑定对应的 action 类型和服务名 "spin"。
//
// 【在 Nav2 导航流程中的位置】
// 用于行为切换：例如导航被抢占（新目标到达）或恢复流程需要停止旋转时。
// 同族的取消节点还有 CancelBackUp、CancelWait、CancelDriveOnHeading、
// CancelAssistedTeleop、CancelController 等。
//
// 【XML 使用示例】
//   <CancelSpin server_name="spin"/>
// =============================================================================

#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/spin_cancel_node.hpp"

namespace nav2_behavior_tree
{

// 构造函数：委托基类 BtCancelActionNode<Spin> 创建 "spin" action 客户端；
// 基类 tick() 负责"取消该服务器指定时刻之前的所有 goal"。
SpinCancel::SpinCancel(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtCancelActionNode<nav2_msgs::action::Spin>(xml_tag_name, action_name, conf)
{
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "CancelSpin" 标签绑定到本类构建器。
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::SpinCancel>(
        name, "spin", config);
    };

  factory.registerBuilder<nav2_behavior_tree::SpinCancel>(
    "CancelSpin", builder);
}
