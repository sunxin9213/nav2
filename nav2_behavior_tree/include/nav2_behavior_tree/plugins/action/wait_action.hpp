// Copyright (c) 2018 Samsung Research America
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
// 【文件说明】wait_action.hpp
// 【所属模块】nav2_behavior_tree —— Wait（等待）行为树节点的类声明
//
// WaitAction 封装 nav2_msgs::action::Wait：让机器人原地等待指定秒数，
// 常用于导航恢复策略（等待临时障碍物离开）。实现见同目录
// wait_action.cpp。端口：wait_duration（等待时长，秒，默认 1）。
// XML 示例：<Wait wait_duration="5"/>
// =============================================================================

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/wait.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::Wait
 *        （封装 Wait action 的 BT 动作节点）
 */
class WaitAction : public BtActionNode<nav2_msgs::action::Wait>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::WaitAction
   * @param xml_tag_name Name for the XML tag for this node
   *        （XML 标签名）
   * @param action_name Action name this node creates a client for
   *        （要连接的 action 服务名）
   * @param conf BT node configuration
   *        （BT 节点配置，含黑板）
   */
  WaitAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   *        （tick 钩子：每次动作开始时递增恢复计数，见 .cpp 实现）
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   *        （端口声明：基础端口 server_name/server_timeout + 本节点端口）
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        // 输入端口：等待时长（秒，int），XML 中可省略（默认 1 秒）。
        BT::InputPort<int>("wait_duration", 1, "Wait time")
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_ACTION_HPP_
