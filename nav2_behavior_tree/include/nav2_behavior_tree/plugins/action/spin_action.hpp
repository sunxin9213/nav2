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
// 【文件说明】spin_action.hpp
// 【所属模块】nav2_behavior_tree —— Spin（原地旋转）行为树节点的类声明
//
// SpinAction 封装 nav2_msgs::action::Spin：让机器人原地旋转指定弧度，
// 是典型的导航恢复行为。实现见同目录 spin_action.cpp。
// 端口：spin_dist（弧度，默认 1.57 即 90 度）、time_allowance（秒）、
// is_recovery（是否为恢复行为）。私有成员 is_recovery_ 缓存该标志。
// XML 示例：<Spin spin_dist="1.57" time_allowance="10" is_recovery="true"/>
// =============================================================================

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SPIN_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SPIN_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::Spin
 *        （封装 Spin action 的 BT 动作节点）
 */
class SpinAction : public BtActionNode<nav2_msgs::action::Spin>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::SpinAction
   * @param xml_tag_name Name for the XML tag for this node
   *        （XML 标签名）
   * @param action_name Action name this node creates a client for
   *        （要连接的 action 服务名）
   * @param conf BT node configuration
   *        （BT 节点配置，含黑板）
   */
  SpinAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   *        （tick 钩子：is_recovery_ 为真时递增恢复计数，见 .cpp 实现）
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   *        （端口声明：基础端口 + 本节点端口）
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        // 输入端口：旋转角度（弧度），默认 1.57（约 90 度）。
        BT::InputPort<double>("spin_dist", 1.57, "Spin distance"),
        // 输入端口：允许的最长执行时间（秒），防止动作卡死。
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for spinning"),
        // 输入端口：是否作为恢复行为（默认 true），
        // 决定是否递增黑板 number_recoveries 计数。
        BT::InputPort<bool>("is_recovery", true, "True if recovery")
      });
  }

private:
  // 缓存"是否为恢复行为"标志（构造时从端口读取）。
  bool is_recovery_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SPIN_ACTION_HPP_
