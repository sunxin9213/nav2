// Copyright (c) 2024 Marc Morcos
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
// 【文件说明】get_pose_from_path_action.cpp
// 【所属模块】nav2_behavior_tree/plugins/action —— Nav2 行为树的动作节点插件
//
// 实现 GetPoseFromPath 行为树节点：从路径中按索引取出一个位姿，
// 写入黑板供其他节点使用（例如取路径终点作为 GoalReached 的比对目标，
// 或取某中间点做局部目标）。纯本地计算，不涉及 ROS2 通信。
//
// 【行为树核心概念】
// - 输入端口：path（输入路径）、index（位姿索引，支持负数：
//   -1 表示最后一个点、-2 倒数第二个，类似 Python 的负索引）；
//   输出端口：pose（取出的位姿）。
// - 分类为动作节点但单次同步完成，返回 SUCCESS（成功取出）或
//   FAILURE（空路径 / 索引越界）。
// - XML 使用示例：
//   <GetPoseFromPath path="{path}" index="-1" pose="{path_goal}"/>
// =============================================================================

#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/action/get_pose_from_path_action.hpp"

namespace nav2_behavior_tree
{

// 构造函数：无额外初始化（所有输入在 tick 中读取）。
GetPoseFromPath::GetPoseFromPath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

// tick 主函数：按索引取出位姿并输出（本地同步计算）。
inline BT::NodeStatus GetPoseFromPath::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  // 读取输入路径与目标索引。
  nav_msgs::msg::Path input_path;
  getInput("path", input_path);

  int pose_index;
  getInput("index", pose_index);

  // 空路径：无法取点，返回失败。
  if (input_path.poses.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  // Account for negative indices
  // 负索引支持：-1 转换为 size-1（最后一个点），依此类推。
  if (pose_index < 0) {
    pose_index = input_path.poses.size() + pose_index;
  }

  // out of bounds index
  // 越界检查（转换后仍为负或 >= size）：返回失败。
  if (pose_index < 0 || static_cast<unsigned>(pose_index) >= input_path.poses.size()) {
    return BT::NodeStatus::FAILURE;
  }

  // 取出指定位姿。
  geometry_msgs::msg::PoseStamped output_pose;
  output_pose = input_path.poses[pose_index];

  // populate pose frame from path if necessary
  // 若该位姿没有 frame_id（消息序列化时可能丢失），用路径头的 frame_id 补上，
  // 保证下游 tf 查询有明确的坐标系。
  if (output_pose.header.frame_id.empty()) {
    output_pose.header.frame_id = input_path.header.frame_id;
  }


  // 把位姿写入输出端口（黑板）。
  setOutput("pose", output_pose);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "GetPoseFromPath" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GetPoseFromPath>("GetPoseFromPath");
}
