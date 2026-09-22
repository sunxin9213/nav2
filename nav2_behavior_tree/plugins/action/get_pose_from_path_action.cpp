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

#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/action/get_pose_from_path_action.hpp"

namespace nav2_behavior_tree
{

GetPoseFromPath::GetPoseFromPath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
// 初始化 ActionNodeBase 对象，读取参数并准备运行时依赖。
: BT::ActionNodeBase(name, conf)
{
}

// 初始化 NodeStatus GetPoseFromPath::tick 对象，读取参数并准备运行时依赖。
inline BT::NodeStatus GetPoseFromPath::tick()
{
// 初始化 NodeStatus::RUNNING); 对象，读取参数并准备运行时依赖。
  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::msg::Path input_path;
// 实现该逻辑，并处理相关状态转换。
  getInput("path", input_path);

  int pose_index;
// 实现该逻辑，并处理相关状态转换。
  getInput("index", pose_index);

  if (input_path.poses.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  // Account for negative indices
  if (pose_index < 0) {
    pose_index = input_path.poses.size() + pose_index;
  }

  // out of bounds index
  if (pose_index < 0 || static_cast<unsigned>(pose_index) >= input_path.poses.size()) {
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped output_pose;
  output_pose = input_path.poses[pose_index];

  // populate pose frame from path if necessary
  if (output_pose.header.frame_id.empty()) {
    output_pose.header.frame_id = input_path.header.frame_id;
  }


// 实现该逻辑，并处理相关状态转换。
  setOutput("pose", output_pose);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 将当前节点类型注册到 BehaviorTreeFactory，使 XML 能直接实例化该节点。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GetPoseFromPath>("GetPoseFromPath");
}
