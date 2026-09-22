// Copyright (c) 2020 Fetullah Atas
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

#include "nav2_waypoint_follower/plugins/wait_at_waypoint.hpp"

#include <string>
#include <exception>

#include "pluginlib/class_list_macros.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_waypoint_follower
{
WaitAtWaypoint::WaitAtWaypoint()
: waypoint_pause_duration_(0),
  is_enabled_(true)
{
}

WaitAtWaypoint::~WaitAtWaypoint()
{
}

void WaitAtWaypoint::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  // 初始化阶段只读取插件自己的参数；WaypointFollower 会在导航成功后调用 processAtWaypoint()。
  // pause duration 的单位是毫秒，设为 0 时自动禁用等待任务，避免每个 waypoint 无意义地阻塞。
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node in wait at waypoint plugin!"};
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".waypoint_pause_duration",
    rclcpp::ParameterValue(0));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".enabled",
    rclcpp::ParameterValue(true));
  node->get_parameter(
    plugin_name + ".waypoint_pause_duration",
    waypoint_pause_duration_);
  node->get_parameter(
    plugin_name + ".enabled",
    is_enabled_);
  if (waypoint_pause_duration_ == 0) {
    is_enabled_ = false;
    RCLCPP_INFO(
      logger_,
      "Waypoint pause duration is set to zero, disabling task executor plugin.");
  } else if (!is_enabled_) {
    RCLCPP_INFO(
      logger_, "Waypoint task executor plugin is disabled.");
  }
}

bool WaitAtWaypoint::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/, const int & curr_waypoint_index)
{
  // 该插件不关心位姿，只把“到达 waypoint”转换为一个同步等待动作。
  // 返回 true 表示等待任务完成，WaypointFollower 可以继续处理下一个 waypoint。
  if (!is_enabled_) {
    return true;
  }
  RCLCPP_INFO(
    logger_, "Arrived at %i'th waypoint, sleeping for %i milliseconds",
    curr_waypoint_index,
    waypoint_pause_duration_);
  clock_->sleep_for(std::chrono::milliseconds(waypoint_pause_duration_));
  return true;
}
}  // namespace nav2_waypoint_follower
PLUGINLIB_EXPORT_CLASS(
  nav2_waypoint_follower::WaitAtWaypoint,
  nav2_core::WaypointTaskExecutor)
