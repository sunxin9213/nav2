// Copyright (c) 2020 Samsung Research America
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

#include "nav2_waypoint_follower/plugins/input_at_waypoint.hpp"

#include <string>
#include <exception>

#include "pluginlib/class_list_macros.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_waypoint_follower
{

using std::placeholders::_1;

InputAtWaypoint::InputAtWaypoint()
: input_received_(false),
  is_enabled_(true),
  timeout_(10.0, 0.0)
{
}

InputAtWaypoint::~InputAtWaypoint()
{
}

void InputAtWaypoint::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  // 该插件把外部 Empty 消息当作“允许离开当前 waypoint”的同步信号。
  // timeout 使用秒，防止外部设备或人工确认永远不来时阻塞整个 waypoint 序列。
  auto node = parent.lock();

  if (!node) {
    throw std::runtime_error{"Failed to lock node in input at waypoint plugin!"};
  }

  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double timeout;
  std::string input_topic;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".timeout",
    rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".enabled",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".input_topic",
    rclcpp::ParameterValue("input_at_waypoint/input"));
  node->get_parameter(plugin_name + ".timeout", timeout);
  node->get_parameter(plugin_name + ".enabled", is_enabled_);
  node->get_parameter(plugin_name + ".input_topic", input_topic);

  timeout_ = rclcpp::Duration(timeout, 0.0);

  RCLCPP_INFO(
    logger_, "InputAtWaypoint: Subscribing to input topic %s.", input_topic.c_str());
  subscription_ = node->create_subscription<std_msgs::msg::Empty>(
    input_topic, 1, std::bind(&InputAtWaypoint::Cb, this, _1));
}

void InputAtWaypoint::Cb(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  // 回调只设置标志位，不直接结束 processAtWaypoint()；
  // 主线程会在下一次轮询时读取该标志，避免跨线程直接操作任务控制流。
  std::lock_guard<std::mutex> lock(mutex_);
  input_received_ = true;
}

bool InputAtWaypoint::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
  const int & curr_waypoint_index)
{
  // 到达 waypoint 后清除上一点的输入状态，再以固定频率等待新的外部确认。
  // 收到消息返回 true；超时返回 false，由 WaypointFollower 的 stop_on_failure 策略决定后续行为。
  if (!is_enabled_) {
    return true;
  }

  input_received_ = false;

  rclcpp::Time start = clock_->now();
  rclcpp::Rate r(50);
  bool input_received = false;
  while (clock_->now() - start < timeout_) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      input_received = input_received_;
    }

    if (input_received) {
      return true;
    }

    r.sleep();
  }

  RCLCPP_WARN(
    logger_, "Unable to get external input at wp %i. Moving on.", curr_waypoint_index);
  return false;
}

}  // namespace nav2_waypoint_follower

PLUGINLIB_EXPORT_CLASS(
  nav2_waypoint_follower::InputAtWaypoint,
  nav2_core::WaypointTaskExecutor)
