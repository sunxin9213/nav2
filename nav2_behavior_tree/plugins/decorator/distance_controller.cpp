// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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
// 【文件说明】distance_controller.cpp
// 【所属模块】nav2_behavior_tree/plugins/decorator —— Nav2 行为树的装饰节点插件
//
// 实现 DistanceController（距离控制器）装饰节点：按"机器人行驶距离"
// 限制子节点执行频率——机器人每行驶 distance 米，子节点才被 tick 一次；
// 期间保持状态不变。与 RateController（按时间限频）对应，本节点按空间
// 距离限频。
//
// 【在 Nav2 导航流程中的位置】
// 可用于"每前进 X 米做一次某动作"的场景，如周期性地图更新、
// 里程触发的重定位校验等。
//
// 【BehaviorTree.CPP 与 ROS2 API 要点】
// - 装饰节点（BT::DecoratorNode），单个子节点。
// - 通过 tf2（黑板 "tf_buffer"）查询机器人当前位姿，与起始位姿做
//   欧氏距离比较；子节点成功后把起始位姿重置为当前位姿
//   （重新累计下一段距离）。
// - 输入端口：distance（触发距离，米）、global_frame、robot_base_frame。
// - XML 使用示例：
//   <DistanceController distance="1.0"><SomeChild/></DistanceController>
// =============================================================================

#include <chrono>
#include <string>
#include <memory>
#include <cmath>

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/decorator/distance_controller.hpp"

namespace nav2_behavior_tree
{

// 构造函数：读取端口，获取共享节点与 tf 缓冲区。
DistanceController::DistanceController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  distance_(1.0),
  global_frame_("map"),
  robot_base_frame_("base_link"),
  first_time_(false)
{
  // 触发距离（米）、目标坐标系、机器人基座坐标系。
  getInput("distance", distance_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  // 从黑板取共享 ROS2 节点与 tf2 缓冲区。
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  // tf 查询时间容差（超龄的变换视为不可用）。
  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

// tick 主函数：按行驶距离决定是否透传 tick 给子节点。
inline BT::NodeStatus DistanceController::tick()
{
  // 节点从 IDLE 启动（新一轮执行）：记录起始位姿作为距离累计起点。
  if (status() == BT::NodeStatus::IDLE) {
    // Reset the starting position since we're starting a new iteration of
    // the distance controller (moving from IDLE to RUNNING)
    if (!nav2_util::getCurrentPose(
        start_pose_, *tf_, global_frame_, robot_base_frame_,
        transform_tolerance_))
    {
      // tf 查询失败（机器人位姿不可用）：本节点失败。
      RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
      return BT::NodeStatus::FAILURE;
    }
    first_time_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  // 查询机器人当前位姿。
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  // Get euclidean distance
  // 计算自起始点以来的行驶距离（欧氏距离）。
  auto travelled = nav2_util::geometry_utils::euclidean_distance(
    start_pose_.pose, current_pose.pose);

  // The child gets ticked the first time through and every time the threshold
  // distance is crossed. In addition, once the child begins to run, it is
  // ticked each time 'til completion
  // （三种情况透传 tick：首次、子节点 RUNNING、行驶距离达到阈值。）
  if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    travelled >= distance_)
  {
    first_time_ = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      // 子节点仍在运行：本节点 RUNNING。
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      // 子节点成功：把起始位姿重置为当前位姿
      // （重新开始累计下一段触发距离），并透传 SUCCESS。
      case BT::NodeStatus::SUCCESS:
        if (!nav2_util::getCurrentPose(
            start_pose_, *tf_, global_frame_, robot_base_frame_,
            transform_tolerance_))
        {
          RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
          return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;

      // 子节点失败：透传 FAILURE。
      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  // 距离未达标且子节点未在运行：不 tick 子节点，保持当前状态。
  return status();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
// 节点注册：动态库加载时把 "DistanceController" 标签注册进 factory。
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::DistanceController>("DistanceController");
}
