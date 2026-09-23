// Copyright (c) 2022 Samsung Research America
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

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_graceful_controller/path_handler.hpp"
#include "nav2_core/exceptions.hpp"

namespace nav2_graceful_controller
{

using nav2_util::geometry_utils::euclidean_distance;

// 中文说明（文件级）：
// 路径处理器：负责全局路径的存储、剪枝与向机器人坐标系的变换。
// 与 RPP 的 transformGlobalPlan 类似：找到机器人附近的路径起点、
// 截断到局部代价地图范围内的近段、变换到 base 坐标系，
// 并将已走过的部分从全局路径中删除（剪枝）。

// 构造：保存 TF 容差、TF 缓冲与代价地图
PathHandler::PathHandler(
  tf2::Duration transform_tolerance,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: transform_tolerance_(transform_tolerance), tf_buffer_(tf), costmap_ros_(costmap_ros)
{
}

// 变换全局计划：截取局部近段并转换到机器人坐标系（含路径剪枝）
nav_msgs::msg::Path PathHandler::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose,
  double max_robot_pose_search_dist)
{
  // Check first if the plan is empty
  //（空路径直接报错）
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // Let's get the pose of the robot in the frame of the plan
  //（把机器人位姿变换到全局路径的坐标系）
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav_2d_utils::transformPose(
      tf_buffer_, global_plan_.header.frame_id, pose, robot_pose,
      transform_tolerance_))
  {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // Find the first pose in the global plan that's further than max_robot_pose_search_dist
  // from the robot using integrated distance
  //（用积分距离确定搜索上界：距机器人最大搜索距离处的路径点）
  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  //（在搜索上界内找离机器人最近的路径点作为变换起点；
  //  限定范围可避免匹配到回折路径的远端）
  auto transformation_begin =
    nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // We'll discard points on the plan that are outside the local costmap
  //（超出局部代价地图半边长的路径点作为变换终点）
  double dist_threshold = std::max(
    costmap_ros_->getCostmap()->getSizeInMetersX(),
    costmap_ros_->getCostmap()->getSizeInMetersY()) / 2.0;
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(global_plan_pose, robot_pose) > dist_threshold;
    });

  // Lambda to transform a PoseStamped from global frame to local
  //（单个路径点从全局系变换到 base 坐标系的 lambda）
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      if (!nav_2d_utils::transformPose(
          tf_buffer_, costmap_ros_->getBaseFrameID(), stamped_pose,
          transformed_pose, transform_tolerance_))
      {
        throw nav2_core::PlannerException("Unable to transform plan pose into local frame");
      }
      transformed_pose.pose.position.z = 0.0;  // 平面化：z 置零
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  //（批量变换 [起点, 终点) 区间的路径点到机器人系）
  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  //（路径剪枝：删除已走过的部分，下周期不再重复处理）
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

// 保存新的全局计划
void PathHandler::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

}  // namespace nav2_graceful_controller
