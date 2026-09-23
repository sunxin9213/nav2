// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Dexory
// Copyright (c) 2023 Open Navigation LLC
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

#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

// 本文件实现 PathHandler（路径处理器）：
// 负责全局路径的接收、裁剪与坐标变换。每个控制周期它把全局规划器
// 输出的路径限制到"机器人附近的一段"：
//   1) 在路径上限制搜索范围（max_robot_pose_search_dist）内找最近点；
//   2) 从最近点向后保留 prune_distance 长度的局部路径；
//   3) 将局部路径变换到代价地图坐标系，并截断于代价地图边界。
// 还支持 enforce_path_inversion 选项：在路径首次掉头（方向反转）处截断，
// 防止控制器贪心跟踪"回头路"。
namespace mppi
{

// 生命周期初始化：保存依赖并读取路径处理相关参数
void PathHandler::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap,
  std::shared_ptr<tf2_ros::Buffer> buffer, ParametersHandler * param_handler)
{
  name_ = name;
  costmap_ = costmap;         // 代价地图（用于坐标变换与边界判断）
  tf_buffer_ = buffer;        // TF 变换缓冲
  auto node = parent.lock();
  logger_ = node->get_logger();
  parameters_handler_ = param_handler;

  auto getParam = parameters_handler_->getParamGetter(name_);
  // 默认取代价地图较长边的一半：最近点搜索的最大路径长度
  getParam(max_robot_pose_search_dist_, "max_robot_pose_search_dist", getMaxCostmapDist());
  getParam(prune_distance_, "prune_distance", 1.5);                       // 每周期保留的局部路径长度
  getParam(transform_tolerance_, "transform_tolerance", 0.1);               // TF 变换等待超时
  getParam(enforce_path_inversion_, "enforce_path_inversion", false);      // 是否在路径掉头处截断
  if (enforce_path_inversion_) {
    getParam(inversion_xy_tolerance_, "inversion_xy_tolerance", 0.2);      // 到达掉头点的位置容差
    getParam(inversion_yaw_tolerance, "inversion_yaw_tolerance", 0.4);     // 到达掉头点的角度容差
    inversion_locale_ = 0u;  // 0 表示尚未通过任何掉头点
  }
}

// 核心函数：从全局路径中裁剪出代价地图范围内的局部路径
std::pair<nav_msgs::msg::Path, PathIterator>
PathHandler::getGlobalPlanConsideringBoundsInCostmapFrame(
  const geometry_msgs::msg::PoseStamped & global_pose)
{
  using nav2_util::geometry_utils::euclidean_distance;

  auto begin = global_plan_up_to_inversion_.poses.begin();

  // Limit the search for the closest pose up to max_robot_pose_search_dist on the path
  // 搜索上限：沿路径累计弧长达到 max_robot_pose_search_dist 处，
  // 防止机器人在长路径上"抄近道"跳到远处路径点
  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_up_to_inversion_.poses.begin(), global_plan_up_to_inversion_.poses.end(),
    max_robot_pose_search_dist_);

  // Find closest point to the robot
  // 在受限范围内找距离机器人最近的路径点（min_by 按欧氏距离取最小）
  auto closest_point = nav2_util::geometry_utils::min_by(
    begin, closest_pose_upper_bound,
    [&global_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(global_pose, ps);
    });

  // 输出路径：位于代价地图坐标系中
  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_->getGlobalFrameID();
  transformed_plan.header.stamp = global_pose.header.stamp;

  // 从最近点开始只保留 prune_distance 弧长的路径（局部化）
  auto pruned_plan_end =
    nav2_util::geometry_utils::first_after_integrated_distance(
    closest_point, global_plan_up_to_inversion_.poses.end(), prune_distance_);

  unsigned int mx, my;
  // Find the furthest relevent pose on the path to consider within costmap
  // bounds
  // Transforming it to the costmap frame in the same loop
  // 逐点变换到代价地图坐标系，遇到越界点即停止
  for (auto global_plan_pose = closest_point; global_plan_pose != pruned_plan_end;
    ++global_plan_pose)
  {
    // Transform from global plan frame to costmap frame
    // 用当前时刻盖时间戳，保证 TF 能查到最近变换
    geometry_msgs::msg::PoseStamped costmap_plan_pose;
    global_plan_pose->header.stamp = global_pose.header.stamp;
    global_plan_pose->header.frame_id = global_plan_.header.frame_id;
    transformPose(costmap_->getGlobalFrameID(), *global_plan_pose, costmap_plan_pose);

    // Check if pose is inside the costmap
    // worldToMap 返回 false 说明该点已超出代价地图边界 -> 截断
    if (!costmap_->getCostmap()->worldToMap(
        costmap_plan_pose.pose.position.x, costmap_plan_pose.pose.position.y, mx, my))
    {
      return {transformed_plan, closest_point};
    }

    // Filling the transformed plan to return with the transformed pose
    transformed_plan.poses.push_back(costmap_plan_pose);
  }

  // 返回：变换后的局部路径 + 最近点迭代器（供调用方裁剪已走过的路径）
  return {transformed_plan, closest_point};
}

// 把机器人位姿变换到全局路径坐标系，便于在路径坐标系内做最近点搜索
geometry_msgs::msg::PoseStamped PathHandler::transformToGlobalPlanFrame(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_up_to_inversion_.poses.empty()) {
    throw std::runtime_error("Received plan with zero length");
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_up_to_inversion_.header.frame_id, pose, robot_pose)) {
    throw std::runtime_error(
            "Unable to transform robot pose into global plan's frame");
  }

  return robot_pose;
}

// 主入口：输入机器人位姿，输出代价地图坐标系下的局部参考路径
nav_msgs::msg::Path PathHandler::transformPath(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Find relevent bounds of path to use
  // 先变换到路径坐标系，再裁剪出局部路径
  geometry_msgs::msg::PoseStamped global_pose =
    transformToGlobalPlanFrame(robot_pose);
  auto [transformed_plan, lower_bound] = getGlobalPlanConsideringBoundsInCostmapFrame(global_pose);

  // 从全局（截断后）路径中删除最近点之前已走过的部分（永久性裁剪）
  prunePlan(global_plan_up_to_inversion_, lower_bound);

  // 若已越过某次掉头且位姿满足容差，则推进掉头计数：
  // 把原始全局路径推进到该掉头点之后，再重新截断到下一个掉头处
  if (enforce_path_inversion_ && inversion_locale_ != 0u) {
    if (isWithinInversionTolerances(global_pose)) {
      prunePlan(global_plan_, global_plan_.poses.begin() + inversion_locale_);
      global_plan_up_to_inversion_ = global_plan_;
      inversion_locale_ = utils::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
    }
  }

  // 裁剪后没有任何位姿（如机器人已远离路径）则报错，由上层进入恢复
  if (transformed_plan.poses.empty()) {
    throw std::runtime_error("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

// TF 坐标变换封装：带超时与异常处理
bool PathHandler::transformPose(
  const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  // 同坐标系则无需变换，直接拷贝
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    // 在 transform_tolerance_ 秒内等待变换可用
    tf_buffer_->transform(
      in_pose, out_pose, frame,
      tf2::durationFromSec(transform_tolerance_));
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;  // 变换失败
}

// 最大最近点搜索距离的默认值：代价地图较长边的一半（像素数 × 分辨率 × 0.5）
double PathHandler::getMaxCostmapDist()
{
  const auto & costmap = costmap_->getCostmap();
  return static_cast<double>(std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY())) *
         costmap->getResolution() * 0.50;
}

// 接收新的全局路径（由 controller setPlan 调用）
void PathHandler::setPath(const nav_msgs::msg::Path & plan)
{
  global_plan_ = plan;
  global_plan_up_to_inversion_ = global_plan_;
  // 若启用掉头截断，删除首个掉头点之后的所有位姿
  if (enforce_path_inversion_) {
    inversion_locale_ = utils::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
  }
}

nav_msgs::msg::Path & PathHandler::getPath() {return global_plan_;}

// 删除 [begin, end) 内的路径位姿：即丢弃已经走过的前缀
void PathHandler::prunePlan(nav_msgs::msg::Path & plan, const PathIterator end)
{
  plan.poses.erase(plan.poses.begin(), end);
}

// 判断机器人是否已"到达"当前掉头点（位置与角度均在容差内）。
// 只有满足此条件才允许推进到掉头点之后的路径段，
// 防止机器人未完成掉头就跳过该段
bool PathHandler::isWithinInversionTolerances(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Keep full path if we are within tolerance of the inversion pose
  const auto last_pose = global_plan_up_to_inversion_.poses.back();
  float distance = hypotf(
    robot_pose.pose.position.x - last_pose.pose.position.x,
    robot_pose.pose.position.y - last_pose.pose.position.y);

  // 角度差取最短角（-pi..pi 内）
  float angle_distance = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation),
    tf2::getYaw(last_pose.pose.orientation));

  return distance <= inversion_xy_tolerance_ && fabs(angle_distance) <= inversion_yaw_tolerance;
}

}  // namespace mppi
