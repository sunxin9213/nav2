// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#include "nav2_util/geometry_utils.hpp"
#include "nav2_graceful_controller/graceful_controller.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_core/exceptions.hpp"

namespace nav2_graceful_controller
{

// 中文说明（文件级）：
// 优雅（Graceful）控制器实现：基于平滑控制律的路径跟踪控制器。
// 每周期流程（computeVelocityCommands）：
// 变换全局路径到机器人系 → 取运动目标点（前探点）→
// 按需设置目标朝向/倒车 → 三分支决策（终点原地旋转 /
// 起始原地旋转 / 平滑控制律跟踪）→ 前向仿真轨迹做碰撞检查。

// 配置（生命周期 on_configure）：创建参数/路径/控制律组件与调试发布器
void GracefulController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  // Handles storage and dynamic configuration of parameters.
  // Returns pointer to data current param settings.
  //（参数处理器：存储并动态更新参数，返回当前参数指针）
  param_handler_ = std::make_unique<ParameterHandler>(
    node, plugin_name_, logger_,
    costmap_ros_->getCostmap()->getSizeInMetersX());
  params_ = param_handler_->getParams();

  // Handles global path transformations
  //（路径处理器：负责全局路径的变换与截断）
  path_handler_ = std::make_unique<PathHandler>(
    tf2::durationFromSec(params_->transform_tolerance), tf_buffer_, costmap_ros_);

  // Handles the control law to generate the velocity commands
  //（平滑控制律：生成速度指令的核心组件）
  control_law_ = std::make_unique<SmoothControlLaw>(
    params_->k_phi, params_->k_delta, params_->beta, params_->lambda, params_->slowdown_radius,
    params_->v_linear_min, params_->v_linear_max, params_->v_angular_max);

  // Initialize footprint collision checker
  //（足迹碰撞检查器）
  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_ros_->getCostmap());

  // Publishers（调试发布器：变换后路径/局部轨迹/运动目标点/减速标记）
  transformed_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 1);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  motion_target_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("motion_target", 1);
  slowdown_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("slowdown", 1);

  RCLCPP_INFO(logger_, "Configured Graceful Motion Controller: %s", plugin_name_.c_str());
}

// 清理（生命周期 on_cleanup）：释放全部组件与发布器
void GracefulController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type graceful_controller::GracefulController",
    plugin_name_.c_str());
  transformed_plan_pub_.reset();
  local_plan_pub_.reset();
  motion_target_pub_.reset();
  slowdown_pub_.reset();
  collision_checker_.reset();
  path_handler_.reset();
  param_handler_.reset();
  control_law_.reset();
}

// 激活（生命周期 on_activate）：激活各生命周期发布器
void GracefulController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type nav2_graceful_controller::GracefulController",
    plugin_name_.c_str());
  transformed_plan_pub_->on_activate();
  local_plan_pub_->on_activate();
  motion_target_pub_->on_activate();
  slowdown_pub_->on_activate();
}

// 停用（生命周期 on_deactivate）：暂停发布
void GracefulController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type nav2_graceful_controller::GracefulController",
    plugin_name_.c_str());
  transformed_plan_pub_->on_deactivate();
  local_plan_pub_->on_deactivate();
  motion_target_pub_->on_deactivate();
  slowdown_pub_->on_deactivate();
}

// 计算速度指令：控制器主入口（每控制周期由 controller_server 调用）
geometry_msgs::msg::TwistStamped GracefulController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * goal_checker)
{
  // 锁定参数（防止动态调参与计算竞态）
  std::lock_guard<std::mutex> param_lock(param_handler_->getMutex());

  // Update for the current goal checker's state
  //（从 goal_checker 获取目标位置容差，用于判断是否进入终点旋转阶段）
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tolerance_ = pose_tolerance.position.x;
  }

  // Update the smooth control law with the new params
  //（用最新参数刷新控制律的增益与速度限制）
  control_law_->setCurvatureConstants(
    params_->k_phi, params_->k_delta, params_->beta, params_->lambda);
  control_law_->setSlowdownRadius(params_->slowdown_radius);
  control_law_->setSpeedLimit(params_->v_linear_min, params_->v_linear_max, params_->v_angular_max);

  // Transform path to robot base frame and publish it
  //（全局路径变换到机器人坐标系并截断，发布用于调试）
  auto transformed_plan = path_handler_->transformGlobalPlan(
    pose, params_->max_robot_pose_search_dist);
  transformed_plan_pub_->publish(transformed_plan);

  // Get the particular point on the path at the motion target distance and publish it
  //（在路径上取距机器人 motion_target_dist 的运动目标点并发布）
  auto motion_target = getMotionTarget(params_->motion_target_dist, transformed_plan);
  auto motion_target_point = nav2_graceful_controller::createMotionTargetMsg(motion_target);
  motion_target_pub_->publish(motion_target_point);

  // Publish marker for slowdown radius around motion target for debugging / visualization
  //（发布减速半径标记，可视化接近目标的减速范围）
  auto slowdown_marker = nav2_graceful_controller::createSlowdownMarker(
    motion_target,
    params_->slowdown_radius);
  slowdown_pub_->publish(slowdown_marker);

  // Compute distance to goal as the path's integrated distance to account for path curvatures
  //（用路径积分长度（而非直线距离）计算到终点距离，更符合弯曲路径实际）
  double dist_to_goal = nav2_util::geometry_utils::calculate_path_length(transformed_plan);

  // If the distance to the goal is less than the motion target distance, i.e.
  // the 'motion target' is the goal, then we skip the motion target orientation by pointing
  // it in the same orientation that the last segment of the path
  //（运动目标点在机器人系下的方位角）
  double angle_to_target = atan2(motion_target.pose.position.y, motion_target.pose.position.x);
  if (params_->final_rotation && dist_to_goal < params_->motion_target_dist) {
    // 接近终点：把运动目标朝向设为路径最后一段的方向，
    // 使平滑控制律自然收敛到路径切向（而非直接跳到目标朝向）
    geometry_msgs::msg::PoseStamped stl_pose =
      transformed_plan.poses[transformed_plan.poses.size() - 2];
    geometry_msgs::msg::PoseStamped goal_pose = transformed_plan.poses.back();
    double dx = goal_pose.pose.position.x - stl_pose.pose.position.x;
    double dy = goal_pose.pose.position.y - stl_pose.pose.position.y;
    double yaw = std::atan2(dy, dx);
    motion_target.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  }

  // Flip the orientation of the motion target if the robot is moving backwards
  //（允许倒车且目标点在身后：翻转目标朝向进入倒车模式）
  bool reversing = false;
  if (params_->allow_backward && motion_target.pose.position.x < 0.0) {
    reversing = true;
    motion_target.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
      tf2::getYaw(motion_target.pose.orientation) + M_PI);
  }

  // Compute velocity command:
  // 1. Check if we are close enough to the goal to do a final rotation in place
  // 2. Check if we must do a rotation in place before moving
  // 3. Calculate the new velocity command using the smooth control law
  //（三分支决策：1.终点原地旋转对准目标朝向；2.起始大偏差先原地旋转；3.平滑控制律跟踪）
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  if (params_->final_rotation && (dist_to_goal < goal_dist_tolerance_ || goal_reached_)) {
    // 分支 1：已到达目标位置（goal_reached_ 状态保持，避免来回切换）
    goal_reached_ = true;
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    cmd_vel.twist = rotateToTarget(angle_to_goal);
  } else if (params_->initial_rotation && // NOLINT
    fabs(angle_to_target) > params_->initial_rotation_min_angle)
  {
    // 分支 2：目标方位角偏差过大，先原地旋转对准
    cmd_vel.twist = rotateToTarget(angle_to_target);
  } else {
    // 分支 3：常规跟踪，用平滑控制律直接计算速度指令
    cmd_vel.twist = control_law_->calculateRegularVelocity(motion_target.pose, reversing);
  }

  // Transform local frame to global frame to use in collision checking
  //（获取 base→global 变换，供仿真轨迹的碰撞检查使用）
  geometry_msgs::msg::TransformStamped costmap_transform;
  try {
    costmap_transform = tf_buffer_->lookupTransform(
      costmap_ros_->getGlobalFrameID(), costmap_ros_->getBaseFrameID(),
      tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    // TF 失败：直接返回当前指令（跳过碰撞检查）
    RCLCPP_ERROR(
      logger_, "Could not transform %s to %s: %s",
      costmap_ros_->getBaseFrameID().c_str(), costmap_ros_->getGlobalFrameID().c_str(),
      ex.what());
    return cmd_vel;
  }

  // Generate and publish local plan for debugging / visualization
  //（前向仿真轨迹并做碰撞检查：碰撞则抛异常触发上层恢复行为）
  nav_msgs::msg::Path local_plan;
  if (!simulateTrajectory(pose, motion_target, costmap_transform, local_plan, reversing)) {
    throw nav2_core::PlannerException("Collision detected in the trajectory");
  }
  local_plan.header = transformed_plan.header;
  local_plan_pub_->publish(local_plan);

  return cmd_vel;
}

// 设置全局路径（每条新路径到来时由 controller_server 调用）
void GracefulController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_->setPlan(path);
  goal_reached_ = false;  // 新任务：重置目标到达状态
}

// 设置速度上限（速度限制器/代价地图过滤器回调触发）
void GracefulController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> param_lock(param_handler_->getMutex());

  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // 无限制：恢复初始速度参数
    params_->v_linear_max = params_->v_linear_max_initial;
    params_->v_angular_max = params_->v_angular_max_initial;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      //（百分比限速：线性/角速度按比例缩放，但不低于最小线速度）
      params_->v_linear_max = std::max(
        params_->v_linear_max_initial * speed_limit / 100.0, params_->v_linear_min);
      params_->v_angular_max = params_->v_angular_max_initial * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in m/s
      //（绝对值限速：角速度按线速度比例缩放）
      params_->v_linear_max = std::max(speed_limit, params_->v_linear_min);
      // Limit the angular velocity to be proportional to the linear velocity
      params_->v_angular_max = params_->v_angular_max_initial *
        speed_limit / params_->v_linear_max_initial;
    }
  }
}

// 获取运动目标点：路径上第一个距机器人 ≥ motion_target_dist 的位姿
geometry_msgs::msg::PoseStamped GracefulController::getMotionTarget(
  const double & motion_target_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the motion target distance
  //（在机器人系下找第一个距离达到前探距离的路径点）
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return std::hypot(ps.pose.position.x, ps.pose.position.y) >= motion_target_dist;
    });

  // If the pose is not far enough, take the last pose
  //（剩余路径都不够远：取最后一个点（即终点）作为目标）
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  }

  return *goal_pose_it;
}

// 前向仿真轨迹：逐步应用控制律并检查碰撞（返回是否无碰撞）
bool GracefulController::simulateTrajectory(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::PoseStamped & motion_target,
  const geometry_msgs::msg::TransformStamped & costmap_transform,
  nav_msgs::msg::Path & trajectory, const bool & backward)
{
  // Check for collision before moving
  //（起点即碰撞：直接失败）
  if (inCollision(
      robot_pose.pose.position.x, robot_pose.pose.position.y,
      tf2::getYaw(robot_pose.pose.orientation)))
  {
    return false;
  }

  // First pose
  //（轨迹首点：机器人系原点）
  geometry_msgs::msg::PoseStamped next_pose;
  next_pose.header.frame_id = costmap_ros_->getBaseFrameID();
  next_pose.pose.orientation.w = 1.0;
  trajectory.poses.push_back(next_pose);

  double distance = std::numeric_limits<double>::max();
  double resolution_ = costmap_ros_->getCostmap()->getResolution();
  // 仿真步长：走一个栅格距离所需的时间
  double dt = (params_->v_linear_max > 0.0) ? resolution_ / params_->v_linear_max : 0.0;

  // Set max iter to avoid infinite loop
  //（迭代上限约为目标距离 2 倍的栅格数，防止死循环）
  unsigned int max_iter = 2 * sqrt(
    motion_target.pose.position.x * motion_target.pose.position.x +
    motion_target.pose.position.y * motion_target.pose.position.y) / resolution_;

  // Generate path
  //（循环：按控制律推演下一步位姿并检查碰撞，直到到达目标点或超限）
  do{
    // Apply velocities to calculate next pose
    //（用平滑控制律计算下一仿真位姿）
    next_pose.pose = control_law_->calculateNextPose(
      dt, motion_target.pose, next_pose.pose, backward);

    // Add the pose to the trajectory for visualization
    trajectory.poses.push_back(next_pose);

    // Check for collision
    //（变换到全局系后做碰撞检查）
    geometry_msgs::msg::PoseStamped global_pose;
    tf2::doTransform(next_pose, global_pose, costmap_transform);
    if (inCollision(
        global_pose.pose.position.x, global_pose.pose.position.y,
        tf2::getYaw(global_pose.pose.orientation)))
    {
      return false;  // 仿真碰撞：轨迹不安全
    }

    // Check if we reach the goal
    //（与目标点的距离小于一个栅格即认为到达）
    distance = nav2_util::geometry_utils::euclidean_distance(motion_target.pose, next_pose.pose);
  }while(distance > resolution_ && trajectory.poses.size() < max_iter);

  return true;
}

// 原地旋转到目标朝向：按角度差比例输出角速度（线速度为 0）
geometry_msgs::msg::Twist GracefulController::rotateToTarget(const double & angle_to_target)
{
  geometry_msgs::msg::Twist vel;
  vel.linear.x = 0.0;
  // 角速度 = 缩放系数 * 角度差 * 最大角速度（角度差大则快转）
  vel.angular.z = params_->rotation_scaling_factor * angle_to_target * params_->v_angular_max;
  return vel;
}

// 碰撞检查：按足迹形状（多边形或圆）查询给定全局系位姿的碰撞状态
bool GracefulController::inCollision(const double & x, const double & y, const double & theta)
{
  unsigned int mx, my;
  if (!costmap_ros_->getCostmap()->worldToMap(x, y, mx, my)) {
    // 超出代价地图范围：无法检查，警告后按无碰撞处理
    RCLCPP_WARN(
      logger_, "The path is not in the costmap. Cannot check for collisions. "
      "Proceed at your own risk, slow the robot, or increase your costmap size.");
    return false;
  }

  // Calculate the cost of the footprint at the robot's current position depending
  // on the shape of the footprint
  //（根据代价地图配置决定用足迹多边形还是中心点检查）
  bool is_tracking_unknown =
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
  bool consider_footprint = !costmap_ros_->getUseRadius();

  double footprint_cost;
  if (consider_footprint) {
    footprint_cost = collision_checker_->footprintCostAtPose(
      x, y, theta, costmap_ros_->getRobotFootprint());
  } else {
    footprint_cost = collision_checker_->pointCost(mx, my);
  }

  // 分类判断代价值：致命障碍必碰撞；内切障碍仅圆形模型视为碰撞；
  // 未知区域按配置决定
  switch (static_cast<unsigned char>(footprint_cost)) {
    case (nav2_costmap_2d::LETHAL_OBSTACLE):
      return true;
    case (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE):
      return consider_footprint ? false : true;
    case (nav2_costmap_2d::NO_INFORMATION):
      return is_tracking_unknown ? false : true;
  }

  return false;
}

}  // namespace nav2_graceful_controller

// Register this controller as a nav2_core plugin
//（pluginlib 导出为 Nav2 控制器插件）
PLUGINLIB_EXPORT_CLASS(
  nav2_graceful_controller::GracefulController,
  nav2_core::Controller)
