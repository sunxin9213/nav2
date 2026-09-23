// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include <cmath>
#include "nav2_mppi_controller/critics/obstacles_critic.hpp"
#include "nav2_core/exceptions.hpp"

namespace mppi::critics
{

// 中文说明（文件级）：
// 障碍物 critic：惩罚靠近或撞上障碍物的轨迹。
// 代价分为两部分：
// 1. 碰撞代价（critical）：轨迹任一点处于碰撞状态时整条轨迹记 collision_cost 大罚分；
// 2. 斥力代价（repulsion）：利用膨胀层的代价梯度换算到障碍物距离，
//    距离小于 collision_margin 的点重罚，距离在膨胀半径内的点按
//    (inflation_radius - dist) 累积轻罚（类似势场斥力）。
// 若所有轨迹均碰撞则置 fail_flag 触发优化器回退。

// 初始化：读取参数并计算外接圆代价阈值
void ObstaclesCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  // consider_footprint：是否用完整多边形足迹（而非圆形）做碰撞检查
  getParam(consider_footprint_, "consider_footprint", false);
  getParam(power_, "cost_power", 1);  // 代价幂次（1=线性，2=平方）
  getParam(repulsion_weight_, "repulsion_weight", 1.5);  // 斥力项权重
  getParam(critical_weight_, "critical_weight", 20.0);    // 近距临界项权重
  getParam(collision_cost_, "collision_cost", 10000.0);   // 碰撞罚分
  // 距障碍物小于该距离的轨迹点受重罚
  getParam(collision_margin_distance_, "collision_margin_distance", 0.10);
  // 接近目标时禁用斥力项（目标附近常紧邻障碍物）
  getParam(near_goal_distance_, "near_goal_distance", 0.5);

  // 配置足迹碰撞检查器并计算外接圆半径对应的代价值
  collision_checker_.setCostmap(costmap_);
  possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);

  // 外接代价 < 1 说明膨胀层缺失或膨胀半径不足，无法做优化的非圆形碰撞检查
  if (possibly_inscribed_cost_ < 1.0f) {
    RCLCPP_ERROR(
      logger_,
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
      " for full instructions. This will substantially impact run-time performance.");
  }

  // 代价地图用半径而 critic 用足迹（或反之）时配置不一致：警告或报错
  if (costmap_ros_->getUseRadius() == consider_footprint_) {
    RCLCPP_WARN(
      logger_,
      "Inconsistent configuration in collision checking. Please verify the robot's shape settings "
      "in both the costmap and the obstacle critic.");
    if (costmap_ros_->getUseRadius()) {
      throw nav2_core::PlannerException(
              "Considering footprint in collision checking but no robot footprint provided in the "
              "costmap.");
    }
  }

  RCLCPP_INFO(
    logger_,
    "ObstaclesCritic instantiated with %d power and %f / %f weights. "
    "Critic will collision check based on %s cost.",
    power_, critical_weight_, repulsion_weight_, consider_footprint_ ?
    "footprint" : "circular");
}

// 计算外接圆（circumscribed radius）处的膨胀代价值
//（用于快速判定：中心点代价低于该值则足迹必定安全，无需逐点检查足迹）
float ObstaclesCritic::findCircumscribedCost(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  double result = -1.0;
  bool inflation_layer_found = false;

  const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
  if (static_cast<float>(circum_radius) == circumscribed_radius_) {
    // early return if footprint size is unchanged
    //（足迹尺寸未变化时直接返回缓存值，支持动态足迹场景的快速路径）
    return circumscribed_cost_;
  }

  // check if the costmap has an inflation layer
  //（遍历代价地图各层，查找膨胀层并计算外接圆处的代价值）
  for (auto layer = costmap->getLayeredCostmap()->getPlugins()->begin();
    layer != costmap->getLayeredCostmap()->getPlugins()->end();
    ++layer)
  {
    auto inflation_layer = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(*layer);
    if (!inflation_layer) {
      continue;  // 跳过非膨胀层
    }

    inflation_layer_found = true;
    // 将外接圆半径换算为栅格数并查询膨胀代价
    const double resolution = costmap->getCostmap()->getResolution();
    result = inflation_layer->computeCost(circum_radius / resolution);
    // 读取膨胀层的缩放参数（用于反算距离）
    auto getParam = parameters_handler_->getParamGetter(name_);
    getParam(inflation_scale_factor_, "cost_scaling_factor", 10.0);
    getParam(inflation_radius_, "inflation_radius", 0.55);
  }

  if (!inflation_layer_found) {
    RCLCPP_WARN(
      logger_,
      "No inflation layer found in costmap configuration. "
      "If this is an SE2-collision checking plugin, it cannot use costmap potential "
      "field to speed up collision checking by only checking the full footprint "
      "when robot is within possibly-inscribed radius of an obstacle. This may "
      "significantly slow down planning times and not avoid anything but absolute collisions!");
  }

  // 缓存结果供后续周期复用
  circumscribed_radius_ = static_cast<float>(circum_radius);
  circumscribed_cost_ = static_cast<float>(result);

  return circumscribed_cost_;
}

// 由膨胀代价值反推轨迹点到最近障碍物的距离
//（利用膨胀层的指数衰减模型：cost = 253 * exp(-scale*(d - r_inscribed))）
float ObstaclesCritic::distanceToObstacle(const CollisionCost & cost)
{
  const float scale_factor = inflation_scale_factor_;
  const float min_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  // 由代价公式取对数反解距离
  float dist_to_obj = (scale_factor * min_radius - log(cost.cost) + log(253.0f)) / scale_factor;

  // If not footprint collision checking, the cost is using the center point cost and
  // needs the radius subtracted to obtain the closest distance to the object
  //（若代价取自中心点而非足迹，需减去内切半径得到实际最近距离）
  if (!cost.using_footprint) {
    dist_to_obj -= min_radius;
  }

  return dist_to_obj;
}

// 评分主函数：对每条候选轨迹的每个点做碰撞检查并累计斥力代价
void ObstaclesCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;
  if (!enabled_) {
    return;  // 已禁用则跳过
  }

  if (consider_footprint_) {
    // footprint may have changed since initialization if user has dynamic footprints
    //（动态足迹场景：每周期重新计算外接圆代价）
    possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);
  }

  // If near the goal, don't apply the preferential term since the goal is near obstacles
  //（接近目标时禁用斥力项，避免目标附近的障碍物斥力阻止机器人到达）
  bool near_goal = false;
  if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path)) {
    near_goal = true;
  }

  // 分开累计两种代价：近距临界代价与斥力代价
  auto && raw_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  raw_cost.fill(0.0f);
  auto && repulsive_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  repulsive_cost.fill(0.0f);

  const size_t traj_len = data.trajectories.x.shape(1);  // 轨迹时间步数
  bool all_trajectories_collide = true;  // 全部轨迹碰撞标志（触发回退）
  // 外层遍历每条轨迹，内层遍历轨迹上的每个位姿点
  for (size_t i = 0; i < data.trajectories.x.shape(0); ++i) {
    bool trajectory_collide = false;
    float traj_cost = 0.0f;
    const auto & traj = data.trajectories;
    CollisionCost pose_cost;

    for (size_t j = 0; j < traj_len; j++) {
      // 查询该位姿处的代价值（必要时做完整足迹检查）
      pose_cost = costAtPose(traj.x(i, j), traj.y(i, j), traj.yaws(i, j));
      if (pose_cost.cost < 1.0f) {continue;}  // In free space（自由空间，跳过）

      if (inCollision(pose_cost.cost)) {
        // 任一点碰撞即整条轨迹判为碰撞，提前退出内层循环
        trajectory_collide = true;
        break;
      }

      // Cannot process repulsion if inflation layer does not exist
      //（无膨胀层时无法换算距离，跳过斥力计算）
      if (inflation_radius_ == 0.0f || inflation_scale_factor_ == 0.0f) {
        continue;
      }

      // 由代价反算到最近障碍物的距离
      const float dist_to_obj = distanceToObstacle(pose_cost);

      // Let near-collision trajectory points be punished severely
      //（距障碍小于 collision_margin 的点：重罚 (margin - dist)）
      if (dist_to_obj < collision_margin_distance_) {
        traj_cost += (collision_margin_distance_ - dist_to_obj);
      } else if (!near_goal) {  // Generally prefer trajectories further from obstacles
        // 一般情况：按 (膨胀半径 - 距离) 累积轻斥力，偏好远离障碍的轨迹
        repulsive_cost[i] += (inflation_radius_ - dist_to_obj);
      }
    }

    if (!trajectory_collide) {all_trajectories_collide = false;}
    // 碰撞轨迹直接记碰撞罚分，否则记近距临界代价
    raw_cost[i] = trajectory_collide ? collision_cost_ : traj_cost;
  }

  // 合成总代价：critical 权重的碰撞/临界项 + repulsion 权重的斥力项（按轨迹长度归一）
  data.costs += xt::pow(
    (critical_weight_ * raw_cost) +
    (repulsion_weight_ * repulsive_cost / traj_len),
    power_);
  // 全部轨迹碰撞时置失败标志，优化器将触发回退重试
  data.fail_flag = all_trajectories_collide;
}

/**
  * @brief Checks if cost represents a collision
  * @param cost Costmap cost
  * @return bool if in collision
  *（判断代价值是否代表碰撞：致命障碍/内切碰撞/未知区域三种情形）
  */
bool ObstaclesCritic::inCollision(float cost) const
{
  bool is_tracking_unknown =
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

  switch (static_cast<unsigned char>(cost)) {
    using namespace nav2_costmap_2d; // NOLINT
    case (LETHAL_OBSTACLE):
      // 致命障碍：一定碰撞
      return true;
    case (INSCRIBED_INFLATED_OBSTACLE):
      // 内切障碍：圆模型下视为碰撞；足迹模型下需进一步做完整足迹检查
      return consider_footprint_ ? false : true;
    case (NO_INFORMATION):
      // 未知区域：取决于代价地图是否将未知视为障碍
      return is_tracking_unknown ? false : true;
  }

  return false;  // 其余代价值不视为碰撞
}

// 查询位姿 (x, y, theta) 处的碰撞代价：先用中心点，必要时升级为完整足迹检查
CollisionCost ObstaclesCritic::costAtPose(float x, float y, float theta)
{
  CollisionCost collision_cost;
  float & cost = collision_cost.cost;
  collision_cost.using_footprint = false;
  unsigned int x_i, y_i;
  // 世界坐标转地图栅格，越界则返回 NO_INFORMATION
  if (!collision_checker_.worldToMap(x, y, x_i, y_i)) {
    cost = nav2_costmap_2d::NO_INFORMATION;
    return collision_cost;
  }
  // 先取中心点代价（快速路径）
  cost = collision_checker_.pointCost(x_i, y_i);

  // 中心点代价已达到可能内切阈值（或膨胀层缺失）时，才做昂贵的完整足迹检查
  if (consider_footprint_ &&
    (cost >= possibly_inscribed_cost_ || possibly_inscribed_cost_ < 1.0f))
  {
    cost = static_cast<float>(collision_checker_.footprintCostAtPose(
        x, y, theta, costmap_ros_->getRobotFootprint()));
    collision_cost.using_footprint = true;  // 标记：此次代价来自足迹
  }

  return collision_cost;
}

}  // namespace mppi::critics

// 通过 pluginlib 导出为可动态加载的 critic 插件
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::ObstaclesCritic,
  mppi::critics::CriticFunction)
