// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include <cmath>
#include "nav2_mppi_controller/critics/cost_critic.hpp"
#include "nav2_core/exceptions.hpp"

namespace mppi::critics
{

// 中文说明（文件级）：
// 代价（Cost）critic：直接以轨迹点在代价地图上的代价值作为惩罚。
// 与 ObstaclesCritic 的区别：不再反算距离，而是把膨胀层的势场值
// 本身作为斥力代价；代价 ≥ 内切障碍阈值的点记 critical_cost 重罚，
// 碰撞轨迹记 collision_cost。权重按最大代价值 254 归一化，
// 使其与其他 critic 权重处于同一量级。
// 该 critic 计算更快，是 MPPI 默认的避障 critic。

// 初始化：读取参数并归一化权重
void CostCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  // consider_footprint：碰撞判定是否使用完整足迹
  getParam(consider_footprint_, "consider_footprint", false);
  getParam(power_, "cost_power", 1);        // 代价幂次
  getParam(weight_, "cost_weight", 3.81);   // 斥力项权重（后除以 254 归一化）
  getParam(critical_cost_, "critical_cost", 300.0);      // 内切级代价点重罚值
  getParam(collision_cost_, "collision_cost", 1000000.0);  // 碰撞罚分
  // 接近目标时禁用斥力项（目标常紧邻障碍物）
  getParam(near_goal_distance_, "near_goal_distance", 0.5);
  // 可指定膨胀层名称（代价地图中有多层膨胀时区分用）
  getParam(inflation_layer_name_, "inflation_layer_name", std::string(""));

  // Normalized by cost value to put in same regime as other weights
  //（用最大代价值 254 归一化权重，使其与其他 critic 权重同量级）
  weight_ /= 254.0f;

  // Normalize weight when parameter is changed dynamically as well
  //（动态调参时同样做归一化）
  auto weightDynamicCb = [&](const rclcpp::Parameter & weight) {
      weight_ = weight.as_double() / 254.0f;
    };
  parameters_handler_->addDynamicParamCallback(name_ + ".cost_weight", weightDynamicCb);

  // 配置碰撞检查器并计算外接圆代价阈值
  collision_checker_.setCostmap(costmap_);
  possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);

  // 膨胀层缺失或不足：无法优化碰撞检查，给出严重警告
  if (possibly_inscribed_cost_ < 1.0f) {
    RCLCPP_ERROR(
      logger_,
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
      " for full instructions. This will substantially impact run-time performance.");
  }

  // 代价地图形状设置与 critic 不一致时的警告/报错
  if (costmap_ros_->getUseRadius() == consider_footprint_) {
    RCLCPP_WARN(
      logger_,
      "Inconsistent configuration in collision checking. Please verify the robot's shape settings "
      "in both the costmap and the cost critic.");
    if (costmap_ros_->getUseRadius()) {
      throw nav2_core::PlannerException(
              "Considering footprint in collision checking but no robot footprint provided in the "
              "costmap.");
    }
  }

  RCLCPP_INFO(
    logger_,
    "InflationCostCritic instantiated with %d power and %f / %f weights. "
    "Critic will collision check based on %s cost.",
    power_, critical_cost_, weight_, consider_footprint_ ?
    "footprint" : "circular");
}

// 计算外接圆处的膨胀代价值（快速判定足迹是否可能碰撞的阈值）
float CostCritic::findCircumscribedCost(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  bool inflation_layer_found = false;
  double result = -1.0;
  const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
  if (static_cast<float>(circum_radius) == circumscribed_radius_) {
    // early return if footprint size is unchanged
    //（足迹未变化则复用缓存，支持动态足迹）
    return circumscribed_cost_;
  }

  // check if the costmap has an inflation layer
  //（遍历代价地图层，找到（与指定名称匹配的）膨胀层）
  for (auto layer = costmap->getLayeredCostmap()->getPlugins()->begin();
    layer != costmap->getLayeredCostmap()->getPlugins()->end();
    ++layer)
  {
    auto inflation_layer = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(*layer);
    if (!inflation_layer ||
      (!inflation_layer_name_.empty() &&
      inflation_layer->getName() != inflation_layer_name_))
    {
      continue;  // 非目标膨胀层，跳过
    }

    inflation_layer_found = true;
    // 外接圆半径换算栅格数并查询膨胀代价值
    const double resolution = costmap->getCostmap()->getResolution();
    result = inflation_layer->computeCost(circum_radius / resolution);
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

  // 缓存结果
  circumscribed_radius_ = static_cast<float>(circum_radius);
  circumscribed_cost_ = static_cast<float>(result);

  return circumscribed_cost_;
}

// 评分主函数：按轨迹点的中心点代价值累积惩罚
void CostCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;
  if (!enabled_) {
    return;  // 已禁用则跳过
  }

  if (consider_footprint_) {
    // footprint may have changed since initialization if user has dynamic footprints
    //（动态足迹：重新计算外接圆代价）
    possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);
  }

  // If near the goal, don't apply the preferential term since the goal is near obstacles
  //（接近目标时不施加偏好项）
  bool near_goal = false;
  if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path)) {
    near_goal = true;
  }

  auto && repulsive_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  repulsive_cost.fill(0.0);

  const size_t traj_len = data.trajectories.x.shape(1);  // 时间步数
  bool all_trajectories_collide = true;  // 全碰撞标志（触发回退）
  // 外层轨迹、内层时间步逐点评分
  for (size_t i = 0; i < data.trajectories.x.shape(0); ++i) {
    bool trajectory_collide = false;
    const auto & traj = data.trajectories;
    float pose_cost;

    for (size_t j = 0; j < traj_len; j++) {
      // The costAtPose doesn't use orientation
      // The footprintCostAtPose will always return "INSCRIBED" if footprint is over it
      // So the center point has more information than the footprint
      //（中心点代价不含朝向信息，但保留完整代价值息量，比足迹代价更适合评分）
      pose_cost = costAtPose(traj.x(i, j), traj.y(i, j));
      if (pose_cost < 1.0f) {continue;}  // In free space（自由空间跳过）

      if (inCollision(pose_cost, traj.x(i, j), traj.y(i, j), traj.yaws(i, j))) {
        // 碰撞：整条轨迹判碰撞并跳出
        trajectory_collide = true;
        break;
      }

      // Let near-collision trajectory points be punished severely
      // Note that we collision check based on the footprint actual,
      // but score based on the center-point cost regardless
      //（碰撞判定可用足迹，但评分始终基于中心点代价）
      using namespace nav2_costmap_2d; // NOLINT
      if (pose_cost >= INSCRIBED_INFLATED_OBSTACLE) {
        // 代价达到内切障碍级：重罚（接近真实碰撞）
        repulsive_cost[i] += critical_cost_;
      } else if (!near_goal) {  // Generally prefer trajectories further from obstacles
        // 一般情况：直接累加膨胀代价值（代价越大离障碍越近）
        repulsive_cost[i] += pose_cost;
      }
    }

    if (!trajectory_collide) {
      all_trajectories_collide = false;
    } else {
      // 碰撞轨迹：斥力代价替换为巨大碰撞罚分
      repulsive_cost[i] = collision_cost_;
    }
  }

  // 合成：权重 * 斥力代价（按轨迹长度平均）后取幂次累加到总代价
  data.costs += xt::pow((weight_ * repulsive_cost / traj_len), power_);
  // 全部轨迹碰撞时置失败标志
  data.fail_flag = all_trajectories_collide;
}

/**
  * @brief Checks if cost represents a collision
  * @param cost Costmap cost
  * @return bool if in collision
  *（碰撞判定：中心点代价达阈值时按需做完整足迹检查后分类判断）
  */
bool CostCritic::inCollision(float cost, float x, float y, float theta)
{
  bool is_tracking_unknown =
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

  // If consider_footprint_ check footprint scort for collision
  //（启用足迹检查且中心点代价达阈值时，取足迹代价再判断）
  if (consider_footprint_ &&
    (cost >= possibly_inscribed_cost_ || possibly_inscribed_cost_ < 1.0f))
  {
    cost = static_cast<float>(collision_checker_.footprintCostAtPose(
        x, y, theta, costmap_ros_->getRobotFootprint()));
  }

  switch (static_cast<unsigned char>(cost)) {
    using namespace nav2_costmap_2d; // NOLINT
    case (LETHAL_OBSTACLE):
      return true;  // 致命障碍：碰撞
    case (INSCRIBED_INFLATED_OBSTACLE):
      // 内切障碍：足迹模式下不视为碰撞（已做过足迹检查），圆模式下视为碰撞
      return consider_footprint_ ? false : true;
    case (NO_INFORMATION):
      // 未知区域：按配置决定
      return is_tracking_unknown ? false : true;
  }

  return false;
}

// 查询 (x, y) 处中心点的代价地图值
float CostCritic::costAtPose(float x, float y)
{
  using namespace nav2_costmap_2d;   // NOLINT
  unsigned int x_i, y_i;
  // 世界坐标转栅格坐标，越界返回未知
  if (!collision_checker_.worldToMap(x, y, x_i, y_i)) {
    return nav2_costmap_2d::NO_INFORMATION;
  }

  return collision_checker_.pointCost(x_i, y_i);
}

}  // namespace mppi::critics

// pluginlib 导出
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::CostCritic,
  mppi::critics::CriticFunction)
