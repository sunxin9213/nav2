// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

/**
 * @class nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
 * @brief Regulated pure pursuit controller plugin
 * （受调节纯跟踪控制器插件）
 *
 * 中文说明：
 * 本文件是 Nav2 中“受调节纯跟踪（Regulated Pure Pursuit, RPP）”控制器的头文件，
 * 对应实现位于 src/regulated_pure_pursuit_controller.cpp。
 *
 * 算法概述：
 * 1. 纯跟踪（Pure Pursuit）是一种几何跟踪算法：在全局路径上选取一个
 *    “前瞻点（lookahead point / carrot 胡萝卜点）”，其与机器人的距离等于前瞻距离 L；
 *    把该点转换到机器人坐标系后坐标为 (x=0, y=L_y)，则路径曲率
 *    κ = 2 * y / L²（圆弧拟合公式），转向角速度 ω = κ * v。
 * 2. “受调节（Regulated）”指在前向速度上叠加多种约束：
 *    - 曲率调节：曲率越大（转弯越急）速度越低（按最小转弯半径线性缩放）；
 *    - 代价调节：机器人附近代价值越高速度越低（利用膨胀层的代价梯度）；
 *    - 接近目标减速（approach velocity scaling）；
 *    - 碰撞检测（沿圆弧投影位姿，用足迹碰撞检查器验证）。
 * 3. 支持倒车（reversing）与朝向突变（cusp 尖点）处理。
 *
 * 在 Nav2 中的位置：本控制器实现 nav2_core::Controller 纯虚接口，由
 * nav2_controller（控制器服务器）通过 pluginlib 动态加载，负责将全局路径
 * 转换为局部速度指令 cmd_vel，实现路径跟踪。
 */
class RegulatedPurePursuitController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
   */
  RegulatedPurePursuitController() = default;

  /**
   * @brief Destrructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
   */
  ~RegulatedPurePursuitController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Transforms global plan into same frame as pose and clips poses ineligible for lookaheadPoint
   * Points ineligible to be selected as a lookahead point if they are any of the following:
   * - Outside the local_costmap (collision avoidance cannot be assured)
   * @param pose pose to transform
   * @return Path in new frame
   */
  nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Transform a pose to another frame.
   * @param frame Frame ID to transform to
   * @param in_pose Pose input to transform
   * @param out_pose transformed output
   * @return bool if successful
   */
  bool transformPose(
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  /**
   * @brief Get lookahead distance
   * @param cmd the current speed to use to compute lookahead point
   * @return lookahead distance
   */
  double getLookAheadDistance(const geometry_msgs::msg::Twist &);

  /**
   * @brief Creates a PointStamped message for visualization
   * @param carrot_pose Input carrot point as a PoseStamped
   * @return CarrotMsg a carrot point marker, PointStamped
   */
  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Whether robot should rotate to rough path heading
   * @param carrot_pose current lookahead point
   * @param angle_to_path Angle of robot output relatie to carrot marker
   * @return Whether should rotate to path heading
   */
  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path);

  /**
   * @brief Whether robot should rotate to final goal orientation
   * @param carrot_pose current lookahead point
   * @return Whether should rotate to goal heading
   */
  bool shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Create a smooth and kinematically smoothed rotation command
   * @param linear_vel linear velocity
   * @param angular_vel angular velocity
   * @param angle_to_path Angle of robot output relatie to carrot marker
   * @param curr_speed the current robot speed
   */
  void rotateToHeading(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed);

  /**
   * @brief Whether collision is imminent
   * @param robot_pose Pose of robot
   * @param carrot_pose Pose of carrot
   * @param linear_vel linear velocity to forward project
   * @param angular_vel angular velocity to forward project
   * @param carrot_dist Distance to the carrot for PP
   * @return Whether collision is imminent
   */
  bool isCollisionImminent(
    const geometry_msgs::msg::PoseStamped &,
    const double &, const double &,
    const double &);

  /**
   * @brief checks for collision at projected pose
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @param theta orientation of Yaw
   * @return Whether in collision
   */
  bool inCollision(
    const double & x,
    const double & y,
    const double & theta);
  /**
   * @brief Cost at a point
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @return Cost of pose in costmap
   */
  double costAtPose(const double & x, const double & y);

  double approachVelocityScalingFactor(
    const nav_msgs::msg::Path & path
  ) const;

  void applyApproachVelocityScaling(
    const nav_msgs::msg::Path & path,
    double & linear_vel
  ) const;

  /**
   * @brief apply regulation constraints to the system
   * @param linear_vel robot command linear velocity input
   * @param lookahead_dist optimal lookahead distance
   * @param curvature curvature of path
   * @param speed Speed of robot
   * @param pose_cost cost at this pose
   */
  void applyConstraints(
    const double & curvature, const geometry_msgs::msg::Twist & speed,
    const double & pose_cost, const nav_msgs::msg::Path & path,
    double & linear_vel, double & sign);

  /**
   * @brief Find the intersection a circle and a line segment.
   * This assumes the circle is centered at the origin.
   * If no intersection is found, a floating point error will occur.
   * @param p1 first endpoint of line segment
   * @param p2 second endpoint of line segment
   * @param r radius of circle
   * @return point of intersection
   */
  static geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    double r);

  /**
   * @brief Get lookahead point
   * @param lookahead_dist Optimal lookahead distance
   * @param path Current global path
   * @return Lookahead point
   */
  geometry_msgs::msg::PoseStamped getLookAheadPoint(const double &, const nav_msgs::msg::Path &);

  /**
   * @brief checks for the cusp position
   * @param pose Pose input to determine the cusp position
   * @return robot distance from the cusp
   */
  double findVelocitySignChange(const nav_msgs::msg::Path & transformed_plan);

  /**
   * Get the greatest extent of the costmap in meters from the center.
   * @return max of distance from center in meters to edge of costmap
   */
  double getCostmapMaxExtent() const;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;        // 父生命周期节点的弱引用（controller_server）
  std::shared_ptr<tf2_ros::Buffer> tf_;                 // TF2 缓冲区，用于坐标变换查询
  std::string plugin_name_;                             // 本插件名称（参数命名空间）
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  // 局部代价地图 ROS 封装对象
  nav2_costmap_2d::Costmap2D * costmap_;                // 局部代价地图底层指针（由 costmap_ros_ 管理）
  rclcpp::Logger logger_ {rclcpp::get_logger("RegulatedPurePursuitController")};  // 日志器
  rclcpp::Clock::SharedPtr clock_;                      // 节点时钟

  double desired_linear_vel_, base_desired_linear_vel_;  // 当前/基础期望线速度（速度限制会修改 desired）
  double lookahead_dist_;                                // 固定前瞻距离（未启用速度自适应时使用）
  double rotate_to_heading_angular_vel_;                  // 旋转到路径朝向时的角速度
  double max_lookahead_dist_;                             // 前瞻距离上限（速度自适应模式下）
  double min_lookahead_dist_;                            // 前瞻距离下限（速度自适应模式下）
  double lookahead_time_;                                 // 速度自适应前瞻的时间系数：L = v * t
  bool use_velocity_scaled_lookahead_dist_;              // 是否启用随速度缩放的前瞻距离
  tf2::Duration transform_tolerance_;                     // TF 变换容差时间
  double min_approach_linear_velocity_;                  // 接近目标终点时的最小线速度
  double approach_velocity_scaling_dist_;                // 开始接近减速的距离阈值
  double control_duration_;                              // 控制周期时长（秒）
  double max_allowed_time_to_collision_up_to_carrot_;    // 前向碰撞检查投影的最大时长
  bool use_collision_detection_;                          // 是否启用碰撞检测
  bool use_regulated_linear_velocity_scaling_;           // 是否启用基于曲率的速度调节
  bool use_cost_regulated_linear_velocity_scaling_;      // 是否启用基于代价值的速度调节
  double cost_scaling_dist_;                             // 代价调节的起始距离阈值
  double cost_scaling_gain_;                             // 代价调节增益
  double inflation_cost_scaling_factor_;                 // 膨胀层代价缩放因子（用于 costAtPose）
  double regulated_linear_scaling_min_radius_;           // 曲率调节的最小半径阈值（低于此半径速度降到最小）
  double regulated_linear_scaling_min_speed_;            // 曲率调节下的最小速度
  bool use_rotate_to_heading_;                           // 角度偏差过大时是否原地旋转对准路径
  double max_angular_accel_;                             // 最大角加速度（用于平滑旋转指令）
  double rotate_to_heading_min_angle_;                   // 触发“旋转到路径朝向”的最小角度偏差
  double goal_dist_tol_;                                 // 到达目标的距离容差（用于旋转到目标朝向判断）
  bool allow_reversing_;                                 // 是否允许倒车跟踪路径
  double max_robot_pose_search_dist_;                   // 在路径上搜索机器人最近点的最大距离
  bool use_interpolation_;                               // 是否在前瞻点搜索中使用线性插值

  nav_msgs::msg::Path global_plan_;  // 当前接收到的全局路径（在机器人坐标系下的截断版本见 transformGlobalPlan）
  // 调试/可视化发布器：变换后的局部路径
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  // 调试/可视化发布器：前瞻点（carrot 胡萝卜点）标记
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
  carrot_pub_;
  // 调试/可视化发布器：跟踪圆弧路径
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> carrot_arc_pub_;
  // 足迹碰撞检查器：对给定位姿用机器人足迹在代价地图上检查碰撞
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;

  // Dynamic parameters handler（动态参数回调处理）
  std::mutex mutex_;  // 保护动态参数修改的互斥锁
  // ROS2 参数变化回调句柄（运行时动态调参）
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace nav2_regulated_pure_pursuit_controller

#endif  // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
