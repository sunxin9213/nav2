// Copyright (c) 2022 Samsung R&D Institute Russia
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

#ifndef NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
#define NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_

// ============================================================================
// 文件说明（中文）：
// CollisionMonitor（碰撞监视器）节点的头文件声明。
// 模块定位：Nav2 的实时安全层（非生命周期服务器，而是数据流节点）。
// 工作方式：串接在速度命令链路上——订阅控制器输出的 cmd_vel_in，
// 结合多传感器障碍点与多层安全区（STOP 停障 / SLOWDOWN 减速 /
// APPROACH 预测逼近）检测碰撞风险，将修正后的速度发布到 cmd_vel_out。
// 即使规划器/控制器失效，该层仍能独立保证安全。
// 详细中文注释的实现见 src/collision_monitor_node.cpp。
// ============================================================================

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/polygon.hpp"
#include "nav2_collision_monitor/circle.hpp"
#include "nav2_collision_monitor/source.hpp"
#include "nav2_collision_monitor/scan.hpp"
#include "nav2_collision_monitor/pointcloud.hpp"
#include "nav2_collision_monitor/range.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Collision Monitor ROS2 node
 */
class CollisionMonitor : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for the nav2_collision_safery::CollisionMonitor
   * @param options Additional options to control creation of the node.
   */
  explicit CollisionMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for the nav2_collision_safery::CollisionMonitor
   */
  ~CollisionMonitor();

protected:
  /**
   * @brief: Initializes and obtains ROS-parameters, creates main subscribers and publishers,
   * creates polygons and data sources objects
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Activates LifecyclePublishers, polygons and main processor, creates bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Deactivates LifecyclePublishers, polygons and main processor, destroys bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Resets all subscribers/publishers, polygons/data sources arrays
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called in shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

protected:
  /**
   * @brief Callback for input cmd_vel
   * @param msg Input cmd_vel message
   */
  void cmdVelInCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  /**
   * @brief Publishes output cmd_vel. If robot was stopped more than stop_pub_timeout_ seconds,
   * quit to publish 0-velocity.
   * @param robot_action Robot action to publish
   */
  void publishVelocity(const Action & robot_action);

  /**
   * @brief Supporting routine obtaining all ROS-parameters
   * @param cmd_vel_in_topic Output name of cmd_vel_in topic
   * @param cmd_vel_out_topic Output name of cmd_vel_out topic
   * is required.
   * @return True if all parameters were obtained or false in failure case
   */
  bool getParameters(
    std::string & cmd_vel_in_topic,
    std::string & cmd_vel_out_topic);
  /**
   * @brief Supporting routine creating and configuring all polygons
   * @param base_frame_id Robot base frame ID
   * @param transform_tolerance Transform tolerance
   * @return True if all polygons were configured successfully or false in failure case
   */
  bool configurePolygons(
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance);
  /**
   * @brief Supporting routine creating and configuring all data sources
   * @param base_frame_id Robot base frame ID
   * @param odom_frame_id Odometry frame ID. Used as global frame to get
   * source->base time inerpolated transform.
   * @param transform_tolerance Transform tolerance
   * @param source_timeout Maximum time interval in which data is considered valid
   * @param base_shift_correction Whether to correct source data towards to base frame movement,
   * considering the difference between current time and latest source time
   * @return True if all sources were configured successfully or false in failure case
   */
  bool configureSources(
    const std::string & base_frame_id,
    const std::string & odom_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout,
    const bool base_shift_correction);

  /**
   * @brief Main processing routine
   * @param cmd_vel_in Input desired robot velocity
   */
  void process(const Velocity & cmd_vel_in);

  /**
   * @brief Processes the polygon of STOP and SLOWDOWN action type
   * @param polygon Polygon to process
   * @param collision_points Array of 2D obstacle points
   * @param velocity Desired robot velocity
   * @param robot_action Output processed robot action
   * @return True if returned action is caused by current polygon, otherwise false
   */
  bool processStopSlowdown(
    const std::shared_ptr<Polygon> polygon,
    const std::vector<Point> & collision_points,
    const Velocity & velocity,
    Action & robot_action) const;

  /**
   * @brief Processes APPROACH action type
   * @param polygon Polygon to process
   * @param collision_points Array of 2D obstacle points
   * @param velocity Desired robot velocity
   * @param robot_action Output processed robot action
   * @return True if returned action is caused by current polygon, otherwise false
   */
  bool processApproach(
    const std::shared_ptr<Polygon> polygon,
    const std::vector<Point> & collision_points,
    const Velocity & velocity,
    Action & robot_action) const;

  /**
   * @brief Prints robot action and polygon caused it (if it was)
   * @param robot_action Robot action to print
   * @param action_polygon Pointer to a polygon causing a selected action
   */
  void printAction(
    const Action & robot_action, const std::shared_ptr<Polygon> action_polygon) const;

  /**
   * @brief Polygons publishing routine. Made for visualization.
   */
  void publishPolygons() const;

  // ----- Variables -----

  /// @brief TF buffer
  /// TF 缓冲（所有数据源坐标变换共用）
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  /// @brief TF listener
  /// TF 监听器（订阅 /tf、/tf_static）
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// @brief Polygons array
  /// 多层安全区列表（Polygon 或 Circle 实例）
  std::vector<std::shared_ptr<Polygon>> polygons_;

  /// @brief Data sources array
  /// 数据源列表（Scan / PointCloud / Range 实例）
  std::vector<std::shared_ptr<Source>> sources_;

  // Input/output speed controls
  /// @beirf Input cmd_vel subscriber
  /// 输入速度指令订阅器（来自控制器）
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_in_sub_;
  /// @brief Output cmd_vel publisher
  /// 输出速度指令发布器（发往底盘）
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;

  /// @brief Whether main routine is active
  /// 主处理流程是否激活（cmd_vel_in 回调中检查）
  bool process_active_;

  /// @brief Previous robot action
  /// 上一次输出的机器人动作（用于动作变化检测）
  Action robot_action_prev_;
  /// @brief Latest timestamp when robot has 0-velocity
  /// 最近一次速度为 0 的时刻
  rclcpp::Time stop_stamp_;
  /// @brief Timeout after which 0-velocity ceases to be published
  /// 持续停车超过该时限后不再发布 0 速度（避免长期占用话题带宽）
  rclcpp::Duration stop_pub_timeout_;
};  // class CollisionMonitor

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
