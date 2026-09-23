// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2019 Intel Corporation
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

// ============================================================================
// 文件说明（中文）：
// SmootherServer（路径平滑服务器）类的头文件声明。
// 模块定位：Nav2 框架中的平滑任务服务器节点。它本身不实现平滑算法，
// 而是通过 pluginlib 加载实现了 nav2_core::Smoother 接口的算法插件
// （如 SimpleSmoother / SavitzkyGolaySmoother / ConstrainedSmoother），
// 并对外提供 nav2_msgs::action::SmoothPath 动作服务。
// 类继承 nav2_util::LifecycleNode（ROS2 生命周期节点），
// 按 configure -> activate -> deactivate -> cleanup -> shutdown 生命周期运行。
// 本头文件实现（.cpp）见 src/nav2_smoother.cpp。
// ============================================================================

#ifndef NAV2_SMOOTHER__NAV2_SMOOTHER_HPP_
#define NAV2_SMOOTHER__NAV2_SMOOTHER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"

namespace nav2_smoother
{

/**
 * @class nav2_smoother::SmootherServer
 * @brief This class hosts variety of plugins of different algorithms to
 * smooth or refine a path from the exposed SmoothPath action server.
 */
class SmootherServer : public nav2_util::LifecycleNode
{
public:
  // 插件名 -> 平滑插件实例 的映射表类型
  using SmootherMap = std::unordered_map<std::string, nav2_core::Smoother::Ptr>;

  /**
   * @brief A constructor for nav2_smoother::SmootherServer
   * @param options Additional options to control creation of the node.
   */
  explicit SmootherServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for nav2_smoother::SmootherServer
   */
  ~SmootherServer();

protected:
  /**
   * @brief Configures smoother parameters and member variables
   *
   * Configures smoother plugin and costmap; Initialize odom subscriber,
   * velocity publisher and smooth path action server.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize smoother
   * plugin
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Loads smoother plugins from parameter file
   * @return bool if successfully loaded the plugins
   */
  bool loadSmootherPlugins();

  /**
   * @brief Activates member variables
   *
   * Activates smoother, costmap, velocity publisher and smooth path action
   * server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivates member variables
   *
   * Deactivates smooth path action server, smoother, costmap and velocity
   * publisher. Before calling deactivate state, velocity is being set to zero.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Calls clean up states and resets member variables.
   *
   * Smoother and costmap clean up state is called, and resets rest of the
   * variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // 本节点实现的动作类型：nav2_msgs/action/SmoothPath（平滑路径）
  using Action = nav2_msgs::action::SmoothPath;
  // 简单动作服务器封装（nav2_util），负责目标接收/抢占/结果返回的线程管理
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  /**
   * @brief SmoothPath action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * Provides global path to smoother received from action client. Local
   * section of the path is optimized using smoother.
   * @throw nav2_core::PlannerException
   */
  void smoothPlan();

  /**
   * @brief Find the valid smoother ID name for the given request
   *
   * @param c_name The requested smoother name
   * @param name Reference to the name to use for control if any valid available
   * @return bool Whether it found a valid smoother to use
   */
  bool findSmootherId(const std::string & c_name, std::string & name);

  // Our action server implements the SmoothPath action
  // SmoothPath 动作服务器（回调为 smoothPlan()）
  std::unique_ptr<ActionServer> action_server_;

  // Transforms
  // TF 坐标变换：缓冲区 + 监听器（订阅 /tf、/tf_static）
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  // Publishers and subscribers
  // 平滑后路径发布器（话题 plan_smoothed，供可视化/下游使用）
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

  // Smoother Plugins
  // pluginlib 类加载器，用于按类型字符串动态创建平滑插件实例
  pluginlib::ClassLoader<nav2_core::Smoother> lp_loader_;
  SmootherMap smoothers_;           // 已加载的插件表（名字 -> 实例）
  std::vector<std::string> default_ids_;    // 默认插件名（未配置参数时使用）
  std::vector<std::string> default_types_;  // 默认插件类型字符串
  std::vector<std::string> smoother_ids_;   // 参数中配置的插件名列表
  std::vector<std::string> smoother_types_; // 对应的插件类型字符串
  // 所有插件名的拼接串（日志用）/ 当前动作正在使用的插件名
  std::string smoother_ids_concat_, current_smoother_;

  // Utilities
  // 代价地图订阅器（全局代价地图原始数据，供插件与碰撞检查使用）
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  // 机器人足迹订阅器（碰撞检查需要足迹轮廓）
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  // 基于话题数据的碰撞检查器（结合代地图+足迹判断位姿是否碰撞）
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
};

}  // namespace nav2_smoother

#endif  // NAV2_SMOOTHER__NAV2_SMOOTHER_HPP_
