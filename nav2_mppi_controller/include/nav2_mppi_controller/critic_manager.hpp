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

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <pluginlib/class_loader.hpp>
#include <xtensor/xtensor.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_mppi_controller/critic_function.hpp"

namespace mppi
{

/**
 * @class mppi::CriticManager
 * @brief Manager of objective function plugins for scoring trajectories
 *（critic 管理器：负责加载并调度所有代价函数插件）
 *
 * 中文说明：
 * 从参数 critics（字符串数组）读取要加载的 critic 列表，
 * 通过 pluginlib ClassLoader 动态实例化（如 "mppi::critics::ConstraintCritic"），
 * 评分时按加载顺序依次调用每个 critic 的 score()，各 critic 的代价
 * 累加到同一 costs 张量上，实现多个目标项的线性组合。
 */
class CriticManager
{
public:
  /**
    * @brief Constructor for mppi::CriticManager
    */
  CriticManager() = default;

  /**
    * @brief Virtual Destructor for mppi::CriticManager
    */
  virtual ~CriticManager() = default;

  /**
    * @brief Configure critic manager on bringup and load plugins
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>, ParametersHandler *);

  /**
    * @brief Score trajectories by the set of loaded critic functions
    * @param CriticData Struct of necessary information to pass to the critic functions
    */
  void evalTrajectoriesScores(CriticData & data) const;

protected:
  /**
    * @brief Get parameters (critics to load)
    */
  void getParams();

  /**
    * @brief Load the critic plugins
    */
  virtual void loadCritics();

  /**
    * @brief Get full-name namespaced critic IDs
    */
  std::string getFullName(const std::string & name);

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;  // 父节点弱引用
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  // 代价地图封装
  std::string name_;  // 控制器插件名

  ParametersHandler * parameters_handler_;  // 参数处理器
  std::vector<std::string> critic_names_;   // 待加载的 critic 名称列表（来自参数）
  // pluginlib 类加载器：负责动态加载 CriticFunction 派生类
  std::unique_ptr<pluginlib::ClassLoader<critics::CriticFunction>> loader_;
  std::vector<std::unique_ptr<critics::CriticFunction>> critics_;  // 已加载的 critic 实例

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};  // 日志器
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_
