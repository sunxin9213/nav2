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

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_

#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/critic_data.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::CollisionCost
 * @brief Utility for storing cost information
 *（碰撞代价结构：记录轨迹点在代价地图上的代价值与计算方式）
 */
struct CollisionCost
{
  float cost{0};              // 代价地图上查询到的代价值
  bool using_footprint{false};  // 是否使用完整足迹（而非单点）计算的代价
};

/**
 * @class mppi::critics::CriticFunction
 * @brief Abstract critic objective function to score trajectories
 *（critic 基类：MPPI 的代价函数插件抽象接口）
 *
 * 中文说明：
 * 每个 critic 实现一个目标函数项，对 batch_size 条候选轨迹逐条打分，
 * 各 critic 的分数累加到 CriticData::costs 中，最终参与 softmax 加权。
 * critic 以 pluginlib 插件形式加载（见 CriticManager::loadCritics），
 * 常见 critic 包括：障碍物、路径对齐、路径跟随、朝向角、目标、
 * 速度约束、代价地图等（见 critics/ 目录）。
 */
class CriticFunction
{
public:
  /**
    * @brief Constructor for mppi::critics::CriticFunction
    */
  CriticFunction() = default;

  /**
    * @brief Destructor for mppi::critics::CriticFunction
    */
  virtual ~CriticFunction() = default;

  /**
    * @brief Configure critic on bringup
    * @param parent WeakPtr to node
    * @param parent_name name of the controller
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param dynamic_parameter_handler Parameter handler object
    *（配置 critic：由 CriticManager 在加载插件后调用，模板方法，
    *  保存公共上下文（节点/代价地图/参数）后调用子类 initialize()）
    */
  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string & parent_name,
    const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    ParametersHandler * param_handler)
  {
    // 保存节点、插件名、控制器名与代价地图引用
    parent_ = parent;
    logger_ = parent_.lock()->get_logger();
    name_ = name;
    parent_name_ = parent_name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    parameters_handler_ = param_handler;

    // 读取每个 critic 通用的 enabled 参数（可单独禁用某 critic）
    auto getParam = parameters_handler_->getParamGetter(name_);
    getParam(enabled_, "enabled", true);

    // 调用子类初始化（读取各自特有参数）
    initialize();
  }

  /**
    * @brief Main function to score trajectory
    * @param data Critic data to use in scoring
    *（评分接口：子类实现，对 data.trajectories 中的所有轨迹
    *  逐条累加代价到 data.costs）
    */
  virtual void score(CriticData & data) = 0;

  /**
    * @brief Initialize critic
    *（子类初始化：读取 critic 特有参数）
    */
  virtual void initialize() = 0;

  /**
    * @brief Get name of critic
    */
  std::string getName()
  {
    return name_;
  }

protected:
  bool enabled_;             // 本 critic 是否启用
  std::string name_, parent_name_;  // critic 插件名 / 所属控制器名
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;              // 父节点
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;    // 代价地图封装
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};                // 代价地图底层指针

  ParametersHandler * parameters_handler_;  // 参数处理器
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};  // 日志器
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
