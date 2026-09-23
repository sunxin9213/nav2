// Copyright (c) 2019 RoboTech Vision
// Copyright (c) 2019 Intel Corporation
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

// ============================================================================
// 文件说明（中文）：
// 本文件是 Nav2 导航框架中"路径平滑服务器"（SmootherServer）的核心实现。
// 模块定位：nav2_smoother 是 Nav2 的路径平滑任务服务器，它以 ROS2 生命周期节点
// （LifecycleNode）的形式运行，对外暴露 nav2_msgs::action::SmoothPath 动作（action），
// 接收规划器输出的全局路径，调用具体平滑算法插件（如 SimpleSmoother、
// SavitzkyGolaySmoother 或 ConstrainedSmoother）对路径进行平滑优化，
// 并可选地对平滑结果进行碰撞检查（基于全局代价地图 costmap 与机器人足迹 footprint）。
//
// 主要工作流程：
//   1. on_configure：声明/读取参数，创建 TF 缓冲与监听器、代价地图订阅器、
//      足迹订阅器、碰撞检查器，加载平滑插件，创建 SmoothPath 动作服务器。
//   2. smoothPlan（动作回调）：按请求中的 smoother_id 选择插件执行平滑，
//      发布平滑后的路径，若请求要求则逐点做碰撞检测，最后返回动作结果。
// ============================================================================

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_smoother/nav2_smoother.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "tf2_ros/create_timer_ros.h"

using namespace std::chrono_literals;

namespace nav2_smoother
{

// ---------------------------------------------------------------------------
// 构造函数：初始化生命周期节点 smoother_server。
// lp_loader_ 是 pluginlib 的类加载器，用于在运行时动态加载实现了
// nav2_core::Smoother 接口的平滑算法插件（基库为 "nav2_core"）。
// default_ids_/default_types_ 为未配置 smoother_plugins 参数时的默认插件。
// ---------------------------------------------------------------------------
SmootherServer::SmootherServer(const rclcpp::NodeOptions & options)
: LifecycleNode("smoother_server", "", options),
  lp_loader_("nav2_core", "nav2_core::Smoother"),
  default_ids_{"simple_smoother"},
  default_types_{"nav2_smoother::SimpleSmoother"}
{
  RCLCPP_INFO(get_logger(), "Creating smoother server");

  // 声明本节点所需的 ROS2 参数：
  // costmap_topic：全局代价地图原始数据话题（用于碰撞检查）
  declare_parameter(
    "costmap_topic", rclcpp::ParameterValue(
      std::string(
        "global_costmap/costmap_raw")));
  // footprint_topic：机器人足迹（多边形轮廓）话题
  declare_parameter(
    "footprint_topic",
    rclcpp::ParameterValue(
      std::string("global_costmap/published_footprint")));
  // robot_base_frame：机器人本体坐标系（TF 坐标系名称，默认 base_link）
  declare_parameter(
    "robot_base_frame",
    rclcpp::ParameterValue(std::string("base_link")));
  // transform_tolerance：TF 变换查询的容差时间（秒）
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));
  // smoother_plugins：要加载的平滑插件名称列表（string 数组）
  declare_parameter("smoother_plugins", default_ids_);
}

// 析构函数：清空插件映射表，释放插件资源
SmootherServer::~SmootherServer()
{
  smoothers_.clear();
}

// ---------------------------------------------------------------------------
// 生命周期回调 on_configure：节点配置阶段（inactive 状态前调用）。
// 完成参数读取、TF、订阅器、碰撞检查器、插件以及动作服务器的初始化。
// ---------------------------------------------------------------------------
nav2_util::CallbackReturn
SmootherServer::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring smoother server");

  auto node = shared_from_this();

  // 读取平滑插件名称列表；若用户未自定义（仍为默认值），
  // 则为默认插件声明 "<插件名>.plugin" 参数（指定插件的实现类型字符串）
  get_parameter("smoother_plugins", smoother_ids_);
  if (smoother_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
    }
  }

  // 创建 TF2 缓冲区（用于坐标变换查找），并绑定 ROS 定时器接口，
  // 使 TF 缓冲能在 ROS 事件循环中异步等待变换可用
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  // 创建 TF 监听器，订阅 /tf 与 /tf_static 话题以填充 TF 缓冲
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  // 读取碰撞检查相关的参数
  std::string costmap_topic, footprint_topic, robot_base_frame;
  double transform_tolerance;
  this->get_parameter("costmap_topic", costmap_topic);
  this->get_parameter("footprint_topic", footprint_topic);
  this->get_parameter("transform_tolerance", transform_tolerance);
  this->get_parameter("robot_base_frame", robot_base_frame);

  // 创建代价地图订阅器（订阅全局代价地图原始数据，而非自建 costmap）
  costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
    shared_from_this(), costmap_topic);
  // 创建足迹订阅器（订阅并转换机器人足迹到本体坐标系下）
  footprint_sub_ = std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
    shared_from_this(), footprint_topic, *tf_, robot_base_frame, transform_tolerance);

  // 创建基于话题的碰撞检查器：结合代价地图与足迹判断某位姿是否发生碰撞
  collision_checker_ =
    std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
    *costmap_sub_, *footprint_sub_, this->get_name());

  // 加载平滑插件；失败则配置阶段返回 FAILURE，生命周期管理器会终止启动
  if (!loadSmootherPlugins()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // Initialize pubs & subs
  // 创建平滑后路径的发布器（话题 plan_smoothed，深度 1）
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan_smoothed", 1);

  // Create the action server that we implement with our smoothPath method
  // 创建 SmoothPath 动作服务器：回调绑定到 smoothPlan()；
  // 500ms 为服务器休眠/超时相关的时间片，true 表示允许在执行期间处理抢占请求
  action_server_ = std::make_unique<ActionServer>(
    shared_from_this(),
    "smooth_path",
    std::bind(&SmootherServer::smoothPlan, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  return nav2_util::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// 加载所有配置的平滑插件：
// 通过 pluginlib 根据参数中指定的类型字符串动态创建插件实例，
// 依次调用其 configure() 完成初始化，并以 "插件名 -> 实例" 存入映射表。
// ---------------------------------------------------------------------------
bool SmootherServer::loadSmootherPlugins()
{
  auto node = shared_from_this();

  smoother_types_.resize(smoother_ids_.size());

  // 遍历所有插件名称，逐个加载
  for (size_t i = 0; i != smoother_ids_.size(); i++) {
    try {
      // 从参数 "<插件名>.plugin" 中取出插件的完整类型名（如 nav2_smoother::SimpleSmoother）
      smoother_types_[i] =
        nav2_util::get_plugin_type_param(node, smoother_ids_[i]);
      // 通过 pluginlib 类加载器创建插件实例（工厂模式）
      nav2_core::Smoother::Ptr smoother =
        lp_loader_.createUniqueInstance(smoother_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created smoother : %s of type %s",
        smoother_ids_[i].c_str(), smoother_types_[i].c_str());
      // 调用插件自身的配置函数，传入节点句柄、TF 与代价地图/足迹订阅器
      smoother->configure(
        node, smoother_ids_[i], tf_, costmap_sub_,
        footprint_sub_);
      smoothers_.insert({smoother_ids_[i], smoother});
    } catch (const pluginlib::PluginlibException & ex) {
      // 任一插件加载失败都视为致命错误：打印并返回 false
      RCLCPP_FATAL(
        get_logger(), "Failed to create smoother. Exception: %s",
        ex.what());
      return false;
    }
  }

  // 拼接所有插件名（用于日志与错误提示中列出可用插件）
  for (size_t i = 0; i != smoother_ids_.size(); i++) {
    smoother_ids_concat_ += smoother_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(), "Smoother Server has %s smoothers available.",
    smoother_ids_concat_.c_str());

  return true;
}

// ---------------------------------------------------------------------------
// 生命周期回调 on_activate：激活阶段。
// 激活路径发布器、所有插件与动作服务器，并创建 bond 连接
// （bond 用于向生命周期管理器报告本进程存活状态）。
// ---------------------------------------------------------------------------
nav2_util::CallbackReturn
SmootherServer::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  plan_publisher_->on_activate();
  SmootherMap::iterator it;
  for (it = smoothers_.begin(); it != smoothers_.end(); ++it) {
    it->second->activate();
  }
  action_server_->activate();

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// 生命周期回调 on_deactivate：停用阶段。
// 与 on_activate 顺序相反：先停用动作服务器与插件，再停用发布器，最后断开 bond。
// ---------------------------------------------------------------------------
nav2_util::CallbackReturn
SmootherServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  SmootherMap::iterator it;
  for (it = smoothers_.begin(); it != smoothers_.end(); ++it) {
    it->second->deactivate();
  }
  plan_publisher_->on_deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// 生命周期回调 on_cleanup：清理阶段。
// 调用每个插件的 cleanup()，随后将本节点持有的各类资源（动作服务器、
// 发布器、TF、订阅器、碰撞检查器）全部 reset 释放。
// ---------------------------------------------------------------------------
nav2_util::CallbackReturn
SmootherServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Cleanup the helper classes
  SmootherMap::iterator it;
  for (it = smoothers_.begin(); it != smoothers_.end(); ++it) {
    it->second->cleanup();
  }
  smoothers_.clear();

  // Release any allocated resources
  action_server_.reset();
  plan_publisher_.reset();
  transform_listener_.reset();
  tf_.reset();
  footprint_sub_.reset();
  costmap_sub_.reset();
  collision_checker_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

// 生命周期回调 on_shutdown：关闭阶段，仅打印日志（无需额外清理）
nav2_util::CallbackReturn
SmootherServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// 根据动作请求中的 smoother_id 查找可用的平滑插件名：
//  - 若名字存在于插件映射表中，直接选用；
//  - 若名字为空且只加载了一个插件，则默认使用该唯一插件（仅告警一次）；
//  - 否则报错并返回 false，动作将被终止。
// ---------------------------------------------------------------------------
bool SmootherServer::findSmootherId(
  const std::string & c_name,
  std::string & current_smoother)
{
  if (smoothers_.find(c_name) == smoothers_.end()) {
    if (smoothers_.size() == 1 && c_name.empty()) {
      // 请求未指定插件名且服务器只加载了一个插件：使用唯一插件并提示（仅一次）
      RCLCPP_WARN_ONCE(
        get_logger(),
        "No smoother was specified in action call."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.",
        smoother_ids_concat_.c_str());
      current_smoother = smoothers_.begin()->first;
    } else {
      // 请求的插件名不存在：报错并列出当前可用的插件
      RCLCPP_ERROR(
        get_logger(),
        "SmoothPath called with smoother name %s, "
        "which does not exist. Available smoothers are: %s.",
        c_name.c_str(), smoother_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected smoother: %s.", c_name.c_str());
    current_smoother = c_name;
  }

  return true;
}

// ---------------------------------------------------------------------------
// SmoothPath 动作服务器的主回调：执行路径平滑的完整流程。
// 步骤：取目标 -> 选择插件 -> 平滑 -> 发布结果 -> （可选）碰撞检查 -> 返回结果。
// ---------------------------------------------------------------------------
void SmootherServer::smoothPlan()
{
  auto start_time = this->now();

  RCLCPP_INFO(get_logger(), "Received a path to smooth.");

  auto result = std::make_shared<Action::Result>();
  try {
    // 获取当前待处理的动作目标；若服务器已被停用则目标为空，直接返回
    auto goal = action_server_->get_current_goal();
    if (!goal) {
      return;  //  if action_server_ is inactivate, goal would be a nullptr
    }

    // 根据请求中的 smoother_id 选择本次使用的平滑插件
    std::string c_name = goal->smoother_id;
    std::string current_smoother;
    if (findSmootherId(c_name, current_smoother)) {
      current_smoother_ = current_smoother;
    } else {
      // 无效插件名：终止当前动作请求
      action_server_->terminate_current();
      return;
    }

    // Perform smoothing
    // 复制原始路径到结果中，调用插件的 smooth()（受最大平滑时长约束），
    // 返回值 was_completed 表示是否在时限内完成
    result->path = goal->path;
    result->was_completed = smoothers_[current_smoother_]->smooth(
      result->path, goal->max_smoothing_duration);
    result->smoothing_duration = this->now() - start_time;

    // 若平滑因超时被打断，记录提示信息（结果仍会返回已平滑部分）
    if (!result->was_completed) {
      RCLCPP_INFO(
        get_logger(),
        "Smoother %s did not complete smoothing in specified time limit"
        "(%lf seconds) and was interrupted after %lf seconds",
        current_smoother_.c_str(),
        rclcpp::Duration(goal->max_smoothing_duration).seconds(),
        rclcpp::Duration(result->smoothing_duration).seconds());
    }

    // 发布平滑后的路径到 plan_smoothed 话题，便于 RViz 等可视化
    plan_publisher_->publish(result->path);

    // Check for collisions
    // 可选步骤：若请求要求检查碰撞，则对平滑后路径的每个位姿做碰撞检测
    if (goal->check_for_collisions) {
      geometry_msgs::msg::Pose2D pose2d;
      bool fetch_data = true;
      for (const auto & pose : result->path.poses) {
        // 将 PoseStamped 转为 Pose2D（位置 + 偏航角）
        pose2d.x = pose.pose.position.x;
        pose2d.y = pose.pose.position.y;
        pose2d.theta = tf2::getYaw(pose.pose.orientation);

        // 使用基于话题的碰撞检查器检测该位姿是否碰撞
        // fetch_data 仅在首个点为 true：第一次从订阅器拉取代价地图/足迹缓存
        if (!collision_checker_->isCollisionFree(pose2d, fetch_data)) {
          RCLCPP_ERROR(
            get_logger(),
            "Smoothed path leads to a collision at x: %lf, y: %lf, theta: %lf",
            pose2d.x, pose2d.y, pose2d.theta);
          // 检测到碰撞：以该（有碰撞的）结果终止当前动作，客户端据此判定失败
          action_server_->terminate_current(result);
          return;
        }
        fetch_data = false;
      }
    }

    RCLCPP_DEBUG(
      get_logger(), "Smoother succeeded (time: %lf), setting result",
      rclcpp::Duration(result->smoothing_duration).seconds());

    // 全部成功：动作以 succeeded 状态结束并携带结果
    action_server_->succeeded_current(result);
  } catch (nav2_core::PlannerException & e) {
    // 捕获规划器类异常（如插件内部抛出），终止动作（无结果）
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    action_server_->terminate_current();
    return;
  } catch (std::exception & ex) {
    // 捕获其他标准异常，终止动作并携带已生成的结果
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    action_server_->terminate_current(result);
    return;
  }
}

}  // namespace nav2_smoother

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
// 注册组件节点宏：使本类可作为 rclcpp_components 组件被动态加载运行（容器进程）
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_smoother::SmootherServer)
