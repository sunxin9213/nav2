// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_NAVFN_PLANNER__NAVFN_PLANNER_HPP_
#define NAV2_NAVFN_PLANNER__NAVFN_PLANNER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_navfn_planner
{

// 【中文注释】NavfnPlanner：NavFn 全局规划器的 ROS2 插件封装类。
// 继承 nav2_core::GlobalPlanner 接口，通过 pluginlib 加载到 planner_server 中，
// 负责生命周期管理、参数读取、坐标转换，并把实际的势场计算/路径回溯
// 委托给核心算法类 NavFn（navfn.hpp/navfn.cpp）。
// 算法类型：Dijkstra/A* 势场扩散 + 梯度下降回溯（详见 navfn.cpp 头部注释）。
class NavfnPlanner : public nav2_core::GlobalPlanner
{
public:
  /**
   * @brief constructor
   */
  NavfnPlanner();

  /**
   * @brief destructor
   */
  ~NavfnPlanner();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  // 【中文注释】生命周期 configure：声明/读取参数、保存 costmap 和 TF 句柄、
  // 创建 NavFn 核心算法对象。对应 ROS2 导航中 "unconfigured -> inactive" 转换。
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;


  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return nav_msgs::Path of the generated path
   */
  // 【中文注释】插件主入口：planner_server 收到 ComputePathToPose 请求时调用。
  // 流程：世界坐标 -> 栅格坐标 -> NavFn 势场计算 -> 梯度回溯出路径 ->
  // 终点朝向修正，返回带 global_frame 头的 nav_msgs::Path。
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  /**
   * @brief Compute a plan given start and goal poses, provided in global world frame.
   * @param start Start pose
   * @param goal Goal pose
   * @param tolerance Relaxation constraint in x and y
   * @param plan Path to be computed
   * @return true if can find the path
   */
  // 【中文注释】内部规划实现：清空起点栅格代价 -> 同步 NavFn 网格尺寸 ->
  // 计算势场(computePotential) -> 回溯路径(getPlanFromPotential)。
  // tolerance 允许目标被障碍占据时在附近搜索最近可达点。
  bool makePlan(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal, double tolerance,
    nav_msgs::msg::Path & plan);

  /**
   * @brief Compute the navigation function given a seed point in the world to start from
   * @param world_point Point in world coordinate frame
   * @return true if can compute
   */
  // 【中文注释】以 world_point 为势场源点计算导航函数：
  // 把 costmap 导入 NavFn、设置源点(势值 0)、执行 Dijkstra/A* 扩散。
  bool computePotential(const geometry_msgs::msg::Point & world_point);

  /**
   * @brief Compute a plan to a goal from a potential - must call computePotential first
   * @param goal Goal pose
   * @param plan Path to be computed
   * @return true if can compute a plan path
   */
  // 【中文注释】从已算好的势场中回溯路径（不重算势场）：
  // 调用 NavFn::calcPath() 沿梯度下降从源点走向目标，再转回世界坐标。
  bool getPlanFromPotential(
    const geometry_msgs::msg::Pose & goal,
    nav_msgs::msg::Path & plan);

  /**
   * @brief Remove artifacts at the end of the path - originated from planning on a discretized world
   * @param goal Goal pose
   * @param plan Computed path
   */
  // 【中文注释】终点逼近平滑：离散回溯可能产生指向目标的锯齿末端，
  // 用倒数第二个点与目标的连线替换最后一段，消除离散化伪影。
  void smoothApproachToGoal(
    const geometry_msgs::msg::Pose & goal,
    nav_msgs::msg::Path & plan);

  /**
   * @brief Compute the potential, or navigation cost, at a given point in the world
   *        must call computePotential first
   * @param world_point Point in world coordinate frame
   * @return double point potential (navigation cost)
   */
  // 【中文注释】查询某世界坐标点的势值（导航代价），需先 computePotential。
  // 用于在 tolerance 范围内搜索目标附近的低势值可达点。
  double getPointPotential(const geometry_msgs::msg::Point & world_point);

  // Check for a valid potential value at a given point in the world
  // - must call computePotential first
  // - currently unused
  // bool validPointPotential(const geometry_msgs::msg::Point & world_point);
  // bool validPointPotential(const geometry_msgs::msg::Point & world_point, double tolerance);

  /**
   * @brief Compute the squared distance between two points
   * @param p1 Point 1
   * @param p2 Point 2
   * @return double squared distance between two points
   */
  inline double squared_distance(
    const geometry_msgs::msg::Pose & p1,
    const geometry_msgs::msg::Pose & p2)
  {
    double dx = p1.position.x - p2.position.x;
    double dy = p1.position.y - p2.position.y;
    return dx * dx + dy * dy;
  }

  /**
   * @brief Transform a point from world to map frame
   * @param wx double of world X coordinate
   * @param wy double of world Y coordinate
   * @param mx int of map X coordinate
   * @param my int of map Y coordinate
   * @return true if can transform
   */
  // 【中文注释】世界坐标 -> 地图栅格坐标（越界返回 false）
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);

  /**
   * @brief Transform a point from map to world frame
   * @param mx double of map X coordinate
   * @param my double of map Y coordinate
   * @param wx double of world X coordinate
   * @param wy double of world Y coordinate
   */
  // 【中文注释】地图栅格坐标 -> 世界坐标（栅格中心）
  void mapToWorld(double mx, double my, double & wx, double & wy);

  /**
   * @brief Set the corresponding cell cost to be free space
   * @param mx int of map X coordinate
   * @param my int of map Y coordinate
   */
  // 【中文注释】把机器人所在栅格的代价临时清为自由空间：
  // 机器人可能因定位误差/膨胀层而站在"代价较高"的格子里，不清除会导致规划失败
  void clearRobotCell(unsigned int mx, unsigned int my);

  /**
   * @brief Determine if a new planner object should be made
   * @return true if planner object is out of date
   */
  // 【中文注释】costmap 尺寸与 NavFn 内部网格不一致（如 rolling window 移动）时
  // 返回 true，提示需要重建/重设 planner 的网格
  bool isPlannerOutOfDate();

  // Planner based on ROS1 NavFn algorithm
  // 【中文注释】核心算法对象（继承自 ROS1 NavFn），持有势场、代价等所有内部缓冲
  std::unique_ptr<NavFn> planner_;

  // TF buffer
  // 【中文注释】TF2 缓冲，用于坐标变换（当前实现中主要在基类接口中传入）
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Clock
  // 【中文注释】ROS 时钟，供日志和时间戳使用
  rclcpp::Clock::SharedPtr clock_;

  // Logger
  // 【中文注释】日志器
  rclcpp::Logger logger_{rclcpp::get_logger("NavfnPlanner")};

  // Global Costmap
  // 【中文注释】全局代价地图原始指针（从 costmap_ros 获取），提供栅格代价和坐标转换
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  // 【中文注释】costmap 的全局坐标系名称 & 插件实例名
  std::string global_frame_, name_;

  // Whether or not the planner should be allowed to plan through unknown space
  // 【中文注释】allow_unknown：是否允许穿越未知区域(255)；
  // use_final_approach_orientation：终点朝向是否用"接近方向"而非目标点朝向
  bool allow_unknown_, use_final_approach_orientation_;

  // If the goal is obstructed, the tolerance specifies how many meters the planner
  // can relax the constraint in x and y before failing
  // 【中文注释】tolerance：目标不可达时允许在 x/y 方向放松多少米去寻找最近可达终点
  double tolerance_;

  // Whether to use the astar planner or default dijkstras
  // 【中文注释】use_astar：true 用 A*（快，带欧氏启发式），false 用 Dijkstra（完整势场）
  bool use_astar_;

  // parent node weak ptr
  // 【中文注释】父生命周期节点的弱引用（避免循环持有）
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // Dynamic parameters handler
  // 【中文注释】动态参数回调句柄：运行期修改参数时通过此回调安全更新
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  // 【中文注释】动态参数变化回调：仅在 inactive 状态接受参数修改，
  // 校验成功后更新 allow_unknown_/use_astar_/tolerance_ 等成员
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav2_navfn_planner

#endif  // NAV2_NAVFN_PLANNER__NAVFN_PLANNER_HPP_
