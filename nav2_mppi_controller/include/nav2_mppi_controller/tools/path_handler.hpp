// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Dexory
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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_

#include <vector>
#include <utility>
#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

using PathIterator = std::vector<geometry_msgs::msg::PoseStamped>::iterator;  // 路径位姿迭代器
using PathRange = std::pair<PathIterator, PathIterator>;  // 路径区间 [first, second)

/**
 * @class mppi::PathHandler
 * @brief Manager of incoming reference paths for transformation and processing
 * （路径处理器：管理局部路径规划所需的参考路径。核心职责：
 *  - setPath() 接收全局规划路径；
 *  - transformPath() 每周期裁剪出机器人附近一段并变换到代价地图坐标系；
 *  - 支持在路径首次方向反转（掉头）处截断，避免控制器提前抄近路。）
 */

class PathHandler
{
public:
  /**
    * @brief Constructor for mppi::PathHandler
    */
  PathHandler() = default;

  /**
    * @brief Destructor for mppi::PathHandler
    */
  ~PathHandler() = default;

  /**
    * @brief Initialize path handler on bringup
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param tf TF buffer for transformations
    * @param dynamic_parameter_handler Parameter handler object
    */
  // 初始化：保存依赖对象并读取路径处理参数（详见 path_handler.cpp）
  void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>,
    std::shared_ptr<tf2_ros::Buffer>, ParametersHandler *);

  /**
    * @brief Set new reference path
    * @param Plan Path to use
    */
  void setPath(const nav_msgs::msg::Path & plan);  // 接收新的全局参考路径

  /**
    * @brief Get reference path
    * @return Path
    */
  nav_msgs::msg::Path & getPath();  // 获取当前参考路径（可变引用）

  /**
   * @brief transform global plan to local applying constraints,
   * then prune global plan
   * @param robot_pose Pose of robot
   * @return global plan in local frame
   */
  // 主入口：返回代价地图坐标系下的局部参考路径（内部完成最近点搜索、
  // 弧长裁剪、坐标变换、越界截断与掉头推进）
  nav_msgs::msg::Path transformPath(const geometry_msgs::msg::PoseStamped & robot_pose);

protected:
  /**
    * @brief Transform a pose to another frame
    * @param frame Frame to transform to
    * @param in_pose Input pose
    * @param out_pose Output pose
    * @return Bool if successful
    */
  // 带超时的 TF 变换封装，失败返回 false 并记录日志
  bool transformPose(
    const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  /**
   * @brief Get largest dimension of costmap (radially)
   * @return Max distance from center of costmap to edge
   */
  // 最近点搜索距离的默认值：代价地图较长边像素数 × 分辨率 × 0.5
  double getMaxCostmapDist();

  /**
    * @brief Transform a pose to the global reference frame
    * @param pose Current pose
    * @return output poose in global reference frame
    */
  // 将机器人位姿变换到全局路径坐标系（供最近点搜索使用）
  geometry_msgs::msg::PoseStamped
  transformToGlobalPlanFrame(const geometry_msgs::msg::PoseStamped & pose);

  /**
    * @brief Get global plan within window of the local costmap size
    * @param global_pose Robot pose
    * @return plan transformed in the costmap frame and iterator to the first pose of the global
    * plan (for pruning)
    */
  // 裁剪核心：返回 (代价地图坐标系局部路径, 机器人最近点迭代器)
  std::pair<nav_msgs::msg::Path, PathIterator> getGlobalPlanConsideringBoundsInCostmapFrame(
    const geometry_msgs::msg::PoseStamped & global_pose);

  /**
    * @brief Prune a path to only interesting portions
    * @param plan Plan to prune
    * @param end Final path iterator
    */
  // 删除 [begin, end) 内位姿：丢弃机器人已走过的路径前缀
  void prunePlan(nav_msgs::msg::Path & plan, const PathIterator end);

  /**
    * @brief Check if the robot pose is within the set inversion tolerances
    * @param robot_pose Robot's current pose to check
    * @return bool If the robot pose is within the set inversion tolerances
    */
  // 判断机器人是否已到达当前掉头点容差范围内（位置 + 角度）
  bool isWithinInversionTolerances(const geometry_msgs::msg::PoseStamped & robot_pose);

  std::string name_;                        // 插件名（参数命名空间）
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;  // 代价地图
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;  // TF 变换缓冲
  ParametersHandler * parameters_handler_;   // 参数处理器

  nav_msgs::msg::Path global_plan_;          // 原始全局路径（含掉头点之后部分）
  nav_msgs::msg::Path global_plan_up_to_inversion_;  // 截断到首个掉头点的路径（用于跟踪）
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};

  double max_robot_pose_search_dist_{0};  // 最近点搜索的路径弧长上限
  double prune_distance_{0};              // 每周期保留的局部路径长度（米）
  double transform_tolerance_{0};         // TF 变换等待超时（秒）
  float inversion_xy_tolerance_{0.2};    // 掉头点位置容差（米）
  float inversion_yaw_tolerance{0.4};     // 掉头点角度容差（弧度）
  bool enforce_path_inversion_{false};    // 是否启用掉头截断
  unsigned int inversion_locale_{0u};     // 已跨越的掉头点在原路径中的索引（0=尚未推进）
};
}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_
