/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: David V. Lu!!
 *********************************************************************/
#ifndef NAV2_COSTMAP_2D__LAYER_HPP_
#define NAV2_COSTMAP_2D__LAYER_HPP_
// ============================================================================
// 中文注释：Layer 插件基类头文件（实现见 layer.cpp）
//
// 【类功能】
// 所有代价地图层的抽象基类，定义了分层体系的公共接口：
//   - initialize()：由 LayeredCostmap 注入运行环境后回调 onInitialize()；
//   - updateBounds()：更新第一阶段——各层声明本轮影响的世界坐标边界；
//   - updateCosts()：更新第二阶段——各层把数据写入主地图（master_grid）
//     的指定栅格窗口内（min_i..max_i, min_j..max_j）；
//   - activate()/deactivate()：生命周期启停（订阅的建立/断开）；
//   - reset()/isClearable()：复位与"是否可被 clear costmap 服务清除"；
//   - onFootprintChanged()：足迹变化通知（内切半径等需重算）。
//
// 【两阶段更新机制】（LayeredCostmap 每个 map_update 周期执行）
//   1) updateBounds：所有层依次运行，通过 min/max 指针累积扩大
//      本轮更新窗口（世界坐标，米）；
//   2) updateCosts：窗口换算为栅格索引后，所有层依次把各自数据
//      合并进主地图。两阶段分离使层间可以互相感知更新范围
//      （如膨胀层需要把窗口外扩膨胀半径）。
//
// 【继承体系】
//   Layer → CostmapLayer → StaticLayer/ObstacleLayer/VoxelLayer
//   Layer → InflationLayer
//   Layer → CostmapFilter → KeepoutFilter/SpeedFilter/BinaryFilter
//   Layer → DenoiseLayer / RangeSensorLayer
// 所有子类通过 PLUGINLIB_EXPORT_CLASS 注册为 pluginlib 插件，
// 由 Costmap2DROS 按 YAML 配置动态加载。
// ============================================================================

#include <string>
#include <vector>
#include <unordered_set>

#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_costmap_2d
{
class LayeredCostmap;

/**
 * @class Layer
 * @brief Abstract class for layered costmap plugin implementations
 */
class Layer
{
public:
  /**
   * @brief A constructor
   */
  Layer();
  /**
   * @brief A destructor
   */
  virtual ~Layer() {}

  /**
   * @brief Initialization process of layer on startup
   */
  void initialize(
    LayeredCostmap * parent,
    std::string name,
    tf2_ros::Buffer * tf,
    const nav2_util::LifecycleNode::WeakPtr & node,
    rclcpp::CallbackGroup::SharedPtr callback_group);
  /** @brief Stop publishers. */
  virtual void deactivate() {}
  /** @brief Restart publishers if they've been stopped. */
  virtual void activate() {}
  /**
   * @brief Reset this costmap
   */
  virtual void reset() = 0;
  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable() = 0;

  /**
   * @brief This is called by the LayeredCostmap to poll this plugin as to how
   *        much of the costmap it needs to update. Each layer can increase
   *        the size of this bounds.
   *
   * For more details, see "Layered Costmaps for Context-Sensitive Navigation",
   * by Lu et. Al, IROS 2014.
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) = 0;

  /**
   * @brief Actually update the underlying costmap, only within the bounds
   *        calculated during UpdateBounds().
   */
  virtual void updateCosts(
    Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) = 0;

  /** @brief Implement this to make this layer match the size of the parent costmap. */
  virtual void matchSize() {}

  /** @brief LayeredCostmap calls this whenever the footprint there
   * changes (via LayeredCostmap::setFootprint()).  Override to be
   * notified of changes to the robot's footprint. */
  virtual void onFootprintChanged() {}
  /** @brief Get the name of the costmap layer */
  std::string getName() const
  {
    return name_;
  }

  /**
   * @brief Check to make sure all the data in the layer is up to date.
   *        If the layer is not up to date, then it may be unsafe to
   *        plan using the data from this layer, and the planner may
   *        need to know.
   *
   *        A layer's current state should be managed by the protected
   *        variable current_.
   * @return Whether the data in the layer is up to date.
   */
  bool isCurrent() const
  {
    return current_;
  }

  /**@brief Gets whether the layer is enabled. */
  bool isEnabled() const
  {
    return enabled_;
  }

  /** @brief Convenience function for layered_costmap_->getFootprint(). */
  const std::vector<geometry_msgs::msg::Point> & getFootprint() const;

  /** @brief Convenience functions for declaring ROS parameters */
  void declareParameter(
    const std::string & param_name,
    const rclcpp::ParameterValue & value);
  /** @brief Convenience functions for declaring ROS parameters */
  void declareParameter(
    const std::string & param_name,
    const rclcpp::ParameterType & param_type);
  /** @brief Convenience functions for declaring ROS parameters */
  bool hasParameter(const std::string & param_name);
  /** @brief Convenience functions for declaring ROS parameters */
  std::string getFullName(const std::string & param_name);

protected:
  // ---- 公共运行环境成员（由 initialize() 注入）----
  LayeredCostmap * layered_costmap_;   // 父容器：可访问主地图/足迹/插件列表
  std::string name_;                   // 层名（同时是 ROS 参数前缀）
  tf2_ros::Buffer * tf_;               // TF2 缓冲：层内坐标变换
  rclcpp::CallbackGroup::SharedPtr callback_group_;   // 专用回调组（订阅回调并发）
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;     // 生命周期节点（弱引用，不延长寿命）
  rclcpp::Clock::SharedPtr clock_;     // 节点时钟
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_costmap_2d")};  // 日志器

  /** @brief This is called at the end of initialize().  Override to
   * implement subclass-specific initialization.
   *
   * tf_, name_, and layered_costmap_ will all be set already when this is called.
   */
  virtual void onInitialize() {}

  bool current_;
  // Currently this var is managed by subclasses.
  // TODO(bpwilcox): make this managed by this class and/or container class.
  bool enabled_;

  // Names of the parameters declared on the ROS node
  std::unordered_set<std::string> local_params_;

private:
  std::vector<geometry_msgs::msg::Point> footprint_spec_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__LAYER_HPP_
