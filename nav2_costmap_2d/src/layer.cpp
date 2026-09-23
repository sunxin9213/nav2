/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// ============================================================================
// 中文注释：Layer 插件基类实现文件
//
// 【文件功能】
// 本文件实现 Nav2 代价地图分层体系中所有 Layer 插件的公共基类 Layer。
// 它定义了每个层的公共骨架：
//   - initialize()：由 LayeredCostmap 在加载插件后调用，注入运行环境
//     （父容器指针、层名、TF 缓冲、生命周期节点弱引用、回调组），
//     然后调用子类的 onInitialize() 完成各层专属初始化；
//   - updateBounds()/updateCosts()：两阶段更新接口（本文件无默认实现，
//     由各子类覆写）；
//   - activate()/deactivate()/reset()：生命周期与复位接口；
//   - declareParameter() 等参数工具：自动为参数加上 "<层名>." 前缀，
//     实现 ROS2 层私有参数（namespace 化参数）的声明。
//
// 【在 Nav2 中的位置】
// 继承链：Layer → CostmapLayer → ObstacleLayer/VoxelLayer/StaticLayer...
//          Layer → InflationLayer、Layer → CostmapFilter → 各过滤器。
// Costmap2DROS 通过 pluginlib 创建 Layer 子类实例并调用 initialize()。
//
// 【关键设计】
// node_ 以 weak_ptr 持有生命周期节点：避免层延长节点寿命，
// 使用时 lock() 提升，节点已销毁则抛异常。
// 参数名全名 = 层名 + "." + 参数名，例如 "obstacle_layer.enabled"，
// 这使得同一节点上多个层可以使用同名参数而互不冲突。
// ============================================================================

#include "nav2_costmap_2d/layer.hpp"

#include <string>
#include <vector>
#include "nav2_util/node_utils.hpp"

namespace nav2_costmap_2d
{

// 构造：各指针/状态初始化为空/未启用
Layer::Layer()
: layered_costmap_(nullptr),   // 所属的分层地图容器（稍后由 initialize 注入）
  name_(),                     // 层名（如 "obstacle_layer"）
  tf_(nullptr),                // TF2 缓冲指针（坐标变换用）
  current_(false),             // 是否有最新数据
  enabled_(false)              // 是否启用
{}

// ============================================================================
// initialize()：插件加载后的统一初始化入口（非虚函数，不可覆写）
// 由 LayeredCostmap::registerLayer 调用，注入运行环境后回调 onInitialize()
// ============================================================================

void
Layer::initialize(
  LayeredCostmap * parent,
  std::string name,
  tf2_ros::Buffer * tf,
  const nav2_util::LifecycleNode::WeakPtr & node,
  rclcpp::CallbackGroup::SharedPtr callback_group)
{
  layered_costmap_ = parent;    // 父容器：可访问主地图/足迹等
  name_ = name;                 // 层名（也是参数前缀）
  tf_ = tf;                     // TF 缓冲：层内坐标变换使用
  node_ = node;                 // 生命周期节点（弱引用）
  callback_group_ = callback_group;   // 专用回调组（订阅回调可并发）
  {
    // 从节点提取 logger 与 clock 供层内日志/时间使用
    auto node_shared_ptr = node_.lock();
    logger_ = node_shared_ptr->get_logger();
    clock_ = node_shared_ptr->get_clock();
  }

  // 回调子类专属初始化（声明参数、建订阅、建缓存等）
  onInitialize();
}

// getFootprint()：获取机器人足迹多边形（来自父容器 LayeredCostmap）
const std::vector<geometry_msgs::msg::Point> &
Layer::getFootprint() const
{
  return layered_costmap_->getFootprint();
}

// ============================================================================
// declareParameter()（带默认值版本）：声明层私有参数
// 自动加层名前缀（如 "inflation_layer.inflation_radius"），
// 并登记到 local_params_ 供 Costmap2DROS 统一校验未知参数
// ============================================================================

void
Layer::declareParameter(
  const std::string & param_name,
  const rclcpp::ParameterValue & value)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  local_params_.insert(param_name);   // 记录本层声明的参数
  // 若未声明过才声明（避免重复声明抛异常）
  nav2_util::declare_parameter_if_not_declared(
    node, getFullName(param_name), value);
}

// ============================================================================
// declareParameter()（仅类型版本）：声明参数但不设默认值（必填参数）
// ============================================================================

void
Layer::declareParameter(
  const std::string & param_name,
  const rclcpp::ParameterType & param_type)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  local_params_.insert(param_name);
  nav2_util::declare_parameter_if_not_declared(
    node, getFullName(param_name), param_type);
}

// hasParameter()：查询层私有参数是否存在（同样带前缀）
bool
Layer::hasParameter(const std::string & param_name)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  return node->has_parameter(getFullName(param_name));
}

// getFullName()：参数全名 = 层名.参数名（层私有参数命名约定）
std::string
Layer::getFullName(const std::string & param_name)
{
  return std::string(name_ + "." + param_name);
}

}  // end namespace nav2_costmap_2d
