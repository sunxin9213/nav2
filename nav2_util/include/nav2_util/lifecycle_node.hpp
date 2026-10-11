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
// 【文件概况】Nav2 统一生命周期节点基类
// ----------------------------------------------------------------------------
// 概述: 继承 rclcpp_lifecycle::LifecycleNode，在标准生命周期节点之上
//       补齐 nav2 需要的三块能力: bond 心跳、兜底清理、参数声明辅助。
// 主要内容:
//   1. add_parameter() ×3: 带描述/范围约束（浮点/整数）的参数声明
//   2. createBond()/destroyBond(): 与 lifecycle_manager 的存活心跳
//   3. on_rcl_preshutdown()/runCleanups(): rcl 关闭或析构时自动
//      deactivate+cleanup，防止节点未正常下线就退出
//   4. shared_from_this()/on_error(): 常用接口补全
// 使用场景: nav2 中所有 lifecycle 服务器节点（costmap/planner/controller/
//           smoother/behaviors/waypoint_follower 等）的公共基类。
// ============================================================================

#ifndef NAV2_UTIL__LIFECYCLE_NODE_HPP_
#define NAV2_UTIL__LIFECYCLE_NODE_HPP_

#include <memory>
#include <string>
#include <thread>

#include "nav2_util/node_thread.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "bondcpp/bond.hpp"
#include "bond/msg/constants.hpp"

namespace nav2_util
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class nav2_util::LifecycleNode
 * @brief A lifecycle node wrapper to enable common Nav2 needs such as manipulating parameters
 */
class LifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief A lifecycle node constructor
   * @param node_name Name for the node
   * @param namespace Namespace for the node, if any
   * @param options Node options
   */
  // 创建 Nav2 统一生命周期节点；options 可携带参数覆盖、自动声明和组件组合配置。
  LifecycleNode(
    const std::string & node_name,
    const std::string & ns = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~LifecycleNode();

  // 浮点参数范围约束（供 add_parameter 使用）
  typedef struct
  {
    double from_value;  // 允许的最小值
    double to_value;    // 允许的最大值
    double step;        // 合法步长
  } floating_point_range;

  // 整型参数范围约束（含义同上）
  typedef struct
  {
    int from_value;  // 允许的最小值
    int to_value;    // 允许的最大值
    int step;        // 合法步长
  } integer_range;

  /**
   * @brief Declare a parameter that has no integer or floating point range constraints
   * @param node_name Name of parameter
   * @param default_value Default node value to add
   * @param description Node description
   * @param additional_constraints Any additional constraints on the parameters to list
   * @param read_only Whether this param should be considered read only
   */
  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false)
  {
    // ParameterDescriptor 将描述、只读属性和约束暴露给 ROS 2 参数系统，便于运行时校验和工具展示。
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  /**
   * @brief Declare a parameter that has a floating point range constraint
   * @param node_name Name of parameter
   * @param default_value Default node value to add
   * @param fp_range floating point range
   * @param description Node description
   * @param additional_constraints Any additional constraints on the parameters to list
   * @param read_only Whether this param should be considered read only
   */
  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const floating_point_range fp_range,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false)
  {
    // floating_point_range 的 from/to/step 分别表示允许范围和步长，单位由具体参数定义。
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = fp_range.from_value;
    descriptor.floating_point_range[0].to_value = fp_range.to_value;
    descriptor.floating_point_range[0].step = fp_range.step;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  /**
   * @brief Declare a parameter that has an integer range constraint
   * @param node_name Name of parameter
   * @param default_value Default node value to add
   * @param integer_range Integer range
   * @param description Node description
   * @param additional_constraints Any additional constraints on the parameters to list
   * @param read_only Whether this param should be considered read only
   */
  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const integer_range int_range,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false)
  {
    // integer_range 为整型参数提供同样的 ROS 2 参数约束元数据。
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = int_range.from_value;
    descriptor.integer_range[0].to_value = int_range.to_value;
    descriptor.integer_range[0].step = int_range.step;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  /**
   * @brief Get a shared pointer of this
   */
  // 取自身的 LifecycleNode 共享指针：基类返回类型是 LifecycleNode 基类指针，
  // 这里通过 static_pointer_cast 协变转换为本类型
  std::shared_ptr<nav2_util::LifecycleNode> shared_from_this()
  {
    return std::static_pointer_cast<nav2_util::LifecycleNode>(
      rclcpp_lifecycle::LifecycleNode::shared_from_this());
  }

  /**
   * @brief Abstracted on_error state transition callback, since unimplemented as of 2020
   * in the managed ROS2 node state machine
   * @param state State prior to error transition
   * @return Return type for success or failed transition to error state
   */
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_FATAL(
      get_logger(),
      "Lifecycle node %s does not have error state implemented", get_name());
    return nav2_util::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Perform preshutdown activities before our Context is shutdown.
   * Note that this is related to our Context's shutdown sequence, not the
   * lifecycle node state machine.
   */
  // rcl Context 即将关闭时的钩子: 先做生命周期清理再断开 bond
  virtual void on_rcl_preshutdown();

  /**
   * @brief Create bond connection to lifecycle manager
   */
  // 创建与 Lifecycle Manager 的 bond，用于监测节点是否仍存活。
  void createBond();

  /**
   * @brief Destroy bond connection to lifecycle manager
   */
  // 销毁 bond，结束与 Lifecycle Manager 的存活连接。
  void destroyBond();

protected:
  /**
   * @brief Print notifications for lifecycle node
   */
  // 打印"节点已启动、等待外部触发迁移"的提示信息
  void printLifecycleNodeNotification();

  /**
   * Register our preshutdown callback for this Node's rcl Context.
   * The callback fires before this Node's Context is shutdown.
   * Note this is not directly related to the lifecycle state machine.
   */
  // 把 on_rcl_preshutdown 注册为 Context 的预关闭回调（构造函数中调用）
  void register_rcl_preshutdown_callback();
  // 预关闭回调句柄: 必须长期持有，否则回调会被立即注销
  std::unique_ptr<rclcpp::PreShutdownCallbackHandle> rcl_preshutdown_cb_handle_{nullptr};

  /**
   * Run some common cleanup steps shared between rcl preshutdown and destruction.
   */
  // 兜底清理: 若仍为 ACTIVE 先 deactivate，再 cleanup（析构/退出时调用）
  void runCleanups();

  // Connection to tell that server is still up
  // 与 lifecycle_manager 的 bond 心跳连接（activate 后创建）
  std::unique_ptr<bond::Bond> bond_{nullptr};
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_NODE_HPP_
