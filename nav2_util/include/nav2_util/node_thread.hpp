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

#ifndef NAV2_UTIL__NODE_THREAD_HPP_
#define NAV2_UTIL__NODE_THREAD_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace nav2_util
{
/**
 * @class nav2_util::NodeThread
 * @brief A background thread to process node/executor callbacks
 * RAII 风格的后台线程封装：构造时启动线程 spin 节点/执行器，
 * 析构时自动 cancel 执行器并 join 线程，避免线程生命周期管理问题。
 */
class NodeThread
{
public:
  /**
   * @brief A background thread to process node callbacks constructor
   * @param node_base Interface to Node to spin in thread
   */
  // 用节点的底层接口构造：为节点创建专属执行器并在后台线程中 spin
  explicit NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base);

  /**
   * @brief A background thread to process executor's callbacks constructor
   * @param executor Interface to executor to spin in thread
   */
  // 直接 spin 外部已有的单线程执行器
  explicit NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor);

  /**
   * @brief A background thread to process node callbacks constructor
   * @param node Node pointer to spin in thread
   */
  // 模板构造：接受任意节点类型（Node/LifecycleNode 的 SharedPtr），
  // 委托给 node_base 版本构造函数
  template<typename NodeT>
  explicit NodeThread(NodeT node)
  : NodeThread(node->get_node_base_interface())
  {}

  /**
   * @brief A destructor
   */
  // 析构时取消执行器并 join 后台线程
  ~NodeThread();

protected:
  // 被 spin 的节点（仅第一种构造方式使用）
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  // 后台线程（unique_ptr：本类独占所有权，析构时 join）
  std::unique_ptr<std::thread> thread_;
  // 执行器：自己创建或由外部传入
  rclcpp::Executor::SharedPtr executor_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__NODE_THREAD_HPP_
