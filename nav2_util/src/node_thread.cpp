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
// 【文件概况】NodeThread 的实现
// ----------------------------------------------------------------------------
// 概述: 见 node_thread.hpp。本文件实现:
//   1. NodeThread(node_base): 创建专属单线程执行器 + 后台线程
//      （线程内: 注册节点 → spin 阻塞 → cancel 后注销节点）
//   2. NodeThread(executor): 复用外部执行器，仅开线程 spin 它
//   3. 析构: executor_->cancel() 使 spin() 返回，再 join() 线程
// ============================================================================

#include <memory>

#include "nav2_util/node_thread.hpp"

namespace nav2_util
{

NodeThread::NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
: node_(node_base)
{
  // 为该节点创建一个专属单线程执行器，并在独立线程中 spin：
  // 线程启动后依次完成：注册节点 -> 持续处理回调 -> 被取消后退注册
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  thread_ = std::make_unique<std::thread>(
    [&]()
    {
      executor_->add_node(node_);
      executor_->spin();  // 阻塞直到 executor_->cancel() 被调用
      executor_->remove_node(node_);
    });
}

NodeThread::NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor)
: executor_(executor)
{
  // 另一个构造重载：复用外部传入的执行器，只负责开线程 spin 它
  thread_ = std::make_unique<std::thread>([&]() {executor_->spin();});
}

NodeThread::~NodeThread()
{
  // 析构时先取消执行器（让 spin() 退出），再 join 线程保证安全回收
  executor_->cancel();
  thread_->join();
}

}  // namespace nav2_util
