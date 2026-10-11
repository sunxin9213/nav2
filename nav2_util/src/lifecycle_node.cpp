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
// 【文件概况】nav2_util::LifecycleNode 基类的实现
// ----------------------------------------------------------------------------
// 概述: 见 lifecycle_node.hpp。本文件实现:
//   1. 构造/析构: 声明"bond 不超时"参数、打印启动提示、注册预关闭回调;
//      析构时先 runCleanups() 再注销预关闭回调句柄
//   2. createBond()/destroyBond(): 在 "bond" 话题上建立/断开心跳
//      （心跳周期 100ms、超时 4s，约连续丢 40 次才判失联）
//   3. runCleanups(): ACTIVE → deactivate → cleanup 的兜底清理
//   4. on_rcl_preshutdown()/register_rcl_preshutdown_callback():
//      保证 rcl 关闭前先完成节点清理（防止未下线就退出）
//   5. printLifecycleNodeNotification(): 启动提示信息
// ============================================================================

#include "nav2_util/lifecycle_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

namespace nav2_util
{

LifecycleNode::LifecycleNode(
  const std::string & node_name,
  const std::string & ns,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, ns, options)
{
  // server side never times out from lifecycle manager
  // 声明并设置 "disable_heartbeat_timeout" 参数为 true：
  // 服务端（本节点）永不因心跳超时断开 bond，超时判断完全交由
  // Lifecycle Manager 侧执行（单向存活监测）
  this->declare_parameter(bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true);
  this->set_parameter(
    rclcpp::Parameter(
      bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true));

  // 打印生命周期节点启动提示（告知需要外部触发状态迁移才会激活）
  printLifecycleNodeNotification();

  // 注册 rcl 关闭前的回调，确保进程退出时也能执行清理
  register_rcl_preshutdown_callback();
}

LifecycleNode::~LifecycleNode()
{
  RCLCPP_INFO(get_logger(), "Destroying");

  // 析构时先做兜底清理（deactivate/cleanup），保证资源释放顺序正确
  runCleanups();

  // 从 context 中注销预关闭回调并释放句柄，防止悬空回调
  if (rcl_preshutdown_cb_handle_) {
    rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();
    context->remove_pre_shutdown_callback(*(rcl_preshutdown_cb_handle_.get()));
    rcl_preshutdown_cb_handle_.reset();
  }
}

void LifecycleNode::createBond()
{
  RCLCPP_INFO(get_logger(), "Creating bond (%s) to lifecycle manager.", this->get_name());

  // 在 "bond" 话题上创建 bond 对象，节点名作为 bond 的 id，
  // shared_from_this() 使 bond 持有节点的共享指针，保证节点生命周期覆盖 bond
  bond_ = std::make_unique<bond::Bond>(
    std::string("bond"),
    this->get_name(),
    shared_from_this());

  // 心跳周期 100ms、超时 4s：连续约 40 次心跳丢失才判定节点失联
  bond_->setHeartbeatPeriod(0.10);
  bond_->setHeartbeatTimeout(4.0);
  bond_->start();
}

void LifecycleNode::runCleanups()
{
  /*
   * In case this lifecycle node wasn't properly shut down, do it here.
   * We will give the user some ability to clean up properly here, but it's
   * best effort; i.e. we aren't trying to account for all possible states.
   */
  // 若节点仍处于 ACTIVE 态，先触发 deactivate 回调（停止传感器/执行器等）
  if (get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    this->deactivate();
  }

  // deactivate 后处于 INACTIVE 态，再触发 cleanup 释放状态资源
  if (get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    this->cleanup();
  }
}

void LifecycleNode::on_rcl_preshutdown()
{
  RCLCPP_INFO(
    get_logger(), "Running Nav2 LifecycleNode rcl preshutdown (%s)",
    this->get_name());

  // rcl 即将关闭：执行生命周期清理并断开 bond 连接
  runCleanups();

  destroyBond();
}

void LifecycleNode::register_rcl_preshutdown_callback()
{
  // 取节点所属 context，把成员函数 on_rcl_preshutdown 注册为
  // 预关闭回调；返回的句柄需长期持有，否则回调会被立即注销
  rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();

  rcl_preshutdown_cb_handle_ = std::make_unique<rclcpp::PreShutdownCallbackHandle>(
    context->add_pre_shutdown_callback(
      std::bind(&LifecycleNode::on_rcl_preshutdown, this))
  );
}

void LifecycleNode::destroyBond()
{
  RCLCPP_INFO(get_logger(), "Destroying bond (%s) to lifecycle manager.", this->get_name());

  // unique_ptr 的 reset() 会触发 bond 析构，停止心跳并断开连接
  if (bond_) {
    bond_.reset();
  }
}

void LifecycleNode::printLifecycleNodeNotification()
{
  // 提示用户：节点已启动但处于 UNCONFIGURED 状态，
  // 需要外部（Lifecycle Manager）依次触发 configure/activate 迁移
  RCLCPP_INFO(
    get_logger(),
    "\n\t%s lifecycle node launched. \n"
    "\tWaiting on external lifecycle transitions to activate\n"
    "\tSee https://design.ros2.org/articles/node_lifecycle.html for more information.", get_name());
}

}  // namespace nav2_util
