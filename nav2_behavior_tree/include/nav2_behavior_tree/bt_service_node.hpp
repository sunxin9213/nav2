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

// =============================================================================
// 【文件说明】bt_service_node.hpp
// 【所属模块】nav2_behavior_tree —— Nav2 的行为树核心模块
//
// 本文件定义了 BtServiceNode<ServiceT> 模板类——所有"服务型"行为树节点的
// 公共抽象基类。
//
// 【在 Nav2 导航系统中的位置和作用】
// ROS2 提供两种请求-响应式通信：Service（同步一问一答，如 ClearCostmap、
// 重新全局定位）和 Action（带反馈的长时间任务）。本类封装的是 Service：
//   1. 构造时创建 rclcpp::Client<ServiceT> 服务客户端并等待服务器上线；
//   2. tick() 中异步发送请求（async_send_request），在超时预算内分多个
//      周期等待响应（期间返回 RUNNING，不阻塞整棵树）；
//   3. 通过虚函数 on_tick（填充请求）/ on_completion（处理响应）/
//      on_wait_for_result（等待期间的钩子）把业务逻辑下放给子类。
// 典型子类：ClearCostmapService（清空代价地图）、
// ReinitializeGlobalLocalizationService（全局定位重初始化）等。
//
// 【BehaviorTree.CPP 核心概念】
// - 虽然语义上是"服务"，BT 分类上它仍继承 BT::ActionNodeBase（动作节点），
//   因为服务调用是"需要时间才能出结果的操作"。
// - tick() 返回：SUCCESS（服务返回）/ FAILURE（超时或用户决定不发送）/
//   RUNNING（仍在等待响应）。
// - halt()：复位"请求已发送"标志并把节点状态置回 IDLE，
//   下次 tick 会重新走完整流程。
// - providedPorts() 基础端口：service_name（服务名，默认值提示用户必须在
//   XML 中设置）、server_timeout（应答超时，毫秒）。
//
// 【XML 使用示例】（以子类 ClearCostmapService 为例，一行）
//   <ClearCostmapService service_name="local_costmap/clear_around_local_costmap"/>
// =============================================================================

#ifndef NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace nav2_behavior_tree
{

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief Abstract class representing a service based BT node
 *        （表示"基于 ROS2 Service 的行为树节点"的抽象基类）
 * @tparam ServiceT Type of service
 *        （模板参数：要调用的服务类型，如 nav2_msgs::srv::ClearCostmap）
 */
template<class ServiceT>
class BtServiceNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::BtServiceNode constructor
   *        （构造函数：创建服务客户端并等待服务器上线）
   * @param service_node_name BT node name
   *        （该 BT 节点在 XML 中的标签名）
   * @param conf BT node configuration
   *        （BT 节点配置，含黑板指针和端口映射）
   * @param service_name Optional service name this node creates a client for instead of from input port
   *        （可选：直接指定服务名，优先于从端口读取）
   */
  BtServiceNode(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf,
    const std::string & service_name = "")
  : BT::ActionNodeBase(service_node_name, conf), service_name_(service_name), service_node_name_(
      service_node_name)
  {
    // 从黑板取出共享的 ROS2 节点指针（由 BtActionServer 预先放入）。
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    // 创建本节点专属的互斥回调组（false = 不加入默认执行器），
    // 把服务响应回调与本节点私有执行器绑定，避免并发竞争。
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // Get the required items from the blackboard
    // 从黑板读取引擎级超时配置：
    // bt_loop_duration = 单次 tick 的时间预算；server_timeout = 应答超时
    // （XML 端口值优先）。
    auto bt_loop_duration =
      config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
    server_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
    // 构造阶段等待服务出现的超时时间。
    wait_for_service_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("wait_for_service_timeout");

    // timeout should be less than bt_loop_duration to be able to finish the current tick
    // 单次同步等待上限 = tick 预算的一半，防止单次 tick 阻塞过久。
    max_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(bt_loop_duration * 0.5);

    // 若构造函数未直接指定服务名，则尝试从 XML 端口 "service_name" 读取。
    getInput("service_name", service_name_);
    // 创建服务客户端：使用服务专用 QoS（ServicesQoS）与本节点回调组。
    service_client_ = node_->create_client<ServiceT>(
      service_name_,
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      callback_group_);

    // Make a request for the service without parameter
    // 构造一个默认的空请求对象，子类可在 on_tick 中填充字段。
    request_ = std::make_shared<typename ServiceT::Request>();

    RCLCPP_DEBUG(
      node_->get_logger(), "Waiting for \"%s\" service",
      service_name_.c_str());
    // 阻塞等待服务服务器上线；超时抛异常使节点构造失败
    // （避免运行时调用一个不存在的服务）。
    if (!service_client_->wait_for_service(wait_for_service_timeout_)) {
      RCLCPP_ERROR(
        node_->get_logger(), "\"%s\" service server not available after waiting for %.2fs",
        service_name_.c_str(), wait_for_service_timeout_.count() / 1000.0);
      throw std::runtime_error(
              std::string(
                "Service server %s not available",
                service_node_name.c_str()));
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "\"%s\" BtServiceNode initialized",
      service_node_name_.c_str());
  }

  // 禁用默认构造：必须显式提供节点名和配置。
  BtServiceNode() = delete;

  // 虚析构函数：支持通过基类指针安全释放子类。
  virtual ~BtServiceNode()
  {
  }

  /**
   * @brief Any subclass of BtServiceNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   *        （子类若声明自己的端口，须重写 providedPorts() 并调用本函数）
   * @param addition Additional ports to add to BT port list
   *        （子类追加的端口列表）
   * @return BT::PortsList Containing basic ports along with node-specific ports
   *        （基础端口 + 子类端口的合并结果）
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    // 基础端口：service_name 带默认值提示串（提醒用户在 XML 中必须设置
    // 服务名，否则 Groot 中显示该占位默认值）；server_timeout 单位毫秒。
    BT::PortsList basic = {
      BT::InputPort<std::string>("service_name", "please_set_service_name_in_BT_Node"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    // 合并子类附加端口。
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief Creates list of BT ports
   *        （生成端口列表；未被重写时仅含基础端口）
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  /**
   * @brief The main override required by a BT service
   *        （节点主函数：首次 tick 发送请求，后续 tick 检查响应是否到达）
   * @return BT::NodeStatus Status of tick execution
   *        （SUCCESS/FAILURE/RUNNING）
   */
  BT::NodeStatus tick() override
  {
    // 仅在"尚未发送请求"的首次 tick 执行发送逻辑。
    if (!request_sent_) {
      // reset the flag to send the request or not,
      // allowing the user the option to set it in on_tick
      // 重置"是否发送请求"标志，允许子类在 on_tick 中置 false 跳过发送。
      should_send_request_ = true;

      // 调用子类钩子：填充 request_ 字段或做前置检查。
      on_tick();

      // 子类决定不发送请求：本节点直接判定失败。
      if (!should_send_request_) {
        return BT::NodeStatus::FAILURE;
      }

      // 异步发送服务请求：立即返回 future（.share() 转为可拷贝的共享
      // future），记录发送时刻并置位 request_sent_，
      // 后续 tick 不再重复发送。
      future_result_ = service_client_->async_send_request(request_).share();
      sent_time_ = node_->now();
      request_sent_ = true;
    }
    // 走等待/检查响应流程（见 check_future）。
    return check_future();
  }

  /**
   * @brief The other (optional) override required by a BT service.
   *        （BT 中断钩子：复位发送标志并把节点状态置回 IDLE）
   */
  void halt() override
  {
    // 复位"已发送"标志：下次 tick 将重新构造并发送请求。
    request_sent_ = false;
    // 节点状态复位为 IDLE，下次进入 tick 走完整的"发送-等待"流程。
    setStatus(BT::NodeStatus::IDLE);
  }

  /**
   * @brief Function to perform some user-defined operation on tick
   * Fill in service request with information if necessary
   *        （tick 钩子：子类在此用 getInput() 读端口/黑板并填充请求字段）
   */
  virtual void on_tick()
  {
  }

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the service. Could put a value on the blackboard.
   *        （服务成功返回时的钩子：子类在此解析响应、写黑板）
   * @param response can be used to get the result of the service call in the BT Node.
   *        （服务响应对象，含服务器返回的数据）
   * @return BT::NodeStatus Returns SUCCESS by default, user may override to return another value
   *        （默认返回 SUCCESS）
   */
  virtual BT::NodeStatus on_completion(std::shared_ptr<typename ServiceT::Response>/*response*/)
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief Check the future and decide the status of BT
   *        （等待并检查服务响应：在超时预算内分段等待 future 完成）
   * @return BT::NodeStatus SUCCESS if future complete before timeout, FAILURE otherwise
   *        （SUCCESS = 收到响应；RUNNING = 仍在等待；FAILURE = 超时）
   */
  virtual BT::NodeStatus check_future()
  {
    // 计算已等待时长和剩余额度（相对 server_timeout_）。
    auto elapsed = (node_->now() - sent_time_).template to_chrono<std::chrono::milliseconds>();
    auto remaining = server_timeout_ - elapsed;

    if (remaining > std::chrono::milliseconds(0)) {
      // 单次等待时长取 min(剩余额度, max_timeout_)：
      // max_timeout_ 是 tick 预算的一半，防止单次 tick 阻塞过久。
      auto timeout = remaining > max_timeout_ ? max_timeout_ : remaining;

      // 阻塞至多 timeout 时间等待响应 future 完成。
      rclcpp::FutureReturnCode rc;
      rc = callback_group_executor_.spin_until_future_complete(future_result_, timeout);
      if (rc == rclcpp::FutureReturnCode::SUCCESS) {
        // 收到响应：复位发送标志，调用子类钩子处理响应并返回其状态。
        request_sent_ = false;
        BT::NodeStatus status = on_completion(future_result_.get());
        return status;
      }

      if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        // 本次等待超时但总额度未耗尽：调用等待钩子，
        // 然后返回 RUNNING，把控制权交还行为树引擎，下个周期继续等待。
        on_wait_for_result();
        elapsed = (node_->now() - sent_time_).template to_chrono<std::chrono::milliseconds>();
        if (elapsed < server_timeout_) {
          return BT::NodeStatus::RUNNING;
        }
      }
    }

    // 总超时已耗尽：记录警告，复位标志，节点判定失败。
    RCLCPP_WARN(
      node_->get_logger(),
      "Node timed out while executing service call to %s.", service_name_.c_str());
    request_sent_ = false;
    return BT::NodeStatus::FAILURE;
  }

  /**
   * @brief Function to perform some user-defined operation after a timeout waiting
   * for a result that hasn't been received yet
   *        （单次等待超时但整体未超时的钩子：子类可在此做周期性处理）
   */
  virtual void on_wait_for_result()
  {
  }

protected:
  /**
   * @brief Function to increment recovery count on blackboard if this node wraps a recovery
   *        （若本节点属于恢复行为：递增黑板上的 number_recoveries 计数器）
   */
  void increment_recovery_count()
  {
    // 读取当前计数（黑板无该键时保持 0）。
    int recovery_count = 0;
    config().blackboard->template get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    // 写回黑板，供 RecoveryNode / 监控端统计恢复次数。
    config().blackboard->template set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  // ---- 成员变量说明 ----
  // service_name_ = 要调用的服务名；service_node_name_ = BT 节点标签名。
  std::string service_name_, service_node_name_;
  // ROS2 服务客户端：负责请求/响应的底层通信。
  typename std::shared_ptr<rclcpp::Client<ServiceT>> service_client_;
  // 当前待发送/已发送的服务请求对象（子类填充字段）。
  std::shared_ptr<typename ServiceT::Request> request_;

  // 共享的 ROS2 节点指针（黑板 "node" 键）。
  rclcpp::Node::SharedPtr node_;
  // 本节点专属的互斥回调组。
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  // 泵取回调组回调的单线程执行器。
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // The timeout value while to use in the tick loop while waiting for
  // 等待服务应答的总超时（毫秒）。
  std::chrono::milliseconds server_timeout_;

  // 单次同步等待的上限（tick 预算的一半）。
  std::chrono::milliseconds max_timeout_;

  // 构造阶段等待服务上线的超时（毫秒）。
  std::chrono::milliseconds wait_for_service_timeout_;

  // 服务响应对应的共享 future（发送时创建）。
  std::shared_future<typename ServiceT::Response::SharedPtr> future_result_;
  // 标志：请求已发送、正在等待响应。
  bool request_sent_{false};
  // 请求发送时刻，用于计算等待耗时。
  rclcpp::Time sent_time_;

  // Can be set in on_tick or on_wait_for_result to indicate if a request should be sent.
  // 子类可在 on_tick / on_wait_for_result 中置 false，表示本周期不发送请求。
  bool should_send_request_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_
