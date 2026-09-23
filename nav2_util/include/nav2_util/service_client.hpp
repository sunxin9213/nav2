// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_UTIL__SERVICE_CLIENT_HPP_
#define NAV2_UTIL__SERVICE_CLIENT_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace nav2_util
{

/**
 * @class nav2_util::ServiceClient
 * @brief A simple wrapper on ROS2 services for invoke() and block-style calling
 * ROS2 服务客户端模板封装：把"等待服务出现 -> 异步发送请求 -> 阻塞等待结果"
 * 的完整流程封装成同步的 invoke() 调用。
 *
 * 核心技巧：客户端使用独立的 MutuallyExclusive 回调组 + 专用 SingleThreadedExecutor，
 * 这样即使外层正在 spin 别的执行器（或处于回调中），也能安全地同步等待服务响应，
 * 而不会造成死锁（响应不会依赖外层执行器来分发）。
 */
template<class ServiceT>
class ServiceClient
{
public:
  /**
  * @brief A constructor
  * @param service_name name of the service to call
  * @param provided_node Node to create the service client off of
  */
  explicit ServiceClient(
    const std::string & service_name,
    const rclcpp::Node::SharedPtr & provided_node)
  : service_name_(service_name), node_(provided_node)
  {
    // 创建独立回调组：第二个参数 false 表示不自动加入节点的默认执行器
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    // 把回调组加入本类专属的单线程执行器，专门负责处理该服务的响应
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    // 在独立回调组上创建服务客户端（使用服务专用 QoS 配置）
    client_ = node_->create_client<ServiceT>(
      service_name,
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      callback_group_);
  }

  // 从服务类型中提取 Request/Response 消息类型（typedef 提取技巧）
  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  /**
  * @brief Invoke the service and block until completed or timed out
  * @param request The request object to call the service using
  * @param timeout Maximum timeout to wait for, default infinite
  * @return Response A pointer to the service response from the request
  */
  typename ResponseType::SharedPtr invoke(
    typename RequestType::SharedPtr & request,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1))
  {
    // 循环等待服务端上线：每次最多等 1s，期间不断打印提示；
    // 若 rclcpp 已关闭（如 Ctrl+C）则抛异常退出
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
                service_name_ + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(
        node_->get_logger(), "%s service client: waiting for service to appear...",
        service_name_.c_str());
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "%s service client: send async request",
      service_name_.c_str());
    // 异步发送请求，得到 future
    auto future_result = client_->async_send_request(request);

    // 在专用执行器上自旋直到 future 完成（timeout=-1 表示无限等待）
    if (callback_group_executor_.spin_until_future_complete(future_result, timeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      // Pending request must be manually cleaned up if execution is interrupted or timed out
      // 失败（超时/中断）时必须手动移除挂起请求，防止资源泄漏
      client_->remove_pending_request(future_result);
      throw std::runtime_error(service_name_ + " service client: async_send_request failed");
    }

    // 取出响应结果（SharedPtr）
    return future_result.get();
  }

  /**
  * @brief Invoke the service and block until completed
  * @param request The request object to call the service using
  * @param Response A pointer to the service response from the request
  * @return bool Whether it was successfully called
  */
  bool invoke(
    typename RequestType::SharedPtr & request,
    typename ResponseType::SharedPtr & response)
  {
    // 与上一个 invoke 相同的等待服务上线逻辑
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
                service_name_ + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(
        node_->get_logger(), "%s service client: waiting for service to appear...",
        service_name_.c_str());
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "%s service client: send async request",
      service_name_.c_str());
    auto future_result = client_->async_send_request(request);

    // 无限等待版本：结果通过出参 response 返回，函数返回是否成功
    if (callback_group_executor_.spin_until_future_complete(future_result) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      // Pending request must be manually cleaned up if execution is interrupted or timed out
      client_->remove_pending_request(future_result);
      return false;
    }

    response = future_result.get();
    // 注意：Response 消息首字段按惯例是 bool success，
    // get() 解引用 SharedPtr 隐式转换为 bool 即取该字段值
    return response.get();
  }

  /**
  * @brief Block until a service is available or timeout
  * @param timeout Maximum timeout to wait for, default infinite
  * @return bool true if service is available
  */
  // 阻塞等待服务可用（默认无限等待）
  bool wait_for_service(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max())
  {
    return client_->wait_for_service(timeout);
  }

  /**
  * @brief Gets the service name
  * @return string Service name
  */
  std::string getServiceName()
  {
    return service_name_;
  }

protected:
  // 服务名与宿主节点
  std::string service_name_;
  rclcpp::Node::SharedPtr node_;
  // 独立回调组及其专用单线程执行器（处理该客户端的响应回调）
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  // 服务客户端句柄
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__SERVICE_CLIENT_HPP_
