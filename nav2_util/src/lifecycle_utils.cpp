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

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_util/lifecycle_service_client.hpp"

using std::string;
using lifecycle_msgs::msg::Transition;

namespace nav2_util
{

// RETRY 宏：反复执行表达式 fn，捕获 runtime_error 并重试，
// 超过 retries 次后把异常重新抛出（用于对抗服务调用偶发挂死的问题）
#define RETRY(fn, retries) \
  { \
    int count = 0; \
    while (true) { \
      try { \
        fn; \
        break; \
      } catch (std::runtime_error & e) { \
        ++count; \
        if (count > (retries)) { \
          throw e;} \
      } \
    } \
  }

// 启动单个生命周期节点：依次触发 configure -> activate 状态迁移
static void startupLifecycleNode(
  const std::string & node_name,
  const std::chrono::seconds service_call_timeout,
  const int retries)
{
  // 为目标节点创建一个生命周期服务客户端（内部会生成临时内部节点）
  LifecycleServiceClient sc(node_name);

  // Despite waiting for the service to be available and using reliable transport
  // service calls still frequently hang. To get reliable startup it's necessary
  // to timeout the service call and retry it when that happens.
  // 实践中即使服务已可用，调用仍可能挂起，因此用超时 + 重试保证启动可靠
  RETRY(
    sc.change_state(Transition::TRANSITION_CONFIGURE, service_call_timeout),
    retries);
  RETRY(
    sc.change_state(Transition::TRANSITION_ACTIVATE, service_call_timeout),
    retries);
}

// 按顺序启动一批生命周期节点（每个节点依次 configure + activate）
void startup_lifecycle_nodes(
  const std::vector<std::string> & node_names,
  const std::chrono::seconds service_call_timeout,
  const int retries)
{
  for (const auto & node_name : node_names) {
    startupLifecycleNode(node_name, service_call_timeout, retries);
  }
}

// 复位单个生命周期节点：依次触发 deactivate -> cleanup
static void resetLifecycleNode(
  const std::string & node_name,
  const std::chrono::seconds service_call_timeout,
  const int retries)
{
  LifecycleServiceClient sc(node_name);

  // Despite waiting for the service to be available and using reliable transport
  // service calls still frequently hang. To get reliable reset it's necessary
  // to timeout the service call and retry it when that happens.
  RETRY(
    sc.change_state(Transition::TRANSITION_DEACTIVATE, service_call_timeout),
    retries);
  RETRY(
    sc.change_state(Transition::TRANSITION_CLEANUP, service_call_timeout),
    retries);
}

// 按顺序复位一批生命周期节点
void reset_lifecycle_nodes(
  const std::vector<std::string> & node_names,
  const std::chrono::seconds service_call_timeout,
  const int retries)
{
  for (const auto & node_name : node_names) {
    resetLifecycleNode(node_name, service_call_timeout, retries);
  }
}

}  // namespace nav2_util
