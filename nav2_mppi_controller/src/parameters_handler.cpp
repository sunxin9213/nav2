// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

// 本文件实现 ParametersHandler（参数处理器）：
// MPPI 控制器的统一参数管理中心。它提供两层能力：
// 1) 参数读取封装 getParamGetter()：绑定命名空间前缀，首次读取时从节点
//    声明并获取参数值；对标记为 Auto 的参数，同时注册动态更新回调，
//    使运行期可通过 ros2 param set 实时调参（无需重启导航）。
// 2) ROS2 set_parameters 回调服务：动态参数变更到达时按
//    pre_callbacks_ -> 各参数回调 -> post_callbacks_ 的顺序分发。
ParametersHandler::ParametersHandler(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent)
{
  node_ = parent;                 // 保存生命周期节点弱引用
  auto node = node_.lock();
  node_name_ = node->get_name();  // 节点名，作为顶层参数命名空间
  logger_ = node->get_logger();
}

// 启动动态参数服务：向节点注册 on_set_parameters 回调
void ParametersHandler::start()
{
  auto node = node_.lock();
  // 注册 ROS2 参数变更回调；返回的句柄用于 cleanup 时注销（移除回调）
  on_set_param_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ParametersHandler::dynamicParamsCallback, this,
      std::placeholders::_1));

  // 读取本处理器自身的 verbose 参数（是否打印参数明细日志）
  auto get_param = getParamGetter(node_name_);
  get_param(verbose_, "verbose", false);
}

// ROS2 动态参数变更回调：将新参数值分发给已注册的各更新回调
rcl_interfaces::msg::SetParametersResult
ParametersHandler::dynamicParamsCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  // 加锁保护回调表，防止读取线程与配置线程并发访问
  std::lock_guard<std::mutex> lock(parameters_change_mutex_);

  // 前置回调：例如让优化器在改参前先完成一次完整的控制周期，
  // 避免打分中途张量尺寸/语义被改动
  for (auto & pre_cb : pre_callbacks_) {
    pre_cb();
  }

  // 逐个参数查找对应的更新回调并调用；
  // 未注册的参数（静态参数或拼写错误）打印警告
  for (auto & param : parameters) {
    const std::string & param_name = param.get_name();

    if (auto callback = get_param_callbacks_.find(param_name);
      callback != get_param_callbacks_.end())
    {
      callback->second(param);
    } else {
      RCLCPP_WARN(logger_, "Parameter %s not found", param_name.c_str());
    }
  }

  // 后置回调：所有参数更新完成后的收尾工作（如重置内部状态）
  for (auto & post_cb : post_callbacks_) {
    post_cb();
  }

  result.successful = true;
  return result;
}

}  // namespace mppi
