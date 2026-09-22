// Copyright (c) 2018 Intel Corporation
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
// limitations under the License. Reserved.

#include <memory>

#include "nav2_behaviors/behavior_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // 初始化 ROS 2 通信层，解析命令行参数并准备默认上下文。
  rclcpp::init(argc, argv);
  // 创建生命周期行为服务端；具体插件和通信实体在生命周期 configure 阶段初始化。
  auto recoveries_node = std::make_shared<behavior_server::BehaviorServer>();

  // 进入 ROS 2 executor，持续处理生命周期、Action 和 topic 回调，直到进程退出。
  rclcpp::spin(recoveries_node->get_node_base_interface());
  // 停止 executor 后释放 ROS 2 全局资源。
  rclcpp::shutdown();

  return 0;
}
