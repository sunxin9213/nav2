// Copyright (c) 2021 RoboTech Vision
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
// 文件说明（中文）：
// nav2_smoother 模块的独立可执行入口。
// 功能：初始化 rclcpp，构造 SmootherServer（路径平滑服务器生命周期节点），
// 阻塞自旋处理回调，退出时调用 rclcpp::shutdown 清理。
// （该节点也可作为 rclcpp_components 组件被容器进程加载，见 nav2_smoother.cpp 末尾。）
// ============================================================================

#include <memory>

#include "nav2_smoother/nav2_smoother.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // 初始化 ROS2 客户端库（解析命令行参数、设置信号处理等）
  rclcpp::init(argc, argv);
  // 创建路径平滑服务器节点实例
  auto node = std::make_shared<nav2_smoother::SmootherServer>();
  // 阻塞自旋：持续处理订阅、服务、动作与定时器回调，直到节点退出
  rclcpp::spin(node->get_node_base_interface());
  // 关闭 rclcpp，释放全局资源
  rclcpp::shutdown();

  return 0;
}
