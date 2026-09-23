// Copyright (c) 2020 Samsung Research Russia
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

// ============================================================================
// 文件说明（中文）：
// map_saver_server 独立可执行入口：常驻运行的地图保存服务节点。
// 与 map_saver_cli（一次性保存）不同，本程序持续运行，
// 由生命周期管理器触发 configure/activate 后，
// 对外提供 map_saver/save_map 服务（nav2_msgs/srv/SaveMap），
// 可随时按请求保存地图。
// ============================================================================

#include <memory>
#include <stdexcept>
#include <string>

#include "nav2_map_server/map_saver.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // 初始化 ROS2 客户端库
  rclcpp::init(argc, argv);

  // 创建 MapSaver 生命周期节点并阻塞自旋处理服务请求
  auto logger = rclcpp::get_logger("map_saver_server");
  auto service_node = std::make_shared<nav2_map_server::MapSaver>();
  rclcpp::spin(service_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
