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

#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_utils.hpp"

using std::cerr;
using namespace std::chrono_literals;

// 命令行工具 lifecycle_startup 的入口：
// 用法: lifecycle_startup <节点名1> <节点名2> ...
// 把列出的未配置生命周期节点依次带到 CONFIGURED -> ACTIVATED 状态
void usage()
{
  cerr << "Invalid command line.\n\n";
  cerr << "This command will take a set of unconfigured lifecycle nodes through the\n";
  cerr << "CONFIGURED to the ACTIVATED state\n";
  cerr << "The nodes are brought up in the order listed on the command line\n\n";
  cerr << "Usage:\n";
  cerr << " > lifecycle_startup <node name> ...\n";
  std::exit(1);
}

int main(int argc, char * argv[])
{
  // 无参数时打印用法并退出
  if (argc == 1) {
    usage();
  }
  // 初始化 rclcpp（注意不解析命令行参数：传 0/nullptr）
  rclcpp::init(0, nullptr);
  // 把 argv[1..argc) 转成节点名列表，按顺序启动，每次服务调用限时 10s
  nav2_util::startup_lifecycle_nodes(
    std::vector<std::string>(argv + 1, argv + argc),
    10s);
  rclcpp::shutdown();
}
