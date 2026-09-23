// Copyright 2019 Rover Robotics
// Copyright (c) 2008, Willow Garage, Inc.
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
// map_saver_cli 命令行工具入口：一次性保存地图的独立可执行程序。
// 用法：map_saver_cli -t <地图话题> -f <文件名> --occ <阈值> --free <阈值>
//                 --fmt <图像格式> --mode trinary|scale|raw [--ros-args ...]
// 功能：解析命令行参数 -> 构造 MapSaver 节点 -> 手动触发 on_configure
// -> 调用 saveMapTopicToFile 订阅一帧地图并写盘 -> 退出。
// 与 main_server.cpp 的区别：CLI 版本不提供常驻服务，保存完即退出。
// ============================================================================

#include <memory>
#include <string>
#include <vector>
#include <stdexcept>

#include "nav2_map_server/map_mode.hpp"
#include "nav2_map_server/map_saver.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace nav2_map_server;  // NOLINT

// 命令行帮助文本

const char * USAGE_STRING{
  "Usage:\n"
  "  map_saver_cli [arguments] [--ros-args ROS remapping args]\n"
  "\n"
  "Arguments:\n"
  "  -h/--help\n"
  "  -t <map_topic>\n"
  "  -f <mapname>\n"
  "  --occ <threshold_occupied>\n"
  "  --free <threshold_free>\n"
  "  --fmt <image_format>\n"
  "  --mode trinary(default)/scale/raw\n"
  "\n"
  "NOTE: --ros-args should be passed at the end of command line"};

typedef enum
{
  COMMAND_MAP_TOPIC,      // -t：地图话题名
  COMMAND_MAP_FILE_NAME,  // -f：保存文件名
  COMMAND_IMAGE_FORMAT,   // --fmt：图像格式
  COMMAND_OCCUPIED_THRESH,// --occ：占据阈值
  COMMAND_FREE_THRESH,    // --free：空闲阈值
  COMMAND_MODE            // --mode：地图模式
} COMMAND_TYPE;

struct cmd_struct
{
  const char * cmd;             // 命令行选项字符串
  COMMAND_TYPE command_type;    // 对应的选项类型
};

// 参数解析结果状态
typedef enum
{
  ARGUMENTS_INVALID,  // 参数非法
  ARGUMENTS_VALID,    // 参数合法
  HELP_MESSAGE        // 请求帮助（-h/--help）
} ARGUMENTS_STATUS;

// Arguments parser
// Input parameters: logger, argc, argv
// Output parameters: map_topic, save_parameters
ARGUMENTS_STATUS parse_arguments(
  const rclcpp::Logger & logger, int argc, char ** argv,
  std::string & map_topic, SaveParameters & save_parameters)
{
  const struct cmd_struct commands[] = {
    {"-t", COMMAND_MAP_TOPIC},
    {"-f", COMMAND_MAP_FILE_NAME},
    {"--occ", COMMAND_OCCUPIED_THRESH},
    {"--free", COMMAND_FREE_THRESH},
    {"--mode", COMMAND_MODE},
    {"--fmt", COMMAND_IMAGE_FORMAT},
  };

  std::vector<std::string> arguments(argv + 1, argv + argc);
  std::vector<rclcpp::Parameter> params_from_args;


  size_t cmd_size = sizeof(commands) / sizeof(commands[0]);
  size_t i;
  for (auto it = arguments.begin(); it != arguments.end(); it++) {
    if (*it == "-h" || *it == "--help") {
      std::cout << USAGE_STRING << std::endl;
      return HELP_MESSAGE;
    }
    if (*it == "--ros-args") {
      break;
    }
    for (i = 0; i < cmd_size; i++) {
      if (commands[i].cmd == *it) {
        if ((it + 1) == arguments.end()) {
          RCLCPP_ERROR(logger, "Wrong argument: %s should be followed by a value.", it->c_str());
          return ARGUMENTS_INVALID;
        }
        it++;
        switch (commands[i].command_type) {
          case COMMAND_MAP_TOPIC:
            map_topic = *it;
            break;
          case COMMAND_MAP_FILE_NAME:
            save_parameters.map_file_name = *it;
            break;
          case COMMAND_FREE_THRESH:
            save_parameters.free_thresh = atof(it->c_str());
            break;
          case COMMAND_OCCUPIED_THRESH:
            save_parameters.occupied_thresh = atof(it->c_str());
            break;
          case COMMAND_IMAGE_FORMAT:
            save_parameters.image_format = *it;
            break;
          case COMMAND_MODE:
            try {
              save_parameters.mode = map_mode_from_string(*it);
            } catch (std::invalid_argument &) {
              save_parameters.mode = MapMode::Trinary;
              RCLCPP_WARN(
                logger,
                "Map mode parameter not recognized: %s, using default value (trinary)",
                it->c_str());
            }
            break;
        }
        break;
      }
    }
    if (i == cmd_size) {
      RCLCPP_ERROR(logger, "Wrong argument: %s", it->c_str());
      return ARGUMENTS_INVALID;
    }
  }

  return ARGUMENTS_VALID;
}

int main(int argc, char ** argv)
{
  // ROS2 init
  // 初始化 ROS2（必须先于任何节点/日志操作）
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("map_saver_cli");

  // Parse CLI-arguments
  // 解析命令行参数；默认地图话题为 "map"
  SaveParameters save_parameters;
  std::string map_topic = "map";
  switch (parse_arguments(logger, argc, argv, map_topic, save_parameters)) {
    case ARGUMENTS_INVALID:
      // 参数错误：退出码 -1
      rclcpp::shutdown();
      return -1;
    case HELP_MESSAGE:
      // 帮助信息已打印：正常退出
      rclcpp::shutdown();
      return 0;
    case ARGUMENTS_VALID:
      break;
  }

  // Call saveMapTopicToFile()
  // 直接复用 MapSaver 节点类执行一次性保存：
  // 手动调用 on_configure 完成参数初始化（不走完整生命周期管理器），
  // 然后订阅地图话题并写盘
  int retcode;
  try {
    auto map_saver = std::make_shared<nav2_map_server::MapSaver>();
    map_saver->on_configure(rclcpp_lifecycle::State());
    if (map_saver->saveMapTopicToFile(map_topic, save_parameters)) {
      retcode = 0;   // 保存成功
    } else {
      retcode = 1;   // 保存失败（如超时未收到地图）
    }
  } catch (std::exception & e) {
    RCLCPP_ERROR(logger, "Unexpected problem appear: %s", e.what());
    retcode = -1;    // 异常：退出码 -1
  }

  // Exit
  rclcpp::shutdown();
  return retcode;
}
