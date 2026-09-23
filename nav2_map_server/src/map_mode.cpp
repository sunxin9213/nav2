// Copyright 2019 Rover Robotics
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
// MapMode 枚举与字符串的相互转换工具（YAML/服务请求中的 "mode" 字段）。
// 三种模式的语义见 map_mode.hpp：
//   trinary —— 三值模式（占据 100 / 空闲 0 / 未知 -1）
//   scale   —— 比例模式（连续占据度 + 透明像素表示未知）
//   raw     —— 原始模式（灰度直接映射占据值）
// ============================================================================

#include "nav2_map_server/map_mode.hpp"

#include <stdexcept>
#include <string>

namespace nav2_map_server
{
// 枚举 -> 字符串（用于写 YAML 元数据与日志）
const char * map_mode_to_string(MapMode map_mode)
{
  switch (map_mode) {
    case MapMode::Trinary:
      return "trinary";
    case MapMode::Scale:
      return "scale";
    case MapMode::Raw:
      return "raw";
    default:
      throw std::invalid_argument("map_mode");
  }
}

// 字符串 -> 枚举（用于解析 YAML / SaveMap 服务请求；非法值抛异常）
MapMode map_mode_from_string(std::string map_mode_name)
{
  // 大小写不敏感：先统一转小写
  for (auto & c : map_mode_name) {
    c = tolower(c);
  }

  if (map_mode_name == "scale") {
    return MapMode::Scale;
  } else if (map_mode_name == "raw") {
    return MapMode::Raw;
  } else if (map_mode_name == "trinary") {
    return MapMode::Trinary;
  } else {
    throw std::invalid_argument("map_mode_name");
  }
}
}  // namespace nav2_map_server
