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

/* OccupancyGrid map input-output library */
/* 地图输入输出库（中文说明）：
   本头文件声明 nav2_map_server 的地图读写接口：
   - 输入（加载）：loadMapYaml（解析 YAML 元数据）、loadMapFromFile
     （图像 -> OccupancyGrid）、loadMapFromYaml（二者组合的顶层接口）；
   - 输出（保存）：saveMapToFile（OccupancyGrid -> 图像 + YAML）；
   - 辅助：expand_user_home_dir_if_needed（~/ 路径展开）。
   具体实现见 src/map_io.cpp。 */

#ifndef NAV2_MAP_SERVER__MAP_IO_HPP_
#define NAV2_MAP_SERVER__MAP_IO_HPP_

#include <string>
#include <vector>

#include "nav2_map_server/map_mode.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

/* Map input part */
/* ======================= 地图输入部分 ======================= */

namespace nav2_map_server
{

// 地图加载参数：从 YAML 元数据解析得到
struct LoadParameters
{
  std::string image_file_name;          // 栅格图像文件名（绝对路径）
  double resolution{0};                 // 地图分辨率（米/像素）
  std::vector<double> origin{0, 0, 0};  // 地图原点 [x, y, yaw]（左下角世界坐标）
  double free_thresh;                   // 空闲判定灰度阈值
  double occupied_thresh;               // 占据判定灰度阈值
  MapMode mode;                         // 像素解释模式（trinary/scale/raw）
  bool negate;                          // 是否黑白反转
};

// 地图加载结果状态码
typedef enum
{
  LOAD_MAP_SUCCESS,       // 加载成功
  MAP_DOES_NOT_EXIST,     // 文件名为空 / 地图不存在
  INVALID_MAP_METADATA,   // YAML 元数据解析失败
  INVALID_MAP_DATA        // 图像数据读取失败
} LOAD_MAP_STATUS;

/**
 * @brief Load and parse the given YAML file
 * @param yaml_filename Name of the map file passed though parameter
 * @return Map loading parameters obtained from YAML file
 * @throw YAML::Exception
 */
LoadParameters loadMapYaml(const std::string & yaml_filename);

/**
 * @brief Load the image from map file and generate an OccupancyGrid
 * @param load_parameters Parameters of loading map
 * @param map Output loaded map
 * @throw std::exception
 */
void loadMapFromFile(
  const LoadParameters & load_parameters,
  nav_msgs::msg::OccupancyGrid & map);

/**
 * @brief Load the map YAML, image from map file and
 * generate an OccupancyGrid
 * @param yaml_file Name of input YAML file
 * @param map Output loaded map
 * @return status of map loaded
 */
LOAD_MAP_STATUS loadMapFromYaml(
  const std::string & yaml_file,
  nav_msgs::msg::OccupancyGrid & map);


/* Map output part */
/* ======================= 地图输出部分 ======================= */

// 地图保存参数
struct SaveParameters
{
  std::string map_file_name{""};    // 保存文件基础名（不含扩展名）
  std::string image_format{""};     // 图像格式（pgm/png/bmp...）
  double free_thresh{0.0};          // 空闲阈值（0 表示未指定，用默认值）
  double occupied_thresh{0.0};      // 占据阈值（0 表示未指定，用默认值）
  MapMode mode{MapMode::Trinary};   // 保存模式（默认三值模式）
};

/**
 * @brief Write OccupancyGrid map to file
 * @param map OccupancyGrid map data
 * @param save_parameters Map saving parameters.
 * @return true or false
 */
bool saveMapToFile(
  const nav_msgs::msg::OccupancyGrid & map,
  const SaveParameters & save_parameters);

/**
 * @brief Expand ~/ to home user dir.
 * @param yaml_filename Name of input YAML file.
 * @param home_dir Expanded `~/`home dir or empty string if HOME not set
 *
 * @return Expanded string or input string if `~/` not expanded
 */
std::string expand_user_home_dir_if_needed(
  std::string yaml_filename,
  std::string home_dir);

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_IO_HPP_
