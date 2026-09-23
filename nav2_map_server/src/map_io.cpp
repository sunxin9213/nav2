/* Copyright 2019 Rover Robotics
 * Copyright 2010 Brian Gerkey
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// ============================================================================
// 文件说明（中文）：
// nav2_map_server 的地图 I/O 核心实现（加载 + 保存）。
// 模块定位：为 map_server（地图服务器）与 map_saver（地图保存器）
// 提供与具体 ROS 节点无关的纯函数式地图读写工具。
//
// 【地图加载流程】loadMapFromYaml
//   1. loadMapYaml：解析 YAML 元数据（image 路径、resolution 分辨率、
//      origin 原点 [x, y, yaw]、free_thresh/occupied_thresh 阈值、
//      mode 三种解释模式、negate 黑白反转标志）；
//   2. loadMapFromFile：用 ImageMagick（Magick++）读取 pgm/png 等图像，
//      逐像素计算灰度值 shade -> 占据度 occ（interpretValue 逻辑），
//      再按 MapMode 阈值化为 OccupancyGrid 的 data（int8 [-1,100]）：
//        - Trinary 三值模式：occ > occupied_thresh => 100（占据），
//          occ < free_thresh => 0（空闲），否则 -1（未知）；
//        - Scale 比例模式：透明像素为 -1（未知），阈值之间线性映射 0~100；
//        - Raw 原始模式：直接把灰度四舍五入为 0~255 原始占据值。
//      注意图像行序（上->下）与栅格行序（下->上）相反，写入时翻转。
//
// 【地图保存流程】saveMapToFile
//   1. checkSaveParameters：校验/补全保存参数（文件名、阈值范围
//      0 <= free < occupied <= 1、图像格式是否可写等）；
//   2. tryWriteMapToFile：将 OccupancyGrid 逆变换为灰度图像写盘
//      （Trinary: 白=空闲/黑=占据/205 灰=未知；Scale: 灰度=占据度、
//      透明=未知；Raw: 灰度=原始值），并生成配套 YAML 元数据文件。
// ============================================================================

#include "nav2_map_server/map_io.hpp"

#ifndef _WIN32
#include <libgen.h>
#endif
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>
#include <cstdlib>

#include "Magick++.h"
#include "nav2_util/geometry_utils.hpp"

#include "yaml-cpp/yaml.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav2_util/occ_grid_values.hpp"

#ifdef _WIN32
// https://github.com/rtv/Stage/blob/master/replace/dirname.c
// Windows 平台下没有 POSIX 的 dirname() 函数，这里提供等价实现：
// 去掉路径的最后一个组成部分，返回其目录部分（如 "/a/b/c.pgm" -> "/a/b"）
static
char * dirname(char * path)
{
  static const char dot[] = ".";
  char * last_slash;

  if (path == NULL) {
    return path;
  }

  /* Replace all "\" with "/" */
  // 统一把反斜杠替换为正斜杠，便于处理
  char * c = path;
  while (*c != '\0') {
    if (*c == '\\') {*c = '/';}
    ++c;
  }

  /* Find last '/'.  */
  last_slash = path != NULL ? strrchr(path, '/') : NULL;

  if (last_slash != NULL && last_slash == path) {
    /* The last slash is the first character in the string.  We have to
       return "/".  */
    ++last_slash;
  } else if (last_slash != NULL && last_slash[1] == '\0') {
    /* The '/' is the last character, we have to look further.  */
    last_slash = reinterpret_cast<char *>(memchr(path, last_slash - path, '/'));
  }

  if (last_slash != NULL) {
    /* Terminate the path.  */
    // 在最后一个斜杠处截断，得到目录部分
    last_slash[0] = '\0';
  } else {
    /* This assignment is ill-designed but the XPG specs require to
       return a string containing "." in any case no directory part is
       found and so a static and constant string is required.  */
    // 无目录部分时返回 "."
    path = reinterpret_cast<char *>(dot);
  }

  return path;
}
#endif

namespace nav2_map_server
{
using nav2_util::geometry_utils::orientationAroundZAxis;

// === Map input part ===
// ======================= 地图输入（加载）部分 =======================

/// Get the given subnode value.
/// The only reason this function exists is to wrap the exceptions in slightly nicer error messages,
/// including the name of the failed key
/// @throw YAML::Exception
// 读取 YAML 节点中指定键的值并转为类型 T；
// 包装 yaml-cpp 异常，附加上失败键名，便于定位错误
template<typename T>
T yaml_get_value(const YAML::Node & node, const std::string & key)
{
  try {
    return node[key].as<T>();
  } catch (YAML::Exception & e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}

// 获取 HOME 环境变量（用户主目录），未设置返回空串
std::string get_home_dir()
{
  if (const char * home_dir = std::getenv("HOME")) {
    return std::string{home_dir};
  }
  return std::string{};
}

// 若 YAML 文件路径以 "~/" 开头，则把 "~" 展开为用户主目录
std::string expand_user_home_dir_if_needed(
  std::string yaml_filename,
  std::string home_variable_value)
{
  // 不以 "~/" 开头则原样返回
  if (yaml_filename.size() < 2 || !(yaml_filename[0] == '~' && yaml_filename[1] == '/')) {
    return yaml_filename;
  }
  // "~/" 但未设置 HOME 变量：无法展开，提示并原样返回
  if (home_variable_value.empty()) {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger(
        "map_io"), "Map yaml file name starts with '~/' but no HOME variable set. \n"
        << "[INFO] [map_io] User home dir will be not expanded \n");
    return yaml_filename;
  }
  // 用主目录替换首字符 '~'
  const std::string prefix{home_variable_value};
  return yaml_filename.replace(0, 1, prefix);
}

// ---------------------------------------------------------------------------
// 解析地图 YAML 元数据文件，返回 LoadParameters 结构。
// YAML 内容示例：
//   image: map.pgm        # 栅格图像文件（相对 YAML 所在目录或绝对路径）
//   resolution: 0.05      # 分辨率（米/像素）
//   origin: [x, y, yaw]   # 地图左下角在世界坐标系下的位姿
//   negate: 0             # 是否黑白反转
//   occupied_thresh: 0.65 # 判定占据的灰度阈值
//   free_thresh: 0.25     # 判定空闲的灰度阈值
//   mode: trinary         # trinary / scale / raw 三种解释模式
// ---------------------------------------------------------------------------
LoadParameters loadMapYaml(const std::string & yaml_filename)
{
  // 加载 YAML（支持 ~/ 展开），yaml-cpp 解析失败会抛异常
  YAML::Node doc = YAML::LoadFile(expand_user_home_dir_if_needed(yaml_filename, get_home_dir()));
  LoadParameters load_parameters;

  // 读取图像文件名；为空则抛异常
  auto image_file_name = yaml_get_value<std::string>(doc, "image");
  if (image_file_name.empty()) {
    throw YAML::Exception(doc["image"].Mark(), "The image tag was empty.");
  }
  // 相对路径：以 YAML 文件所在目录为基准拼接为绝对路径
  // （dirname 需要可变 char*，故先拷贝到 vector）
  if (image_file_name[0] != '/') {
    // dirname takes a mutable char *, so we copy into a vector
    std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
    fname_copy.push_back('\0');
    image_file_name = std::string(dirname(fname_copy.data())) + '/' + image_file_name;
  }
  load_parameters.image_file_name = image_file_name;

  // 分辨率与原点；origin 必须是 3 元素 [x, y, yaw]
  load_parameters.resolution = yaml_get_value<double>(doc, "resolution");
  load_parameters.origin = yaml_get_value<std::vector<double>>(doc, "origin");
  if (load_parameters.origin.size() != 3) {
    throw YAML::Exception(
            doc["origin"].Mark(), "value of the 'origin' tag should have 3 elements, not " +
            std::to_string(load_parameters.origin.size()));
  }

  // 空闲/占据灰度阈值
  load_parameters.free_thresh = yaml_get_value<double>(doc, "free_thresh");
  load_parameters.occupied_thresh = yaml_get_value<double>(doc, "occupied_thresh");

  // 地图解释模式：未指定时默认 Trinary（三值模式）
  auto map_mode_node = doc["mode"];
  if (!map_mode_node.IsDefined()) {
    load_parameters.mode = MapMode::Trinary;
  } else {
    load_parameters.mode = map_mode_from_string(map_mode_node.as<std::string>());
  }

  // negate 可写成整数 0/1 或布尔 true/false，两种形式都兼容
  try {
    load_parameters.negate = yaml_get_value<int>(doc, "negate");
  } catch (YAML::Exception &) {
    load_parameters.negate = yaml_get_value<bool>(doc, "negate");
  }

  // 打印解析出的全部元数据便于调试
  RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "resolution: " << load_parameters.resolution);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "origin[0]: " << load_parameters.origin[0]);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "origin[1]: " << load_parameters.origin[1]);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "origin[2]: " << load_parameters.origin[2]);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "free_thresh: " << load_parameters.free_thresh);
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(
      "map_io"), "occupied_thresh: " << load_parameters.occupied_thresh);
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("map_io"),
    "mode: " << map_mode_to_string(load_parameters.mode));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "negate: " << load_parameters.negate);

  return load_parameters;
}

// ---------------------------------------------------------------------------
// 从图像文件加载占据栅格地图（核心：像素 -> OccupancyGrid data 的转换）。
// 灰度语义（negate=0 默认）：白（shade≈1）= 空闲，黑（shade≈0）= 占据；
// negate=1 时黑白含义反转。占据度 occ 定义为：
//   occ = negate ? shade : 1 - shade
// 再按 mode 阈值化为 -1（未知）/ 0（空闲）/ 100（占据）或连续值。
// ---------------------------------------------------------------------------
void loadMapFromFile(
  const LoadParameters & load_parameters,
  nav_msgs::msg::OccupancyGrid & map)
{
  // 初始化 ImageMagick 库（必须在使用图像功能前调用一次）
  Magick::InitializeMagick(nullptr);
  nav_msgs::msg::OccupancyGrid msg;

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("map_io"), "Loading image_file: " <<
      load_parameters.image_file_name);
  // 读取图像文件（支持 pgm/png/bmp 等 ImageMagick 支持的格式）
  Magick::Image img(load_parameters.image_file_name);

  // Copy the image data into the map structure
  // 图像尺寸即栅格地图尺寸（宽 x 高，单位：像素/格）
  msg.info.width = img.size().width();
  msg.info.height = img.size().height();

  // 元数据：分辨率与原点位姿（origin[2] 为 yaw，转成四元数）
  msg.info.resolution = load_parameters.resolution;
  msg.info.origin.position.x = load_parameters.origin[0];
  msg.info.origin.position.y = load_parameters.origin[1];
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation = orientationAroundZAxis(load_parameters.origin[2]);

  // Allocate space to hold the data
  // 分配 data 数组（行优先，大小 = 宽 * 高，int8 [-1,100]）
  msg.data.resize(msg.info.width * msg.info.height);

  // Copy pixel data into the map structure
  // 逐像素转换（注意：图像第 0 行在顶部，栅格第 0 行在底部，
  // 写入时按 (height - y - 1) 做垂直翻转）
  for (size_t y = 0; y < msg.info.height; y++) {
    for (size_t x = 0; x < msg.info.width; x++) {
      auto pixel = img.pixelColor(x, y);

      // 收集 R/G/B 通道值（后续取平均得到灰度）
      std::vector<Magick::Quantum> channels = {pixel.redQuantum(), pixel.greenQuantum(),
        pixel.blueQuantum()};
      if (load_parameters.mode == MapMode::Trinary && img.matte()) {
        // To preserve existing behavior, average in alpha with color channels in Trinary mode.
        // CAREFUL. alpha is inverted from what you might expect. High = transparent, low = opaque
        // 三值模式且图像带透明通道时，把不透明度并入通道平均：
        // 注意 alpha 语义相反（值越大越透明），故用 MaxRGB - alpha
        channels.push_back(MaxRGB - pixel.alphaQuantum());
      }
      double sum = 0;
      for (auto c : channels) {
        sum += c;
      }
      /// on a scale from 0.0 to 1.0 how bright is the pixel?
      // shade：0.0（黑）~ 1.0（白）的像素亮度
      double shade = Magick::ColorGray::scaleQuantumToDouble(sum / channels.size());

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels occupied. Otherwise, it's vice versa.
      /// on a scale from 0.0 to 1.0, how occupied is the map cell (before thresholding)?
      // occ：0.0（完全空闲）~ 1.0（完全占据）的占据度
      //（negate=true 时黑白语义反转：越白越占据）
      double occ = (load_parameters.negate ? shade : 1.0 - shade);

      int8_t map_cell;
      switch (load_parameters.mode) {
        // 三值模式：只输出占据(100)/空闲(0)/未知(-1)三种状态
        case MapMode::Trinary:
          if (load_parameters.occupied_thresh < occ) {
            map_cell = nav2_util::OCC_GRID_OCCUPIED;   // 100：占据
          } else if (occ < load_parameters.free_thresh) {
            map_cell = nav2_util::OCC_GRID_FREE;       // 0：空闲
          } else {
            map_cell = nav2_util::OCC_GRID_UNKNOWN;    // -1：未知（灰区）
          }
          break;
        // 比例模式：介于两阈值之间时线性映射为 1~99 的连续占据度，
        // 透明像素表示未知
        case MapMode::Scale:
          if (pixel.alphaQuantum() != OpaqueOpacity) {
            map_cell = nav2_util::OCC_GRID_UNKNOWN;    // 透明 => 未知
          } else if (load_parameters.occupied_thresh < occ) {
            map_cell = nav2_util::OCC_GRID_OCCUPIED;
          } else if (occ < load_parameters.free_thresh) {
            map_cell = nav2_util::OCC_GRID_FREE;
          } else {
            // 阈值之间：线性映射到 (0, 100) 开区间
            map_cell = std::rint(
              (occ - load_parameters.free_thresh) /
              (load_parameters.occupied_thresh - load_parameters.free_thresh) * 100.0);
          }
          break;
        // 原始模式：不经阈值，直接把灰度量化为 0~255 的原始占据值
        // （超出 [0,100] 合法范围之外的按未知处理）
        case MapMode::Raw: {
            double occ_percent = std::round(shade * 255);
            if (nav2_util::OCC_GRID_FREE <= occ_percent &&
              occ_percent <= nav2_util::OCC_GRID_OCCUPIED)
            {
              map_cell = static_cast<int8_t>(occ_percent);
            } else {
              map_cell = nav2_util::OCC_GRID_UNKNOWN;
            }
            break;
          }
        default:
          throw std::runtime_error("Invalid map mode");
      }
      // 垂直翻转后写入（图像顶行 -> 栅格底行）
      msg.data[msg.info.width * (msg.info.height - y - 1) + x] = map_cell;
    }
  }

  // Since loadMapFromFile() does not belong to any node, publishing in a system time.
  // 本函数不属于任何节点，使用系统时钟打时间戳
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  msg.info.map_load_time = clock.now();
  msg.header.frame_id = "map";  // 地图坐标系固定为 "map"
  msg.header.stamp = clock.now();

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(
      "map_io"), "Read map " << load_parameters.image_file_name
                             << ": " << msg.info.width << " X " << msg.info.height << " map @ "
                             << msg.info.resolution << " m/cell");

  map = msg;
}

// ---------------------------------------------------------------------------
// 顶层加载接口：YAML -> LoadParameters -> OccupancyGrid。
// 返回状态枚举：LOAD_MAP_SUCCESS / MAP_DOES_NOT_EXIST /
// INVALID_MAP_METADATA / INVALID_MAP_DATA，供调用方（map_server 的
// load_map 服务回调）区分失败原因。
// ---------------------------------------------------------------------------
LOAD_MAP_STATUS loadMapFromYaml(
  const std::string & yaml_file,
  nav_msgs::msg::OccupancyGrid & map)
{
  // 文件名为空：直接返回"地图不存在"
  if (yaml_file.empty()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_io"), "YAML file name is empty, can't load!");
    return MAP_DOES_NOT_EXIST;
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "Loading yaml file: " << yaml_file);
  LoadParameters load_parameters;
  try {
    // 第一步：解析 YAML 元数据
    load_parameters = loadMapYaml(yaml_file);
  } catch (YAML::Exception & e) {
    // YAML 语法/字段错误：返回"元数据非法"（附带行列位置）
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "map_io"), "Failed processing YAML file " << yaml_file << " at position (" <<
        e.mark.line << ":" << e.mark.column << ") for reason: " << e.what());
    return INVALID_MAP_METADATA;
  } catch (std::exception & e) {
    // 其他异常（如文件不存在）：同样视为元数据非法
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("map_io"), "Failed to parse map YAML loaded from file " << yaml_file <<
        " for reason: " << e.what());
    return INVALID_MAP_METADATA;
  }
  try {
    // 第二步：读取图像并转为占据栅格
    loadMapFromFile(load_parameters, map);
  } catch (std::exception & e) {
    // 图像读取/解码失败：返回"地图数据非法"
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "map_io"), "Failed to load image file " << load_parameters.image_file_name <<
        " for reason: " << e.what());
    return INVALID_MAP_DATA;
  }

  return LOAD_MAP_SUCCESS;
}

// === Map output part ===
// ======================= 地图输出（保存）部分 =======================

/**
 * @brief Checks map saving parameters for consistency
 * @param save_parameters Map saving parameters.
 * NOTE: save_parameters could be updated during function execution.
 * @throw std::exception in case of inconsistent parameters
 *
 * 校验并补全保存参数：文件名为空则按时间戳自动生成；
 * 阈值缺省时填默认值（occupied 0.65 / free 0.25）并检查取值范围
 * （0 <= free < occupied <= 1）；图像格式缺省时 Scale 模式用 png、
 * 其他用 pgm，并验证格式可写（不支持时回退 png）。
 */
void checkSaveParameters(SaveParameters & save_parameters)
{
  // Magick must me initialized before any activity with images
  // 使用图像功能前必须初始化 ImageMagick
  Magick::InitializeMagick(nullptr);

  // Checking map file name
  // 文件名未指定：自动生成 "map_<当前秒数>"
  if (save_parameters.map_file_name == "") {
    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    save_parameters.map_file_name = "map_" +
      std::to_string(static_cast<int>(clock.now().seconds()));
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("map_io"), "Map file unspecified. Map will be saved to " <<
        save_parameters.map_file_name << " file");
  }

  // Checking thresholds
  // 阈值缺省：occupied 0.65 / free 0.25
  if (save_parameters.occupied_thresh == 0.0) {
    save_parameters.occupied_thresh = 0.65;
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger(
        "map_io"), "Occupied threshold unspecified. Setting it to default value: " <<
        save_parameters.occupied_thresh);
  }
  if (save_parameters.free_thresh == 0.0) {
    save_parameters.free_thresh = 0.25;
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("map_io"), "Free threshold unspecified. Setting it to default value: " <<
        save_parameters.free_thresh);
  }
  // 阈值合法性检查：occupied <= 1.0、free >= 0.0、free < occupied
  if (1.0 < save_parameters.occupied_thresh) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_io"), "Threshold_occupied must be 1.0 or less");
    throw std::runtime_error("Incorrect thresholds");
  }
  if (save_parameters.free_thresh < 0.0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_io"), "Free threshold must be 0.0 or greater");
    throw std::runtime_error("Incorrect thresholds");
  }
  if (save_parameters.occupied_thresh <= save_parameters.free_thresh) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "map_io"), "Threshold_free must be smaller than threshold_occupied");
    throw std::runtime_error("Incorrect thresholds");
  }

  // Checking image format
  // 图像格式缺省：Scale 模式需要透明通道故用 png，其他模式用 pgm
  if (save_parameters.image_format == "") {
    save_parameters.image_format = save_parameters.mode == MapMode::Scale ? "png" : "pgm";
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("map_io"), "Image format unspecified. Setting it to: " <<
        save_parameters.image_format);
  }

  // 格式名统一转小写
  std::transform(
    save_parameters.image_format.begin(),
    save_parameters.image_format.end(),
    save_parameters.image_format.begin(),
    [](unsigned char c) {return std::tolower(c);});

  // 推荐格式白名单：bmp / pgm / png；其他格式仅告警不阻止
  const std::vector<std::string> BLESSED_FORMATS{"bmp", "pgm", "png"};
  if (
    std::find(BLESSED_FORMATS.begin(), BLESSED_FORMATS.end(), save_parameters.image_format) ==
    BLESSED_FORMATS.end())
  {
    std::stringstream ss;
    bool first = true;
    for (auto & format_name : BLESSED_FORMATS) {
      if (!first) {
        ss << ", ";
      }
      ss << "'" << format_name << "'";
      first = false;
    }
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("map_io"), "Requested image format '" << save_parameters.image_format <<
        "' is not one of the recommended formats: " << ss.str());
  }
  const std::string FALLBACK_FORMAT = "png";

  // 查询 ImageMagick 是否支持写入该格式；不可写/不识别则回退 png
  try {
    Magick::CoderInfo info(save_parameters.image_format);
    if (!info.isWritable()) {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("map_io"), "Format '" << save_parameters.image_format <<
          "' is not writable. Using '" << FALLBACK_FORMAT << "' instead");
      save_parameters.image_format = FALLBACK_FORMAT;
    }
  } catch (Magick::ErrorOption & e) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger(
        "map_io"), "Format '" << save_parameters.image_format << "' is not usable. Using '" <<
        FALLBACK_FORMAT << "' instead:" << std::endl << e.what());
    save_parameters.image_format = FALLBACK_FORMAT;
  }

  // Checking map mode
  // Scale 模式依赖透明通道，而 pgm/jpg 不支持透明：给出告警建议改用 png
  if (
    save_parameters.mode == MapMode::Scale &&
    (save_parameters.image_format == "pgm" ||
    save_parameters.image_format == "jpg" ||
    save_parameters.image_format == "jpeg"))
  {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("map_io"), "Map mode 'scale' requires transparency, but format '" <<
        save_parameters.image_format <<
        "' does not support it. Consider switching image format to 'png'.");
  }
}

/**
 * @brief Tries to write map data into a file
 * @param map Occupancy grid data
 * @param save_parameters Map saving parameters
 * @throw std::expection in case of problem
 *
 * 将 OccupancyGrid 写为图像文件（占据度 -> 灰度/透明像素的逆变换）
 * 并输出配套的 YAML 元数据文件。
 */
void tryWriteMapToFile(
  const nav_msgs::msg::OccupancyGrid & map,
  const SaveParameters & save_parameters)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(
      "map_io"), "Received a " << map.info.width << " X " << map.info.height << " map @ " <<
      map.info.resolution << " m/pix");

  // 图像数据文件名：<地图名>.<格式>
  std::string mapdatafile = save_parameters.map_file_name + "." + save_parameters.image_format;
  {
    // should never see this color, so the initialization value is just for debugging
    // 创建画布（初始化为红色仅便于调试，正常会被逐像素覆盖）
    Magick::Image image({map.info.width, map.info.height}, "red");

    // In scale mode, we need the alpha (matte) channel. Else, we don't.
    // NOTE: GraphicsMagick seems to have trouble loading the alpha channel when saved with
    // Magick::GreyscaleMatte, so we use TrueColorMatte instead.
    // Scale 模式需要透明通道（TrueColorMatte），否则灰度即可
    image.type(
      save_parameters.mode == MapMode::Scale ?
      Magick::TrueColorMatteType : Magick::GrayscaleType);

    // Since we only need to support 100 different pixel levels, 8 bits is fine
    // 8 位深度足够表示 100 级占据度
    image.depth(8);

    // 阈值换算为占据度整数刻度（0~100）
    int free_thresh_int = std::rint(save_parameters.free_thresh * 100.0);
    int occupied_thresh_int = std::rint(save_parameters.occupied_thresh * 100.0);

    // 逐像素逆变换（与加载时相反：栅格底行 -> 图像顶行）
    for (size_t y = 0; y < map.info.height; y++) {
      for (size_t x = 0; x < map.info.width; x++) {
        int8_t map_cell = map.data[map.info.width * (map.info.height - y - 1) + x];

        Magick::Color pixel;

        switch (save_parameters.mode) {
          // 三值模式：白(254)=空闲、黑(0)=占据、灰(205)=未知
          case MapMode::Trinary:
            if (map_cell < 0 || 100 < map_cell) {
              pixel = Magick::ColorGray(205 / 255.0);   // 未知 => 中灰
            } else if (map_cell <= free_thresh_int) {
              pixel = Magick::ColorGray(254 / 255.0);   // 空闲 => 白
            } else if (occupied_thresh_int <= map_cell) {
              pixel = Magick::ColorGray(0 / 255.0);     // 占据 => 黑
            } else {
              pixel = Magick::ColorGray(205 / 255.0);   // 阈值之间 => 中灰（未知）
            }
            break;
          // 比例模式：灰度 = 1 - 占据度/100，未知 => 50% 灰 + 全透明
          case MapMode::Scale:
            if (map_cell < 0 || 100 < map_cell) {
              pixel = Magick::ColorGray{0.5};
              pixel.alphaQuantum(TransparentOpacity);
            } else {
              pixel = Magick::ColorGray{(100.0 - map_cell) / 100.0};
            }
            break;
          // 原始模式：占据值直接量化为灰度，未知 => 白色
          case MapMode::Raw:
            Magick::Quantum q;
            if (map_cell < 0 || 100 < map_cell) {
              q = MaxRGB;
            } else {
              q = map_cell / 255.0 * MaxRGB;
            }
            pixel = Magick::Color(q, q, q);
            break;
          default:
            RCLCPP_ERROR_STREAM(
              rclcpp::get_logger(
                "map_io"), "Map mode should be Trinary, Scale or Raw");
            throw std::runtime_error("Invalid map mode");
        }
        image.pixelColor(x, y, pixel);
      }
    }

    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("map_io"),
      "Writing map occupancy data to " << mapdatafile);
    // 写出图像文件
    image.write(mapdatafile);
  }

  // 元数据文件名：<地图名>.yaml
  std::string mapmetadatafile = save_parameters.map_file_name + ".yaml";
  {
    std::ofstream yaml(mapmetadatafile);

    // 从原点四元数中提取 yaw 角（原点用 [x, y, yaw] 三元组表示）
    geometry_msgs::msg::Quaternion orientation = map.info.origin.orientation;
    tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    // 取图像文件名（去掉目录部分，YAML 中用相对名引用）
    const int file_name_index = mapdatafile.find_last_of("/\\");
    std::string image_name = mapdatafile.substr(file_name_index + 1);

    // 用 yaml-cpp 生成元数据（保留 3 位有效数字）
    YAML::Emitter e;
    e << YAML::Precision(3);
    e << YAML::BeginMap;
    e << YAML::Key << "image" << YAML::Value << image_name;
    e << YAML::Key << "mode" << YAML::Value << map_mode_to_string(save_parameters.mode);
    e << YAML::Key << "resolution" << YAML::Value << map.info.resolution;
    e << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq << map.info.origin.position.x <<
      map.info.origin.position.y << yaw << YAML::EndSeq;
    e << YAML::Key << "negate" << YAML::Value << 0;
    e << YAML::Key << "occupied_thresh" << YAML::Value << save_parameters.occupied_thresh;
    e << YAML::Key << "free_thresh" << YAML::Value << save_parameters.free_thresh;

    // 生成失败则报错（元数据可能不完整）
    if (!e.good()) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("map_io"), "YAML writer failed with an error " << e.GetLastError() <<
          ". The map metadata may be invalid.");
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "Writing map metadata to " << mapmetadatafile);
    std::ofstream(mapmetadatafile) << e.c_str();
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "Map saved");
}

// ---------------------------------------------------------------------------
// 顶层保存接口：先做参数一致性检查，再尝试写出地图文件。
// 异常统一捕获并转为 false 返回值，供调用方（map_saver）使用。
// ---------------------------------------------------------------------------
bool saveMapToFile(
  const nav_msgs::msg::OccupancyGrid & map,
  const SaveParameters & save_parameters)
{
  // Local copy of SaveParameters that might be modified by checkSaveParameters()
  // 局部拷贝：checkSaveParameters 可能补全/修改参数，不影响调用方
  SaveParameters save_parameters_loc = save_parameters;

  try {
    // Checking map parameters for consistency
    // 校验并补全参数
    checkSaveParameters(save_parameters_loc);

    // 写出图像 + YAML 元数据
    tryWriteMapToFile(map, save_parameters_loc);
  } catch (std::exception & e) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("map_io"),
      "Failed to write map for reason: " << e.what());
    return false;
  }
  return true;
}

}  // namespace nav2_map_server
