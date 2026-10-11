# Copyright 2019 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ============================================================================
# 【文件概况】nav2_common 的 CMake 扩展入口（find_package 时自动执行）
# ----------------------------------------------------------------------------
# 概述: ament 包在被人 find_package(nav2_common) 时会自动加载本文件，
#       作用是把 cmake/nav2_package.cmake 引入调用方环境，
#       使其 CMakeLists.txt 中可以直接调用 nav2_package() 宏。
# 主要内容:
#   1. 设置 ament 构建配置项与关键字的分隔符为 ":"
#      （避免与含 ":" 的链接器选项产生歧义）
#   2. include 同目录下（由 nav2_common_DIR 定位）的 nav2_package.cmake
# ============================================================================

set(AMENT_BUILD_CONFIGURATION_KEYWORD_SEPARATOR ":")

include("${nav2_common_DIR}/nav2_package.cmake")
