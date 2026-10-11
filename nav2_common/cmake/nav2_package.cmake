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
# 【文件概况】Nav2 各包统一使用的 CMake 工程设置宏
# ----------------------------------------------------------------------------
# 概述: 定义 nav2_package() 宏。各 nav2 包在 CMakeLists.txt 顶部
#       find_package(nav2_common) 后调用 nav2_package()，
#       即可统一施加整个项目约定俗成的编译配置。
# 主要内容（按执行顺序）:
#   1. 构建类型: 未指定时默认 Release，并为 cmake-gui 列出可选值
#   2. C++ 标准: 未指定时默认 C++17
#   3. GCC/Clang: 追加严格警告选项，且 -Werror 把警告视为错误
#   4. COVERAGE_ENABLED 选项: 开启 gcov 覆盖率的编译/链接选项
#   5. MSVC 特例: 自动导出全部符号、启用数学常量宏
# ============================================================================

#
# Standard Nav2 project setup
#
# @public
#
macro(nav2_package)
  # --- 1. 构建类型: 未指定（含多配置生成器未配置）时默认 Release ---
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to Release as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE
        STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    # 在 cmake-gui 中把可选构建类型显示为下拉列表
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
      "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
  endif()

  # --- 2. C++ 标准默认值（注释写 14 但实际是 17，以代码为准） ---
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()

  # --- 3. GCC/Clang: 严格警告 + 警告视为错误 + 位置无关代码 ---
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC)
  endif()

  # --- 4. 可选的代码覆盖率支持（cmake -DCOVERAGE_ENABLED=TRUE 开启） ---
  option(COVERAGE_ENABLED "Enable code coverage" FALSE)
  if(COVERAGE_ENABLED)
    # 编译与链接（可执行文件/动态库）都加 --coverage，配合 gcov 统计覆盖率
    add_compile_options(--coverage)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
  endif()

  # --- 5. MSVC（Windows）编译器的默认设置 ---
  if(MSVC)
    # https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
    # 自动导出全部符号: 免去逐个写 __declspec(dllexport) 的工作
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

    # Enable Math Constants
    # https://docs.microsoft.com/en-us/cpp/c-runtime-library/math-constants?view=vs-2019
    # 启用 M_PI 等 POSIX 数学常量宏（MSVC 默认不提供）
    add_compile_definitions(
      _USE_MATH_DEFINES
    )
  endif()
endmacro()
