# ============================================================================
# 【文件概况】nav2_common 包的构建配置
# ----------------------------------------------------------------------------
# 概述: 本包没有任何 C++ 代码，只做两件事:
#   1. 以 Python 包形式安装 nav2_common/launch 下的 4 个 launch 工具
#      （RewrittenYaml / ReplaceString / HasNodeParams / ParseMultiRobotPose）
#   2. 把 CMake 工程设置宏分发给所有 nav2 包:
#      - nav2_common-extras.cmake: find_package 时自动加载（见其文件头）
#      - cmake/ 目录: 安装到 share 供其引用 nav2_package.cmake
# ============================================================================

# 最低 CMake 版本要求
cmake_minimum_required(VERSION 3.5)

# NONE: 本包不编译任何语言（纯 Python + CMake 脚本包）
project(nav2_common NONE)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 把 nav2_common/ 目录作为 Python 包安装到系统（含 launch 子包）
ament_python_install_package(nav2_common)

ament_package(
  # find_package(nav2_common) 时自动执行该文件，进而引入 nav2_package.cmake
  CONFIG_EXTRAS "nav2_common-extras.cmake"
)

# 安装 cmake/ 目录（内含 nav2_package.cmake）到 share/nav2_common
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
