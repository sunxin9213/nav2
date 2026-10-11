# Copyright (c) 2019 Intel Corporation
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
# 【文件概况】nav2_common.launch 包入口
# ----------------------------------------------------------------------------
# 概述: 将本包提供的 4 个 launch 工具类统一导出，供各 launch 文件以
#       `from nav2_common.launch import RewrittenYaml, ...` 方式使用。
# 导出内容:
#   - RewrittenYaml:       YAML 参数文件的参数值/键名动态改写
#   - ReplaceString:       任意文件的纯文本字符串替换
#   - HasNodeParams:       检查参数文件是否包含某节点的配置段
#   - ParseMultiRobotPose: 解析多机器人位姿的命令行参数
# ============================================================================

from .has_node_params import HasNodeParams
from .rewritten_yaml import RewrittenYaml
from .replace_string import ReplaceString
from .parse_multirobot_pose import ParseMultiRobotPose
