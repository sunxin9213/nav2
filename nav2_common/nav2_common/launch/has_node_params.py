# Copyright (c) 2021 PAL Robotics S.L.
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
# 【文件概况】launch 参数文件节点检查工具 HasNodeParams
# ----------------------------------------------------------------------------
# 概述: 实现 launch.Substitution 接口 —— 加载 YAML 参数文件并检查其中
#       是否包含指定节点名的配置段（顶层键），返回 "True"/"False" 字符串，
#       常与 IfCondition 组合实现"参数文件里有该节点配置才做某事"。
# 主要内容:
#   1. HasNodeParams 类
#      - source_file: 参数 YAML 文件路径
#      - node_name:   待检查的节点名（对应 YAML 顶层键）
# 使用场景: bringup launch 中多个节点共用一个参数文件时，判断某节点是否
#           在文件中有自己的配置段，从而决定是否把该文件传给它
#           （如 vector 之类的可选命名空间节点）。
# ============================================================================

from typing import List
from typing import Text

import yaml
import launch

import sys # delete this
# （上面这行 import sys 未被使用，是遗留代码）

class HasNodeParams(launch.Substitution):
  """
  Substitution that checks if a param file contains parameters for a node

  Used in launch system
  """

  def __init__(self,
    source_file: launch.SomeSubstitutionsType,
    node_name: Text) -> None:
    super().__init__()
    """
    Construct the substitution

    :param: source_file the parameter YAML file
    :param: node_name the name of the node to check
    """
    # 参数含义（中文补充）:
    #   source_file: 参数 YAML 文件路径（可含 launch 替换表达式）
    #   node_name:   要检查的节点名

    # import here to avoid loop
    # （延迟导入避免循环导入；文件路径规范化为替换列表）
    from launch.utilities import normalize_to_list_of_substitutions
    self.__source_file = normalize_to_list_of_substitutions(source_file)
    self.__node_name = node_name

  @property
  def name(self) -> List[launch.Substitution]:
    """Getter for name."""
    # 源文件路径的替换列表（launch 内部约定接口）
    return self.__source_file

  def describe(self) -> Text:
    """Return a description of this substitution as a string."""
    return ''

  def perform(self, context: launch.LaunchContext) -> Text:
    # 核心入口：解析出实际文件路径 → 加载 YAML →
    # 检查顶层键中是否存在该节点名，返回 "True" / "False" 字符串
    yaml_filename = launch.utilities.perform_substitutions(context, self.name)
    data = yaml.safe_load(open(yaml_filename, 'r'))

    if self.__node_name in data.keys():
        return "True"
    return "False"
