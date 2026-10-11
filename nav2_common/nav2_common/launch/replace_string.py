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
# 【文件概况】launch 纯文本替换工具 ReplaceString
# ----------------------------------------------------------------------------
# 概述: 实现 launch.Substitution 接口 —— 逐行读取给定文件，把出现的指定
#       字符串替换为新值后写入临时文件，返回临时文件路径。
#       与 RewrittenYaml 不同，本类不解析文件结构，做的是纯文本级替换，
#       因此可用于任意格式的文件（YAML/XML/URDF/SDF 等）。
# 主要内容:
#   1. ReplaceString 类
#      - replacements: {旧字符串: 新字符串} 替换映射（可含 launch 表达式）
#      - condition:    可选 launch 条件，不满足时直接返回原文件路径（不替换）
# 使用场景: 多机器人 launch 中替换参数文件/机器人模型里的命名空间占位串
#           （如把占位符替换成实际的机器人命名空间）。
# ============================================================================

from typing import Dict
from typing import List
from typing import Text
from typing import Optional
import tempfile
import launch

class ReplaceString(launch.Substitution):
  """
  Substitution that replaces strings on a given file.

  Used in launch system
  """

  def __init__(self,
    source_file: launch.SomeSubstitutionsType,
    replacements: Dict,
    condition: Optional[launch.Condition] = None) -> None:
    super().__init__()
    # 参数含义（中文补充）:
    #   source_file:  原始文件路径（可含 launch 替换表达式）
    #   replacements: {待替换字符串: 替换后的字符串} 映射表
    #   condition:    执行条件，None 表示无条件执行替换

    # import here to avoid loop
    # （延迟导入避免循环导入；替换表各值规范化为替换列表以支持 launch 表达式）
    from launch.utilities import normalize_to_list_of_substitutions
    self.__source_file = normalize_to_list_of_substitutions(source_file)
    self.__replacements = {}
    for key in replacements:
        self.__replacements[key] = normalize_to_list_of_substitutions(replacements[key])
    self.__condition = condition

  @property
  def name(self) -> List[launch.Substitution]:
    """Getter for name."""
    # 源文件路径的替换列表（launch 内部约定接口）
    return self.__source_file

  @property
  def condition(self) -> Optional[launch.Condition]:
    """Getter for condition."""
    # 替换是否执行的判断条件（可能为 None）
    return self.__condition

  def describe(self) -> Text:
    """Return a description of this substitution as a string."""
    return ''

  def perform(self, context: launch.LaunchContext) -> Text:
    # 核心入口：launch 解析参数时调用
    # 1. 解析出实际文件路径
    yaml_filename = launch.utilities.perform_substitutions(context, self.name)
    # 2. 无条件或条件成立时才执行替换；否则原样返回文件路径
    if self.__condition is None or self.__condition.evaluate(context):
      # 临时文件保存替换结果（delete=False: 关闭后文件保留）
      output_file = tempfile.NamedTemporaryFile(mode='w', delete=False)
      # 3. 在当前上下文解析出最终的替换字符串表
      replacements = self.resolve_replacements(context)
      try:
        # 4. 逐行读入并替换，结果写入临时文件
        input_file = open(yaml_filename, 'r')
        self.replace(input_file, output_file, replacements)
      except Exception as err:  # noqa: B902
        # 出错只打印不抛出，launch 可继续（返回空临时文件路径）
        print('ReplaceString substitution error: ', err)
      finally:
        input_file.close()
        output_file.close()
      # 5. 返回替换后文件的路径
      return output_file.name
    else:
      # 条件不成立: 不做任何替换，直接使用原文件
      return yaml_filename

  def resolve_replacements(self, context):
    # 把替换表中所有 launch 表达式解析成最终的字符串
    resolved_replacements = {}
    for key in self.__replacements:
      resolved_replacements[key] = launch.utilities.perform_substitutions(context, self.__replacements[key])
    return resolved_replacements

  def replace(self, input_file, output_file, replacements):
    # 逐行处理: 对每一行依次应用所有替换规则后写出
    for line in input_file:
      for key, value in replacements.items():
        # 替换的键与值都必须是字符串，否则属于配置错误，直接抛 TypeError
        if isinstance(key, str) and isinstance(value, str):
          if key in line:
            line = line.replace(key, value)
        else:
          raise TypeError('A provided replacement pair is not a string. Both key and value should be strings.')
      output_file.write(line)
