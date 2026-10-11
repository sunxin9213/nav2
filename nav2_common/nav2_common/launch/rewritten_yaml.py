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
# 【文件概况】launch 参数文件动态改写工具 RewrittenYaml
# ----------------------------------------------------------------------------
# 概述: 实现 launch.Substitution 接口 —— 在 launch 执行期读取一个 YAML
#       参数文件，按字典替换其中的"叶子参数值"或"键名"，再把结果写入
#       临时文件并返回其路径，供节点作为 params 文件使用。
# 主要内容:
#   1. RewrittenYaml: 核心替换类，支持三种改写方式
#      - param_rewrites: 替换参数值。键可用两种形式:
#          a) "叶子键名"  —— 全文件同名的叶子参数都会被替换
#          b) "全路径键名" —— 精确匹配 'key.keyA.keyB.val' 形式的嵌套路径
#      - key_rewrites:   替换键名本身（如适配参数更名迁移）
#      - root_key:       把整个文件内容重新包到指定根键下
#                        （多机器人场景按命名空间隔离参数的关键手段）
#      - convert_types:  是否尝试把替换值的字符串转成 int/float/bool
#   2. DictItemReference: 指向 dict 中某键的"引用"小对象，
#      用于遍历时能原地修改叶子键的值
# 使用场景: nav2_bringup 的 launch 文件大量使用，典型如按 use_sim_time、
#           namespace、autostart 等 launch 参数为每台机器人动态生成参数文件。
# ============================================================================

from typing import Dict
from typing import List
from typing import Text
from typing import Optional

import yaml
import tempfile
import launch


class DictItemReference:
    # "字典条目引用"：保存字典对象与其中的一个键，
    # 使遍历方拿到本对象后仍能通过 setValue() 原地写回该键的值
    def __init__(self, dictionary, key):
        self.dictionary = dictionary
        self.dictKey = key

    def key(self):
        # 返回被引用的键名
        return self.dictKey

    def setValue(self, value):
        # 原地更新该键的值
        self.dictionary[self.dictKey] = value


class RewrittenYaml(launch.Substitution):
    """
    Substitution that modifies the given YAML file.

    Used in launch system
    """

    def __init__(self,
        source_file: launch.SomeSubstitutionsType,
        param_rewrites: Dict,
        root_key: Optional[launch.SomeSubstitutionsType] = None,
        key_rewrites: Optional[Dict] = None,
        convert_types = False) -> None:
        super().__init__()
        """
        Construct the substitution

        :param: source_file the original YAML file to modify
        :param: param_rewrites mappings to replace
        :param: root_key if provided, the contents are placed under this key
        :param: key_rewrites keys of mappings to replace
        :param: convert_types whether to attempt converting the string to a number or boolean
        """
        # 参数含义（中文补充）:
        #   source_file:   原始 YAML 参数文件路径（可含 launch 替换表达式）
        #   param_rewrites: {键: 新值} 的参数值替换表
        #   root_key:      指定时把改写后的全部内容放到该根键下
        #   key_rewrites:  {旧键名: 新键名} 的键名替换表
        #   convert_types: True 时尝试把替换值字符串转为数值/布尔

        # import here to avoid loop
        # （延迟导入 launch.utilities 以避免循环导入）
        from launch.utilities import normalize_to_list_of_substitutions
        # 所有输入都先规范化为"替换列表"，允许键值里使用 launch 表达式
        self.__source_file = normalize_to_list_of_substitutions(source_file)
        self.__param_rewrites = {}
        self.__key_rewrites = {}
        self.__convert_types = convert_types
        self.__root_key = None
        for key in param_rewrites:
            self.__param_rewrites[key] = normalize_to_list_of_substitutions(param_rewrites[key])
        if key_rewrites is not None:
            for key in key_rewrites:
                self.__key_rewrites[key] = normalize_to_list_of_substitutions(key_rewrites[key])
        if root_key is not None:
            self.__root_key = normalize_to_list_of_substitutions(root_key)

    @property
    def name(self) -> List[launch.Substitution]:
        """Getter for name."""
        # 源文件路径的替换列表（launch 内部约定接口）
        return self.__source_file

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return ''

    def perform(self, context: launch.LaunchContext) -> Text:
        # 核心入口：launch 在解析参数时调用本方法
        # 1. 先把替换列表解析成实际文件路径字符串
        yaml_filename = launch.utilities.perform_substitutions(context, self.name)
        # 2. 创建临时文件保存改写结果（delete=False: 关闭后文件仍保留）
        rewritten_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False)
        # 3. 在当前 launch 上下文中把所有替换表解析成最终字符串
        param_rewrites, keys_rewrites = self.resolve_rewrites(context)
        # 4. 加载原 YAML → 值替换 → 键名替换
        data = yaml.safe_load(open(yaml_filename, 'r'))
        self.substitute_params(data, param_rewrites)
        self.substitute_keys(data, keys_rewrites)
        # 5. 若指定 root_key，把全部内容包到该键下（如 namespace）
        if self.__root_key is not None:
            root_key = launch.utilities.perform_substitutions(context, self.__root_key)
            if root_key:
                data = {root_key: data}
        # 6. 结果写入临时文件并返回路径，节点即用该文件作为参数文件
        yaml.dump(data, rewritten_yaml)
        rewritten_yaml.close()
        return rewritten_yaml.name

    def resolve_rewrites(self, context):
        # 把参数替换表和键名替换表中的 launch 表达式
        # 在当前上下文中解析成最终的字符串值
        resolved_params = {}
        for key in self.__param_rewrites:
            resolved_params[key] = launch.utilities.perform_substitutions(context, self.__param_rewrites[key])
        resolved_keys = {}
        for key in self.__key_rewrites:
            resolved_keys[key] = launch.utilities.perform_substitutions(context, self.__key_rewrites[key])
        return resolved_params, resolved_keys

    def substitute_params(self, yaml, param_rewrites):
        # substitute leaf-only parameters
        # 方式一（叶子键名匹配）: 遍历全部叶子键，凡名称在替换表中的一律替换
        for key in self.getYamlLeafKeys(yaml):
            if key.key() in param_rewrites:
                raw_value = param_rewrites[key.key()]
                # 通过引用对象原地写回新值（可能做类型转换）
                key.setValue(self.convert(raw_value))

        # substitute total path parameters
        # 方式二（全路径匹配）: 先把整棵 YAML 树压平成 "a.b.c.val" 路径表，
        # 路径本身出现在替换表中的（说明是嵌套路径精确指定），逐个回写
        yaml_paths = self.pathify(yaml)
        for path in yaml_paths:
            if path in param_rewrites:
                # this is an absolute path (ex. 'key.keyA.keyB.val')
                rewrite_val = self.convert(param_rewrites[path])
                # 把路径字符串拆成键序列，沿路径递归下去更新末级值
                yaml_keys = path.split('.')
                yaml = self.updateYamlPathVals(yaml, yaml_keys, rewrite_val)


    def updateYamlPathVals(self, yaml, yaml_key_list, rewrite_val):
        # 沿键序列逐级下钻，直到最后一级时写入新值；支持 list 下标
        for key in yaml_key_list:
            if key == yaml_key_list[-1]:
                # 已到路径末级: 直接赋值
                yaml[key] = rewrite_val
                break
            # 弹出当前层级的键，继续下钻
            key = yaml_key_list.pop(0)
            if isinstance(yaml, list):
                # 列表层级: 键被解释为整数下标
                yaml[int(key)] = self.updateYamlPathVals(yaml[int(key)], yaml_key_list, rewrite_val)
            else:
                # 字典层级: 缺失的中间键用空字典补齐
                yaml[key] = self.updateYamlPathVals(yaml.get(key, {}), yaml_key_list, rewrite_val)
        return yaml

    def substitute_keys(self, yaml, key_rewrites):
        # 键名替换: 递归遍历，凡键名在 key_rewrites 表中的
        # 用新键名承接旧值并删除旧键（值保持不变）
        if len(key_rewrites) != 0:
            for key in list(yaml.keys()):
                val = yaml[key]
                if key in key_rewrites:
                    new_key = key_rewrites[key]
                    yaml[new_key] = yaml[key]
                    del yaml[key]
                # 值若还是 dict 则继续向下递归替换
                if isinstance(val, dict):
                    self.substitute_keys(val, key_rewrites)

    def getYamlLeafKeys(self, yamlData):
        # 生成器: 深度优先遍历，逐个产出所有"叶子键"的引用对象
        # （即值不再是 dict 的那些键）；到达非 dict 数据时自然终止
        try:
            for key in yamlData.keys():
                # 先递归产出子层级（若存在）的叶子键
                for k in self.getYamlLeafKeys(yamlData[key]):
                    yield k
                # 再产出当前层级的键引用
                yield DictItemReference(yamlData, key)
        except AttributeError:
            # yamlData 不是 dict（已到叶子/标量），结束本层递归
            return

    def pathify(self, d, p=None, paths=None, joinchar='.'):
        # 把嵌套 dict/list 压平成 {"路径字符串": 叶子值} 的映射:
        #   dict 的键、list 的下标都会拼进路径，层级间用 joinchar('.') 连接
        # 递归入口: 首次调用时 p/paths 均为 None，初始化后以 p="" 重入
        if p is None:
            paths = {}
            self.pathify(d, "", paths, joinchar=joinchar)
            return paths
        pn = p
        if p != "":
            pn += joinchar
        if isinstance(d, dict):
            # dict: 逐键下钻，键名拼入路径
            for k in d:
                v = d[k]
                self.pathify(v, str(pn) + str(k), paths, joinchar=joinchar)
        elif isinstance(d, list):
            # list: 用元素下标拼入路径
            for idx, e in enumerate(d):
                self.pathify(e, pn + str(idx), paths, joinchar=joinchar)
        else:
            # 叶子: 记录 "完整路径 -> 值"
            paths[p] = d

    def convert(self, text_value):
        # 替换值的类型转换:
        # 1) convert_types 开启时尝试转 int/float（含 '.' 转 float 否则 int）
        if self.__convert_types:
            # try converting to int or float
            try:
                return float(text_value) if '.' in text_value else int(text_value)
            except ValueError:
                pass

        # 2) 尝试转 bool（"true"/"false" 不区分大小写）
        if text_value.lower() == "true":
            return True
        if text_value.lower() == "false":
            return False

        # 3) 均失败则按原字符串返回
        return text_value
