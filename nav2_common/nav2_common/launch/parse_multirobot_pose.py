# Copyright (c) 2023 LG Electronics.
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
# 【文件概况】多机器人位姿命令行参数解析工具 ParseMultiRobotPose
# ----------------------------------------------------------------------------
# 概述: 直接从 sys.argv 中解析 launch 命令行传入的多机器人位姿字符串，
#       转成 {机器人名: {x, y, z, roll, pitch, yaw}} 字典，缺省字段补 0。
#       注意它不是 launch.Substitution，而是在 launch 文件顶层直接构造使用。
# 命令行格式示例:
#   ros2 launch nav2_bringup bringup_multirobot_launch.py
#     robots:="robot1={x: 1.0, y: 1.0, yaw: 0.0};
#              robot2={x: 1.0, y: 1.0, z: 1.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707}"
#   - 多台机器人之间用 ';' 分隔
#   - 机器人名与位姿之间用 '=' 分隔（机器人名对应其命名空间）
#   - 位姿为 YAML 内联字典格式
# 主要内容:
#   1. __parse_argument(): 从 sys.argv[4:] 中查找 "目标参数:=" 前缀并取值
#   2. value(): 拆分各机器人、解析位姿并补全六个缺省字段
# 使用场景: nav2_bringup 的 bringup_multirobot_launch.py 解析 robots 参数，
#           为每台机器人生成初始位姿（供 spawn 使用）与命名空间。
# ============================================================================

import yaml
import sys
from typing import Text, Dict


class ParseMultiRobotPose():
    """
    Parsing argument using sys module
    """

    def __init__(self, target_argument: Text):
        """
        Parse arguments for multi-robot's pose

        for example,
        `ros2 launch nav2_bringup bringup_multirobot_launch.py
            robots:="robot1={x: 1.0, y: 1.0, yaw: 0.0};
                     robot2={x: 1.0, y: 1.0, z: 1.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707}"`

        `target_argument` shall be 'robots'.
        Then, this will parse a string value for `robots` argument.

        Each robot name which is corresponding to namespace and pose of it will be separted by `;`.
        The pose consists of x, y and yaw with YAML format.

        :param: target argument name to parse
        """
        # 构造时立即从命令行解析目标参数的原始字符串值并缓存
        # （中文补充: target_argument 即 launch 参数名，如 'robots'）
        self.__args: Text = self.__parse_argument(target_argument)

    def __parse_argument(self, target_argument: Text) -> Text:
        """
        get value of target argument
        """
        # 跳过前 4 个固定参数（ros2/launch/包名/launch文件名），
        # 在其后的 argv 中查找 "目标参数:=" 前缀，返回去掉前缀后的值；
        # 未找到时返回空字符串
        if len(sys.argv) > 4:
            argv = sys.argv[4:]
            for arg in argv:
                if arg.startswith(target_argument + ":="):
                    return arg.replace(target_argument + ":=", "")
        return ""

    def value(self) -> Dict:
        """
        get value of target argument
        """
        # 解析缓存的原始字符串 → 多机器人位姿字典
        args = self.__args
        # 按 ';' 拆出各机器人的 "名字={位姿}" 片段
        parsed_args = list() if len(args) == 0 else args.split(';')
        multirobots = dict()
        for arg in parsed_args:
            # 按 '=' 拆成 名字 / 位姿 两段（拆不出两段的片段直接跳过）
            key_val = arg.strip().split('=')
            if len(key_val) != 2:
                continue
            key = key_val[0].strip()
            val = key_val[1].strip()
            # 位姿部分本身是 YAML 内联字典，直接 safe_load 解析
            robot_pose = yaml.safe_load(val)
            # 六个位姿字段缺省一律补 0.0（未指定的自由度不影响 spawn）
            if 'x' not in robot_pose:
                robot_pose['x'] = 0.0
            if 'y' not in robot_pose:
                robot_pose['y'] = 0.0
            if 'z' not in robot_pose:
                robot_pose['z'] = 0.0
            if 'roll' not in robot_pose:
                robot_pose['roll'] = 0.0
            if 'pitch' not in robot_pose:
                robot_pose['pitch'] = 0.0
            if 'yaw' not in robot_pose:
                robot_pose['yaw'] = 0.0
            multirobots[key] = robot_pose
        return multirobots
