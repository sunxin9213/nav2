#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
# Copyright 2022 Afif Swaidan
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
# 文件说明（中文）：
# LineIterator（直线迭代器）—— Python3 API。
# 模块定位：nav2_simple_commander 的几何工具类，用于沿给定线段
# 按固定步长逐点迭代（类似 C++ costmap_2d 的 RayTracer）。
# 被 footprint_collision_checker 用于检查两点连线上的代价地图值，
# 实现足迹边缘的碰撞检测。
# 支持一般斜线、垂直线与水平线三种情形。
# ============================================================================

"""
This is a Python3 API for a line iterator.

It provides the ability to iterate
through the points of a line.
线迭代器：提供沿直线逐点迭代的能力。
"""

from cmath import sqrt


class LineIterator():
    """
    LineIterator.

    LineIterator Python3 API for iterating along the points of a given line
    线迭代器：沿给定线段的点序列进行迭代
    """

    def __init__(self, x0, y0, x1, y1, step_size=1.0):
        """
        Initialize the LineIterator.

        Args
        ----
            x0 (float): Abscissa of the initial point   起点横坐标
            y0 (float): Ordinate of the initial point   起点纵坐标
            x1 (float): Abscissa of the final point     终点横坐标
            y1 (float): Ordinate of the final point     终点纵坐标
            step_size (float): Optional, Increments' resolution, defaults to 1
                               步长（迭代分辨率），默认 1

        Raises
        ------
            TypeError: When one (or more) of the inputs is not a number
                       输入不是数字时抛出
            ValueError: When step_size is not a positive number
                        步长非正数或线段长度为 0 时抛出
        """
        # 输入类型检查：起点/终点坐标与步长必须是 int 或 float
        if type(x0) not in [int, float]:
            raise TypeError("x0 must be a number (int or float)")

        if type(y0) not in [int, float]:
            raise TypeError("y0 must be a number (int or float)")

        if type(x1) not in [int, float]:
            raise TypeError("x1 must be a number (int or float)")

        if type(y1) not in [int, float]:
            raise TypeError("y1 must be a number (int or float)")

        if type(step_size) not in [int, float]:
            raise TypeError("step_size must be a number (int or float)")

        # 步长必须为正数
        if step_size <= 0:
            raise ValueError("step_size must be a positive number")

        # 保存起点/终点坐标与当前迭代点（初始位于起点）
        self.x0_ = x0
        self.y0_ = y0
        self.x1_ = x1
        self.y1_ = y1
        self.x_ = x0
        self.y_ = y0
        self.step_size_ = step_size

        # 按线段类型初始化：
        #  - 一般斜线：计算斜率 m 与截距 b（y = m*x + b）
        #  - 垂直线（x 相同）：只沿 y 迭代
        #  - 水平线（y 相同）：只沿 x 迭代
        #  - 零长度线（起终点重合）：非法
        if x1 != x0 and y1 != y0:
            self.valid_ = True
            self.m_ = (y1-y0)/(x1-x0)
            self.b_ = y1 - (self.m_*x1)
        elif x1 == x0 and y1 != y0:
            self.valid_ = True
        elif y1 == y1 and x1 != x0:
            self.valid_ = True
            self.m_ = (y1-y0)/(x1-x0)
            self.b_ = y1 - (self.m_*x1)
        else:
            self.valid_ = False
            raise ValueError(
                "Line has zero length (All 4 points have same coordinates)")

    def isValid(self):
        """Check if line is valid. 检查迭代器是否仍有效（未越过终点）"""
        return self.valid_

    def advance(self):
        """Advance to the next point in the line. 前进到线段上的下一个点。"""
        # 前进方向按 x 优先；x 相同（垂直线）时按 y 前进。
        # 每次移动一个步长，clamp 防止越过终点；斜线时 y 由直线方程求出。
        if self.x1_ > self.x0_:
            # 沿 x 正方向前进
            if self.x_ < self.x1_:
                self.x_ = round(self.clamp(
                    self.x_ + self.step_size_, self.x0_, self.x1_), 5)
                self.y_ = round(self.m_ * self.x_ + self.b_, 5)
            else:
                self.valid_ = False   # 已到达/越过终点：迭代器失效
        elif self.x1_ < self.x0_:
            # 沿 x 负方向前进
            if self.x_ > self.x1_:
                self.x_ = round(self.clamp(
                    self.x_ - self.step_size_, self.x1_, self.x0_), 5)
                self.y_ = round(self.m_ * self.x_ + self.b_, 5)
            else:
                self.valid_ = False
        else:
            # 垂直线：沿 y 方向前进
            if self.y1_ > self.y0_:
                if self.y_ < self.y1_:
                    self.y_ = round(self.clamp(
                        self.y_ + self.step_size_, self.y0_, self.y1_), 5)
                else:
                    self.valid_ = False
            elif self.y1_ < self.y0_:
                if self.y_ > self.y1_:
                    self.y_ = round(self.clamp(
                        self.y_ - self.step_size_, self.y1_, self.y0_), 5)
                else:
                    self.valid_ = False
            else:
                self.valid_ = False

    def getX(self):
        """Get the abscissa of the current point. 当前点横坐标"""
        return self.x_

    def getY(self):
        """Get the ordinate of the current point. 当前点纵坐标"""
        return self.y_

    def getX0(self):
        """Get the abscissa of the initial point. 起点横坐标"""
        return self.x0_

    def getY0(self):
        """Get the ordinate of the intial point. 起点纵坐标"""
        return self.y0_

    def getX1(self):
        """Get the abscissa of the final point. 终点横坐标"""
        return self.x1_

    def getY1(self):
        """Get the ordinate of the final point. 终点纵坐标"""
        return self.y1_

    def get_line_length(self):
        """Get the length of the line. 线段长度（欧氏距离）"""
        return sqrt(pow(self.x1_ - self.x0_, 2) + pow(self.y1_ - self.y0_, 2))

    def clamp(self, n, min_n, max_n):
        """
        Clamp n to be between min_n and max_n.
        将 n 限制在 [min_n, max_n] 区间内。

        Args
        ----
            n (float): input value             输入值
            min_n (float): minimum value        下界
            max_n (float): maximum value        上界

        Returns
        -------
            n (float): input value clamped between given min and max
                       截断后的值
        """
        if n < min_n:
            return min_n
        elif n > max_n:
            return max_n
        else:
            return n
