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
# FootprintCollisionChecker —— 足迹碰撞检查器的 Python3 API。
# 模块定位：nav2_simple_commander 的几何工具，用于在给定代价地图上
# 检查机器人足迹（多边形轮廓）是否与障碍物碰撞。
# 检测方法：把足迹多边形的每条边用 LineIterator 离散成点列，
# 逐点查询代价并取最大值；任一点为致命障碍值（254）即视为碰撞。
# 支持把足迹按指定位姿（x, y, theta）旋转平移后再检查
# （footprintCostAtPose），常用于验证候选路径点的可行性。
# 与 C++ 版 nav2_costmap_2d::FootprintCollisionChecker 语义一致。
# ============================================================================

"""
This is a Python3 API for a Footprint Collision Checker.

It provides the needed methods to manipulate the coordinates
and calculate the cost of a Footprint
"""

from math import cos, sin

from geometry_msgs.msg import Point32, Polygon
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav2_simple_commander.line_iterator import LineIterator

# 代价地图标准代价值（与 C++ nav2_costmap_2d/cost_values.hpp 一致）
NO_INFORMATION = 255                 # 未知区域
LETHAL_OBSTACLE = 254                # 致命障碍（碰撞）
INSCRIBED_INFLATED_OBSTACLE = 253    # 内切膨胀障碍（必碰）
MAX_NON_OBSTACLE = 252               # 最大非障碍值
FREE_SPACE = 0                       # 自由空间


class FootprintCollisionChecker:
    """
    FootprintCollisionChecker.

    FootprintCollisionChecker Class for getting the cost
    and checking the collisions of a Footprint
    足迹碰撞检查器：计算足迹在代价地图上的代价并检测碰撞
    """

    def __init__(self):
        """Initialize the FootprintCollisionChecker Object."""
        # 代价地图（须先调用 setCostmap 设置）
        self.costmap_ = None
        pass

    def footprintCost(self, footprint: Polygon):
        """
        Iterate over all the points in a footprint and check for collision.

        Args
        ----
            footprint (Polygon): The footprint to calculate the collision cost for

        Returns
        -------
            LETHAL_OBSTACLE (int): If collision was found, 254 will be returned
            footprint_cost (float): The maximum cost found in the footprint points
        """
        # 遍历足迹多边形的所有边，取各边线代价的最大值：
        # 任一边上有致命障碍 => 直接返回 254（碰撞）
        footprint_cost = 0.0
        x1 = 0.0
        y1 = 0.0

        # 第一个顶点转换为地图坐标（越界则视为碰撞）
        x0, y0 = self.worldToMapValidated(footprint.points[0].x, footprint.points[0].y)

        if x0 is None or y0 is None:
            return LETHAL_OBSTACLE

        # 记录起点顶点（闭合边最后处理）
        xstart = x0
        ystart = y0

        # 依次检查相邻顶点构成的每条边
        for i in range(len(footprint.points) - 1):
            x1, y1 = self.worldToMapValidated(
                footprint.points[i + 1].x, footprint.points[i + 1].y
            )

            if x1 is None or y1 is None:
                return LETHAL_OBSTACLE

            # 边代价与当前最大值比较
            footprint_cost = max(float(self.lineCost(x0, x1, y0, y1)), footprint_cost)
            x0 = x1
            y0 = y1

            if footprint_cost == LETHAL_OBSTACLE:
                return footprint_cost

        # 最后检查闭合边（末顶点 -> 首顶点）
        return max(float(self.lineCost(xstart, x1, ystart, y1)), footprint_cost)

    def lineCost(self, x0, x1, y0, y1, step_size=0.5):
        """
        Iterate over all the points along a line and check for collision.

        Args
        ----
            x0 (float): Abscissa of the initial point in map coordinates
            y0 (float): Ordinate of the initial point in map coordinates
            x1 (float): Abscissa of the final point in map coordinates
            y1 (float): Ordinate of the final point in map coordinates
            step_size (float): Optional, Increments' resolution, defaults to 0.5

        Returns
        -------
            LETHAL_OBSTACLE (int): If collision was found, 254 will be returned
            line_cost (float): The maximum cost found in the line points
        """
        # 用线迭代器沿边采样，逐点查询代价并取最大值
        line_cost = 0.0
        point_cost = -1.0
        line_iterator = LineIterator(x0, y0, x1, y1, step_size)

        while line_iterator.isValid():
            # 当前采样点的代价
            point_cost = self.pointCost(
                int(line_iterator.getX()), int(line_iterator.getY())
            )

            if point_cost == LETHAL_OBSTACLE:
                return point_cost

            if line_cost < point_cost:
                line_cost = point_cost

            line_iterator.advance()

        return line_cost

    def worldToMapValidated(self, wx: float, wy: float):
        """
        Get the map coordinate XY using world coordinate XY.

        Args
        ----
            wx (float): world coordinate X
            wy (float): world coordinate Y

        Returns
        -------
            None: if coordinates are invalid
            tuple of int: mx, my (if coordinates are valid)
            mx (int): map coordinate X
            my (int): map coordinate Y
        """
        # 世界坐标 -> 地图栅格坐标（未设置代价地图时抛异常）
        if self.costmap_ is None:
            raise ValueError(
                'Costmap not specified, use setCostmap to specify the costmap first'
            )
        return self.costmap_.worldToMap(wx, wy)

    def pointCost(self, x: int, y: int):
        """
        Get the cost of a point in the costmap using map coordinates XY.

        Args
        ----
            mx (int): map coordinate X
            my (int): map coordinate Y

        Returns
        -------
            np.uint8: cost of a point
        """
        # 查询指定栅格的代价值
        if self.costmap_ is None:
            raise ValueError(
                'Costmap not specified, use setCostmap to specify the costmap first'
            )
        return self.costmap_.getCostXY(x, y)

    def setCostmap(self, costmap: PyCostmap2D):
        """
        Specify which costmap to use.

        Args
        ----
            costmap (PyCostmap2D): costmap to use in the object's methods

        Returns
        -------
            None
        """
        # 设置后续检查所用的代价地图
        self.costmap_ = costmap
        return None

    def footprintCostAtPose(self, x: float, y: float, theta: float, footprint: Polygon):
        """
        Get the cost of a footprint at a specific Pose in map coordinates.

        Args
        ----
            x (float): map coordinate X
            y (float): map coordinate Y
            theta (float): absolute rotation angle of the footprint
            footprint (Polygon): the footprint to calculate its cost at the given Pose

        Returns
        -------
            LETHAL_OBSTACLE (int): If collision was found, 254 will be returned
            footprint_cost (float): The maximum cost found in the footprint points
        """
        # 把足迹顶点按刚体变换（旋转 theta + 平移到 (x,y)）：
        #   x' = x + (px*cos - py*sin), y' = y + (px*sin + py*cos)
        cos_th = cos(theta)
        sin_th = sin(theta)
        oriented_footprint = Polygon()

        for i in range(len(footprint.points)):
            new_pt = Point32()
            new_pt.x = x + (
                footprint.points[i].x * cos_th - footprint.points[i].y * sin_th
            )
            new_pt.y = y + (
                footprint.points[i].x * sin_th + footprint.points[i].y * cos_th
            )
            oriented_footprint.points.append(new_pt)

        # 检查变换后足迹的碰撞代价
        return self.footprintCost(oriented_footprint)
