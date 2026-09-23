#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
# Copyright 2022 Stevedan Ogochukwu Omodolor
# Copyright 2022 Jaehun Jackson Kim
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
# PyCostmap2D —— 代价地图的 Python3 API。
# 模块定位：把 nav_msgs/OccupancyGrid 消息封装为 numpy 数组，
# 提供 C++ nav2_costmap_2d::Costmap2D 等价的基础操作：
# 尺寸/分辨率/原点元数据访问、按坐标或下标读写代价值、
# 世界坐标 <-> 地图栅格坐标 的双向变换、以及栅格索引计算。
# 配合 FootprintCollisionChecker 与 LineIterator 使用。
# ============================================================================

"""
This is a Python3 API for costmap 2d messages from the stack.

It provides the basic conversion, get/set,
and handling semantics found in the costmap 2d C++ API.
"""

import numpy as np


class PyCostmap2D:
    """
    PyCostmap2D.

    Costmap Python3 API for OccupancyGrids to populate from published messages
    """

    def __init__(self, occupancy_map):
        """
        Initialize costmap2D.

        Args:
        ----
            occupancy_map (OccupancyGrid): 2D OccupancyGrid Map
        """
        # 从 OccupancyGrid 消息中提取地图元数据：
        # 宽/高（格数）、分辨率（米/格）、原点（世界坐标）、
        # 坐标系名与时间戳
        self.size_x = occupancy_map.info.width
        self.size_y = occupancy_map.info.height
        self.resolution = occupancy_map.info.resolution
        self.origin_x = occupancy_map.info.origin.position.x
        self.origin_y = occupancy_map.info.origin.position.y
        self.global_frame_id = occupancy_map.header.frame_id
        self.costmap_timestamp = occupancy_map.header.stamp
        # Extract costmap
        # 代价数据拷贝为一维 numpy uint8 数组（行优先存储）
        self.costmap = np.array(occupancy_map.data, dtype=np.uint8)

    def getSizeInCellsX(self):
        """Get map width in cells. 地图宽度（格数）。"""
        return self.size_x

    def getSizeInCellsY(self):
        """Get map height in cells. 地图高度（格数）。"""
        return self.size_y

    def getSizeInMetersX(self):
        """Get x axis map size in meters. 地图 X 方向尺寸（米）。

        (size-1+0.5)*resolution：最后一个栅格中心的坐标。
        """
        return (self.size_x - 1 + 0.5) * self.resolution

    def getSizeInMetersY(self):
        """Get y axis map size in meters. 地图 Y 方向尺寸（米）。"""
        return (self.size_y - 1 + 0.5) * self.resolution

    def getOriginX(self):
        """Get the origin x axis of the map [m]. 地图原点 X（米）。"""
        return self.origin_x

    def getOriginY(self):
        """Get the origin y axis of the map [m]. 地图原点 Y（米）。"""
        return self.origin_y

    def getResolution(self):
        """Get map resolution [m/cell]. 地图分辨率（米/格）。"""
        return self.resolution

    def getGlobalFrameID(self):
        """Get global frame_id. 地图坐标系名。"""
        return self.global_frame_id

    def getCostmapTimestamp(self):
        """Get costmap timestamp. 地图时间戳。"""
        return self.costmap_timestamp

    def getCostXY(self, mx: int, my: int) -> np.uint8:
        """
        Get the cost of a cell in the costmap using map coordinate XY.

        Args
        ----
            mx (int): map coordinate X to get cost
            my (int): map coordinate Y to get cost

        Returns
        -------
            np.uint8: cost of a cell
        """
        # 按栅格坐标查询代价（内部先转为一维索引）
        return self.costmap[self.getIndex(mx, my)]

    def getCostIdx(self, index: int) -> np.uint8:
        """
        Get the cost of a cell in the costmap using Index.

        Args
        ----
            index (int): index of cell to get cost

        Returns
        -------
            np.uint8: cost of a cell
        """
        # 按一维索引直接查询
        return self.costmap[index]

    def setCost(self, mx: int, my: int, cost: np.uint8) -> None:
        """
        Set the cost of a cell in the costmap using map coordinate XY.

        Args
        ----
            mx (int): map coordinate X to get cost
            my (int): map coordinate Y to get cost
            cost (np.uint8): The cost to set the cell

        Returns
        -------
            None
        """
        # 设置指定栅格的代价值
        self.costmap[self.getIndex(mx, my)] = cost

    def mapToWorld(self, mx: int, my: int) -> tuple[float, float]:
        """
        Get the world coordinate XY using map coordinate XY.

        Args
        ----
            mx (int): map coordinate X to get world coordinate
            my (int): map coordinate Y to get world coordinate

        Returns
        -------
            tuple of float: wx, wy
            wx (float) [m]: world coordinate X
            wy (float) [m]: world coordinate Y
        """
        # 栅格 -> 世界坐标：原点 + (栅格下标 + 0.5)*分辨率
        #（+0.5 是取栅格中心而非左下角）
        wx = self.origin_x + (mx + 0.5) * self.resolution
        wy = self.origin_y + (my + 0.5) * self.resolution
        return (wx, wy)

    def worldToMap(self, wx: float, wy: float) -> tuple[int, int]:
        """
        Get the map coordinate XY using world coordinate XY.

        Args
        ----
            wx (float) [m]: world coordinate X to get map coordinate
            wy (float) [m]: world coordinate Y to get map coordinate

        Returns
        -------
            tuple of int: mx, my
            mx (int): map coordinate X
            my (int): map coordinate Y
        """
        # 世界 -> 栅格坐标：(世界 - 原点) / 分辨率 后向下取整
        mx = int((wx - self.origin_x) // self.resolution)
        my = int((wy - self.origin_y) // self.resolution)
        return (mx, my)

    def getIndex(self, mx: int, my: int) -> int:
        """
        Get the index of the cell using map coordinate XY.

        Args
        ----
            mx (int): map coordinate X to get Index
            my (int): map coordinate Y to get Index

        Returns
        -------
            int: The index of the cell
        """
        # 一维索引 = 行号 * 宽度 + 列号（行优先，与 C++ 一致）
        return my * self.size_x + mx
