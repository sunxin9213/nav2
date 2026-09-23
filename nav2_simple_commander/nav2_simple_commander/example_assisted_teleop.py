#! /usr/bin/env python3
# Copyright 2022 Joshua Wallace
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
# nav2_simple_commander 的"辅助遥控"示例程序。
# 模块定位：演示使用 /assisted_teleop 动作（nav2_behaviors）：
# 在人工遥控（cmd_vel）期间由 Nav2 自动做碰撞过滤，
# 使操作者的指令被安全约束修正后转发给底盘。
# 演示期间仅空转轮询任务状态（实际使用时遥控指令由遥杆等发布）。
# ============================================================================

from time import sleep

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

"""
Basic navigation demo to go to pose.
辅助遥控示例：启动 assisted_teleop 并在任务期间保持轮询。
"""


def main():
    # 初始化 rclpy 并创建导航器
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    # 设置演示初始位姿
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 3.45
    initial_pose.pose.position.y = 2.15
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    # 阻塞等待 Nav2 完全激活
    navigator.waitUntilNav2Active()

    # 启动辅助遥控（时间预算 20 秒）
    navigator.assistedTeleop(time_allowance=20)
    while not navigator.isTaskComplete():
        # Publish twist commands to be filtered by the assisted teleop action
        # 实际使用时在此期间发布的 cmd_vel 会被 assisted_teleop
        # 做碰撞过滤后转发；演示中仅休眠等待
        sleep(0.2)
        pass

    # 关闭 Nav2 生命周期系统
    navigator.lifecycleShutdown()
    exit(0)


if __name__ == '__main__':
    main()

