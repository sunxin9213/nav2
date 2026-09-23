#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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
# nav2_simple_commander 的"路径跟随"示例程序。
# 模块定位：演示手动串接 Nav2 各服务器的完整流水线：
#   规划路径（ComputePathToPose）-> 平滑路径（SmoothPath）->
#   跟随路径（FollowPath），绕过 bt_navigator 的自动行为树编排，
#   展示对导航各阶段的精细控制方式。
# ============================================================================

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


"""
Basic navigation demo to follow a given path after smoothing
基础导航示例：规划 -> 平滑 -> 跟随给定路径。
"""


def main():
    # 初始化 rclpy 并创建导航器
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    # 设置演示初始位姿（必须在等待激活前设置）
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

    # Go to our demos first goal pose
    # 构造目标位姿
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -3.0
    goal_pose.pose.position.y = -2.0
    goal_pose.pose.orientation.w = 1.0

    # Get the path, smooth it
    # 第一步：请求 planner_server 规划一条路径
    path = navigator.getPath(initial_pose, goal_pose)
    # 第二步：请求 smoother_server 平滑该路径
    smoothed_path = navigator.smoothPath(path)

    # Follow path
    # 第三步：把平滑后的路径交给 controller_server 执行
    navigator.followPath(smoothed_path)

    i = 0
    # 轮询任务状态并处理反馈
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        # 用户可在此插入自己的应用逻辑
        #
        ################################################

        # Do something with the feedback
        # 反馈内容：距目标剩余距离、当前速度等
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated distance remaining to goal position: ' +
                  '{0:.3f}'.format(feedback.distance_to_goal) +
                  '\nCurrent speed of the robot: ' +
                  '{0:.3f}'.format(feedback.speed))

    # Do something depending on the return code
    # 根据结果码输出
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # 关闭 Nav2 生命周期系统
    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
