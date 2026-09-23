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
# nav2_simple_commander 的"单点导航"示例程序。
# 模块定位：演示如何使用 BasicNavigator（robot_navigator.py）的
# rclpy Python API 完成一次 /navigate_to_pose 动作导航，包括：
#   设置初始位姿 -> 等待 Nav2 激活 -> goToPose 发送目标 ->
#   轮询任务状态与反馈（ETA/超时取消/目标抢占）-> 获取结果。
# ============================================================================

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
Basic navigation demo to go to pose.
基础导航示例：导航到单个目标位姿。
"""


def main():
    # 初始化 rclpy（Python 版 ROS2 客户端库）
    rclpy.init()

    # 创建导航器：内部创建 rclpy 节点并连接 Nav2 的各动作/服务客户端
    navigator = BasicNavigator()

    # Set our demo's initial pose
    # 设置演示用的初始位姿（必须在激活导航前设置，
    # 否则 AMCL 会以地图原点为初值，导致代价地图被错误读数污染）
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'   # map 坐标系下的位姿
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 3.45
    initial_pose.pose.position.y = 2.15
    initial_pose.pose.orientation.z = 1.0  # 四元数表示 180 度朝向
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # 若 Nav2 未自启动，可调用 lifecycleStartup() 手动触发生命周期管理器启动各节点
    # （必须放在 setInitialPose 之后）；若已自启动则直接等待激活即可。
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    # 阻塞等待 Nav2 完全激活（生命周期管理器就绪 + amcl 等服务可用）
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # 需要的话可以运行时更换地图（调用 map_server 的 load_map 服务）
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # 也可以清空或获取代价地图
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose
    # 构造第一个目标位姿（map 坐标系，朝向 0 度）
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -2.0
    goal_pose.pose.position.y = -0.5
    goal_pose.pose.orientation.w = 1.0

    # sanity check a valid path exists
    # 可选：先请求规划一条路径验证可达性（ComputePathToPose 动作）
    # path = navigator.getPath(initial_pose, goal_pose)

    # 发送 NavigateToPose 动作目标（send_goal_async 的封装）
    navigator.goToPose(goal_pose)

    i = 0
    # 轮询循环：任务未完成期间持续处理反馈
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        # 用户可在此处插入自己的应用逻辑
        #
        ################################################

        # Do something with the feedback
        # 获取动作反馈（estimated_time_remaining 剩余时间、
        # navigation_time 已导航时间等）
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            # 演示超时取消：导航超过 600 秒则取消任务
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            # 演示目标抢占：导航超过 18 秒则修改目标点重新发送
            # （动作服务器接受新目标，旧目标被抢占）
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                goal_pose.pose.position.x = -3.0
                navigator.goToPose(goal_pose)

    # Do something depending on the return code
    # 任务结束：根据结果码输出（SUCCEEDED/CANCELED/FAILED）
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # 关闭导航器（触发 Nav2 生命周期关闭）
    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
