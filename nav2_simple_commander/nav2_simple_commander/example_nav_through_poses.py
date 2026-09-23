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
# nav2_simple_commander 的"多途经点导航"示例程序。
# 模块定位：演示使用 /navigate_through_poses 动作依次经过多个位姿：
# 机器人会按顺序访问所有目标点，最终停在最后一个点。
# 与 FollowWaypoints 的区别：NavigateThroughPoses 在每个途经点
# 不做精确定位停止，而是连续通过，仅终点停靠。
# ============================================================================

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
Basic navigation demo to go to poses.
基础导航示例：依次导航经过多个目标位姿。
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

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # 未自启动时可手动 lifecycleStartup()（须在 setInitialPose 之后）
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    # 阻塞等待 Nav2 完全激活
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # 可选：运行时更换地图
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # 可选：清空或获取代价地图
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # set our demo's goal poses
    # 构造多个目标位姿（途经点序列）
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 1.5
    goal_pose1.pose.position.y = 0.55
    goal_pose1.pose.orientation.w = 0.707  # 四元数表示 90 度朝向
    goal_pose1.pose.orientation.z = 0.707
    goal_poses.append(goal_pose1)

    # additional goals can be appended
    # 追加更多途经点
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 1.5
    goal_pose2.pose.position.y = -3.75
    goal_pose2.pose.orientation.w = 0.707
    goal_pose2.pose.orientation.z = 0.707
    goal_poses.append(goal_pose2)
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -3.6
    goal_pose3.pose.position.y = -4.75
    goal_pose3.pose.orientation.w = 0.707
    goal_pose3.pose.orientation.z = 0.707
    goal_poses.append(goal_pose3)

    # sanity check a valid path exists
    # 可选：先规划验证路径可达
    # path = navigator.getPathThroughPoses(initial_pose, goal_poses)

    # 发送 NavigateThroughPoses 动作目标
    navigator.goThroughPoses(goal_poses)

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
        # 反馈：预计剩余到达时间等
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
            # 演示目标抢占：导航超过 35 秒则改发新的目标列表（旧任务被抢占）
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=35.0):
                goal_pose4 = PoseStamped()
                goal_pose4.header.frame_id = 'map'
                goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose4.pose.position.x = -5.0
                goal_pose4.pose.position.y = -4.75
                goal_pose4.pose.orientation.w = 0.707
                goal_pose4.pose.orientation.z = 0.707
                navigator.goThroughPoses([goal_pose4])

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
