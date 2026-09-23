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
# BasicNavigator —— Nav2 的 Python3 (rclpy) 简易导航 API。
# 模块定位：nav2_simple_commander 的核心类，把 Nav2 各服务器
# （导航/规划/平滑/恢复/代价地图/地图服务）的 ROS2 接口
# 封装成简洁的同步函数，供上层应用（示例脚本、演示程序）调用。
#
# 封装的 ROS2 接口一览：
#  动作客户端（ActionClient + send_goal_async + spin_until_future_complete）：
#   navigate_to_pose        单点导航（bt_navigator）
#   navigate_through_poses  多途经点导航
#   follow_waypoints        顺序跟随航点
#   follow_path             执行指定路径（controller_server）
#   compute_path_to_pose    规划到单点的路径（planner_server）
#   compute_path_through_poses 规划经过多点的路径
#   smooth_path             路径平滑（smoother_server）
#   spin / backup / assisted_teleop 恢复行为（behavior_server）
#  服务客户端（create_client + call_async）：
#   map_server/load_map                     更换地图
#   global/local_costmap/clear_entirely_*   清空代价地图
#   global/local_costmap/get_costmap        获取代价地图
#   ManageLifecycleNodes                    生命周期启停
#  话题：
#   amcl_pose（订阅，transient_local QoS）定位位姿
#   initialpose（发布）设置初始位姿
# ============================================================================


from enum import Enum
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import AssistedTeleop, BackUp, Spin
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from nav2_msgs.action import FollowPath, FollowWaypoints, NavigateThroughPoses, NavigateToPose
from nav2_msgs.action import SmoothPath
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class TaskResult(Enum):
    """任务结果枚举：未知/成功/取消/失败（getResult 的返回值）"""
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class BasicNavigator(Node):
    """Nav2 简易导航 API 封装类（继承 rclpy.node.Node）"""

    def __init__(self, node_name='basic_navigator', namespace=''):
        # 调用父类构造创建 ROS2 节点
        super().__init__(node_name=node_name, namespace=namespace)
        # 初始位姿（默认 map 坐标系原点）
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.goal_handle = None      # 当前动作目标句柄
        self.result_future = None   # 当前动作结果 future
        self.feedback = None        # 最近一帧动作反馈
        self.status = None          # 动作最终状态码

        # amcl 位姿话题的 QoS：transient_local + reliable + keep_last(1)。
        # AMCL 的位姿以 latched 方式发布（迟加入的订阅者也能收到
        # 最后一帧），订阅端 QoS 必须匹配才能建立连接
        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        self.initial_pose_received = False
        # ---- 创建各动作客户端 ----
        self.nav_through_poses_client = ActionClient(self,
                                                     NavigateThroughPoses,
                                                     'navigate_through_poses')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self.compute_path_to_pose_client = ActionClient(self, ComputePathToPose,
                                                        'compute_path_to_pose')
        self.compute_path_through_poses_client = ActionClient(self, ComputePathThroughPoses,
                                                              'compute_path_through_poses')
        self.smoother_client = ActionClient(self, SmoothPath, 'smooth_path')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.backup_client = ActionClient(self, BackUp, 'backup')
        self.assisted_teleop_client = ActionClient(self, AssistedTeleop, 'assisted_teleop')
        # ---- 订阅/发布与服务 ----
        # 订阅 AMCL 定位位姿（用于确认初始位姿已被接收）
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self._amclPoseCallback,
                                                              amcl_pose_qos)
        # 发布初始位姿（RViz "2D Pose Estimate" 等价话题）
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)
        # 服务客户端：更换地图 / 清空与获取代价地图
        self.change_maps_srv = self.create_client(LoadMap, 'map_server/load_map')
        self.clear_costmap_global_srv = self.create_client(
            ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap')
        self.clear_costmap_local_srv = self.create_client(
            ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap')
        self.get_costmap_global_srv = self.create_client(GetCostmap, 'global_costmap/get_costmap')
        self.get_costmap_local_srv = self.create_client(GetCostmap, 'local_costmap/get_costmap')

    def destroyNode(self):
        """兼容旧 API 的节点销毁包装"""
        self.destroy_node()

    def destroy_node(self):
        """先销毁全部动作客户端，再销毁节点"""
        self.nav_through_poses_client.destroy()
        self.nav_to_pose_client.destroy()
        self.follow_waypoints_client.destroy()
        self.follow_path_client.destroy()
        self.compute_path_to_pose_client.destroy()
        self.compute_path_through_poses_client.destroy()
        self.smoother_client.destroy()
        self.spin_client.destroy()
        self.backup_client.destroy()
        super().destroy_node()

    def setInitialPose(self, initial_pose):
        """Set the initial pose to the localization system. 设置定位系统的初始位姿。"""
        # 重置接收标志后发布初始位姿（等待 AMCL 回发 amcl_pose 确认）
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()

    def goThroughPoses(self, poses, behavior_tree=''):
        """Send a `NavThroughPoses` action request. 发送 NavigateThroughPoses 动作请求。"""
        # 阻塞等待动作服务器上线（wait_for_server 轮询）
        self.debug("Waiting for 'NavigateThroughPoses' action server")
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateThroughPoses' action server not available, waiting...")

        # 组装目标：途经点位姿列表 + 可选自定义行为树
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        goal_msg.behavior_tree = behavior_tree

        self.info(f'Navigating with {len(goal_msg.poses)} goals....')
        # 异步发送目标（附带反馈回调），同步等待目标被接受
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg,
                                                                         self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(f'Goal with {len(poses)} poses was rejected!')
            return False

        # 保存结果 future 供 isTaskComplete 轮询
        self.result_future = self.goal_handle.get_result_async()
        return True

    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request. 发送 NavigateToPose 动作请求（单点导航）。"""
        # 等待 bt_navigator 的 navigate_to_pose 动作服务器
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        # 发送目标并等待接受
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def followWaypoints(self, poses):
        """Send a `FollowWaypoints` action request. 发送 FollowWaypoints 动作请求。"""
        # 等待 waypoint_follower 的动作服务器
        self.debug("Waiting for 'FollowWaypoints' action server")
        while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowWaypoints' action server not available, waiting...")

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.info(f'Following {len(goal_msg.poses)} goals....')
        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg,
                                                                        self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(f'Following {len(poses)} waypoints request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def spin(self, spin_dist=1.57, time_allowance=10):
        """发送 Spin 旋转恢复动作请求（默认转 90 度）。"""
        # 等待 behavior_server 的 spin 动作
        self.debug("Waiting for 'Spin' action server")
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist                # 目标旋转角（弧度）
        goal_msg.time_allowance = Duration(sec=time_allowance)  # 时间预算

        self.info(f'Spinning to angle {goal_msg.target_yaw}....')
        send_goal_future = self.spin_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Spin request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def backup(self, backup_dist=0.15, backup_speed=0.025, time_allowance=10):
        """发送 BackUp 倒车恢复动作请求（默认后退 0.15m）。"""
        # 等待 behavior_server 的 backup 动作
        self.debug("Waiting for 'Backup' action server")
        while not self.backup_client.wait_for_server(timeout_sec=1.0):
            self.info("'Backup' action server not available, waiting...")
        goal_msg = BackUp.Goal()
        goal_msg.target = Point(x=float(backup_dist))  # 后退距离（米）
        goal_msg.speed = backup_speed                  # 后退速度（m/s）
        goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info(f'Backing up {goal_msg.target.x} m at {goal_msg.speed} m/s....')
        send_goal_future = self.backup_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Backup request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def assistedTeleop(self, time_allowance=30):
        """发送辅助遥控（AssistedTeleop）动作请求：人工遥控但自动避障。"""
        # 等待 assisted_teleop 动作服务器
        self.debug("Wainting for 'assisted_teleop' action server")
        while not self.assisted_teleop_client.wait_for_server(timeout_sec=1.0):
            self.info("'assisted_teleop' action server not available, waiting...")
        goal_msg = AssistedTeleop.Goal()
        goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info("Running 'assisted_teleop'....")
        send_goal_future = \
            self.assisted_teleop_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Assisted Teleop request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def followPath(self, path, controller_id='', goal_checker_id=''):
        """Send a `FollowPath` action request. 发送 FollowPath 动作请求（执行指定路径）。"""
        # 等待 controller_server 的 follow_path 动作
        self.debug("Waiting for 'FollowPath' action server")
        while not self.follow_path_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowPath' action server not available, waiting...")

        goal_msg = FollowPath.Goal()
        goal_msg.path = path                    # 待执行的路径
        goal_msg.controller_id = controller_id  # 指定控制器插件（空 = 默认）
        goal_msg.goal_checker_id = goal_checker_id  # 指定目标到达判定器

        self.info('Executing path...')
        send_goal_future = self.follow_path_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Follow path was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelTask(self):
        """Cancel pending task request of any type. 取消当前任意类型的任务。"""
        self.info('Canceling current task.')
        if self.result_future:
            # 异步取消目标并等待完成
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet. 检查任务是否已完成。"""
        if not self.result_future:
            # task was cancelled or completed
            # 无结果 future：任务已被取消或完成
            return True
        # 0.1 秒超时自旋：非阻塞地推进结果 future
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            # 超时未完成：任务仍在执行中
            return False

        self.debug('Task succeeded!')
        return True

    def getFeedback(self):
        """Get the pending action feedback message. 获取最近一帧动作反馈。"""
        return self.feedback

    def getResult(self):
        """Get the pending action result message. 把动作状态码映射为 TaskResult 枚举。"""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl'):
        """Block until the full navigation system is up and running.
        阻塞等待整个导航系统激活：先等定位器（amcl）激活，
        再等初始位姿被确认（仅 amcl），最后等导航器（bt_navigator）激活。"""
        self._waitForNodeToActivate(localizer)
        if localizer == 'amcl':
            self._waitForInitialPose()
        self._waitForNodeToActivate(navigator)
        self.info('Nav2 is ready for use!')
        return

    def _getPathImpl(self, start, goal, planner_id='', use_start=False):
        """
        Send a `ComputePathToPose` action request.

        Internal implementation to get the full result, not just the path.
        ComputePathToPose 动作的内部实现：返回完整结果对象（而非仅路径）。
        """
        # 等待 planner_server 的动作服务器
        self.debug("Waiting for 'ComputePathToPose' action server")
        while not self.compute_path_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'ComputePathToPose' action server not available, waiting...")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start              # 起点（use_start=false 时由服务器取当前位姿）
        goal_msg.goal = goal                # 目标点
        goal_msg.planner_id = planner_id    # 规划器插件 ID
        goal_msg.use_start = use_start      # 是否使用给定的起点

        self.info('Getting path...')
        # 同步等待：目标接受 -> 结果返回
        send_goal_future = self.compute_path_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Get path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn(f'Getting path failed with status code: {self.status}')
            return None

        return self.result_future.result().result

    def getPath(self, start, goal, planner_id='', use_start=False):
        """Send a `ComputePathToPose` action request. 请求规划一条到单点的路径（仅返回 path）。"""
        rtn = self._getPathImpl(start, goal, planner_id, use_start)
        if not rtn:
            return None
        else:
            return rtn.path

    def getPathThroughPoses(self, start, goals, planner_id='', use_start=False):
        """Send a `ComputePathThroughPoses` action request. 请求规划经过多个途经点的路径。"""
        # 等待 planner_server 的多途经点规划动作
        self.debug("Waiting for 'ComputePathThroughPoses' action server")
        while not self.compute_path_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'ComputePathThroughPoses' action server not available, waiting...")

        goal_msg = ComputePathThroughPoses.Goal()
        goal_msg.start = start
        goal_msg.goals = goals
        goal_msg.planner_id = planner_id
        goal_msg.use_start = use_start

        self.info('Getting path...')
        send_goal_future = self.compute_path_through_poses_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Get path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn(f'Getting path failed with status code: {self.status}')
            return None

        return self.result_future.result().result.path

    def _smoothPathImpl(self, path, smoother_id='', max_duration=2.0, check_for_collision=False):
        """
        Send a `SmoothPath` action request.

        Internal implementation to get the full result, not just the path.
        SmoothPath 动作的内部实现：返回完整结果对象（而非仅路径）。
        """
        # 等待 smoother_server 的动作服务器
        self.debug("Waiting for 'SmoothPath' action server")
        while not self.smoother_client.wait_for_server(timeout_sec=1.0):
            self.info("'SmoothPath' action server not available, waiting...")

        goal_msg = SmoothPath.Goal()
        goal_msg.path = path  # 待平滑的路径
        # 最大平滑时长（防止过久迭代阻塞导航）
        goal_msg.max_smoothing_duration = rclpyDuration(seconds=max_duration).to_msg()
        goal_msg.smoother_id = smoother_id          # 平滑插件 ID
        goal_msg.check_for_collisions = check_for_collision  # 是否检查碰撞

        self.info('Smoothing path...')
        send_goal_future = self.smoother_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Smooth path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn(f'Getting path failed with status code: {self.status}')
            return None

        return self.result_future.result().result

    def smoothPath(self, path, smoother_id='', max_duration=2.0, check_for_collision=False):
        """Send a `SmoothPath` action request. 请求平滑路径（仅返回平滑后的 path）。"""
        rtn = self._smoothPathImpl(
            path, smoother_id, max_duration, check_for_collision)
        if not rtn:
            return None
        else:
            return rtn.path

    def changeMap(self, map_filepath):
        """Change the current static map in the map server. 调用 load_map 服务更换地图。"""
        # 等待 map_server 的 load_map 服务
        while not self.change_maps_srv.wait_for_service(timeout_sec=1.0):
            self.info('change map service not available, waiting...')
        req = LoadMap.Request()
        req.map_url = map_filepath  # 地图 YAML 文件路径
        future = self.change_maps_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        status = future.result().result
        if status != LoadMap.Response().RESULT_SUCCESS:
            self.error('Change map request failed!')
        else:
            self.info('Change map request was successful!')
        return

    def clearAllCostmaps(self):
        """Clear all costmaps. 同时清空局部与全局代价地图。"""
        self.clearLocalCostmap()
        self.clearGlobalCostmap()
        return

    def clearLocalCostmap(self):
        """Clear local costmap. 调用服务清空局部代价地图。"""
        while not self.clear_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear local costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def clearGlobalCostmap(self):
        """Clear global costmap. 调用服务清空全局代价地图。"""
        while not self.clear_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear global costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def getGlobalCostmap(self):
        """Get the global costmap. 调用服务获取全局代价地图（OccupancyGrid）。"""
        while not self.get_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info('Get global costmaps service not available, waiting...')
        req = GetCostmap.Request()
        future = self.get_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def getLocalCostmap(self):
        """Get the local costmap. 调用服务获取局部代价地图（OccupancyGrid）。"""
        while not self.get_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info('Get local costmaps service not available, waiting...')
        req = GetCostmap.Request()
        future = self.get_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def lifecycleStartup(self):
        """Startup nav2 lifecycle system. 触发生命周期管理器启动整个 Nav2。"""
        self.info('Starting up lifecycle nodes based on lifecycle_manager.')
        # 遍历本节点可见的所有服务，找到 ManageLifecycleNodes 类型者
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                self.info(f'Starting up {srv_name}')
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(f'{srv_name} service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().STARTUP
                future = mgr_client.call_async(req)

                # starting up requires a full map->odom->base_link TF tree
                # so if we're not successful, try forwarding the initial pose
                # 启动需要完整的 map->odom->base_link TF 树：
                # 若启动请求迟迟未完成，反复重发初始位姿以帮助建立 TF
                while True:
                    rclpy.spin_until_future_complete(self, future, timeout_sec=0.10)
                    if not future:
                        self._waitForInitialPose()
                    else:
                        break
        self.info('Nav2 is ready for use!')
        return

    def lifecycleShutdown(self):
        """Shutdown nav2 lifecycle system. 触发生命周期管理器关闭整个 Nav2。"""
        self.info('Shutting down lifecycle nodes based on lifecycle_manager.')
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                self.info(f'Shutting down {srv_name}')
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(f'{srv_name} service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().SHUTDOWN
                future = mgr_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                future.result()
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        # 通过 <node>/get_state 服务轮询，直到生命周期节点进入 active 状态
        self.debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        # 每 2 秒查询一次状态，直到 active
        while state != 'active':
            self.debug(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f'Result of get_state: {state}')
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        # 循环发布初始位姿并等待 AMCL 回发 amcl_pose（确认已接收）
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1.0)
        return

    def _amclPoseCallback(self, msg):
        # 收到 amcl 位姿 => 初始位姿已被定位系统接收
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        # 动作反馈回调：缓存最新反馈供 getFeedback 使用
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        # 把 initial_pose 打包为 PoseWithCovarianceStamped 发布到 initialpose 话题
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

    # ---- 日志辅助函数 ----
    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return
