# nav2_msgs 整体流程

## 一、包定位

接口定义包（**无运行时代码**），定义 nav2 全部跨包通信接口。编译时由 rosidl 生成 C++/Python 代码，供其他包 `find_package` 引用。

## 二、接口组成

| 目录 | 数量 | 内容 |
|---|---|---|
| `action/` | 15 | 导航（NavigateToPose/ThroughPoses）、规划（ComputePathToPose/ThroughPoses）、控制（FollowPath）、平滑（SmoothPath）、行为（Spin/BackUp/DriveOnHeading/Wait/AssistedTeleop）、路由（ComputeRoute/ComputeAndTrackRoute）、航点（FollowWaypoints）、测试（DummyBehavior） |
| `srv/` | 11 | 代价地图（Clear*/GetCostmap/IsPathValid）、地图（LoadMap/SaveMap）、生命周期管理（ManageLifecycleNodes）、初始位姿（SetInitialPose）、路由图（DynamicEdges/SetRouteGraph） |
| `msg/` | 14 | 行为树日志（BehaviorTreeLog/StatusChange）、代价地图（Costmap/CostmapMetaData/CostmapFilterInfo）、粒子（Particle/ParticleCloud）、速度限制（SpeedLimit）、体素（VoxelGrid）、路由（Route/RouteNode/RouteEdge/EdgeCost）、碰撞监测状态（CollisionMonitorState） |

## 三、"流程"（消息在系统中的流转）

```
                    ┌─ action ──→ bt_navigator / planner / controller / smoother /
                    │            behaviors / route 各服务器
nav2_msgs 定义 ────┼─ srv ─────→ map_server / costmap / lifecycle_manager / route
                    │
                    └─ msg ─────→ 各服务器周期发布状态（costmap、粒子云、BT 日志、限速等）
```

## 四、阅读建议

各文件均已添加中文注释（参数含义 + 适用场景），建议作为**字典**在其他包阅读过程中随查随用，不必通读。

## 五、与其他包的关系

- 被所有功能包依赖，处于依赖链最底层。
