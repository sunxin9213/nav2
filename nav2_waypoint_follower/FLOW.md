# nav2_waypoint_follower 整体流程

## 一、包定位

航点巡航服务器：提供 `FollowWaypoints` action，按顺序导航到一串航点，并支持在航点处执行自定义任务（如拍照、等待），是 `bt_navigator` 之上的简单多目标编排层。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `WaypointFollowerServer` | `waypoint_follower_server.cpp` | lifecycle 服务器：循环调用导航、任务插件分发 |
| `WaitAtWaypoint` | `wait_at_waypoint.cpp` | 实现 `nav2_core::WaypointTaskExecutor`：到达航点后等待 pause_at_waypoint 时长 |

## 三、主流程

```
on_configure
    ├─ 加载 WaypointTaskExecutor 插件
    └─ 创建 FollowWaypoints action server + NavigateToPose action 客户端

收到 FollowWaypoints goal (poses[])
    └─ 循环 i = 0..N-1:
        1. 发布 feedback: current_waypoint = i
        2. 调用 bt_navigator 的 NavigateToPose（目标 = poses[i]）
            ├─ 成功 → task_executor->processAtWaypoint(...)（如原地等待）
            └─ 失败 → 记录 i 到 missed_waypoints，继续下一个
        3. 期间 goal 被抢占 → 中止并返回已完成的 missed 列表
    结束 → 返回 missed_waypoints
```

## 四、数据流

```
用户 ──FollowWaypoints──→ WaypointFollowerServer ──NavigateToPose(i)──→ bt_navigator
                                │                                        │
                                └────────── feedback: current_waypoint ←─┘
                            到点后 → TaskExecutor 插件（等待/自定义任务）
```

## 五、与其他包的关系

- 依赖 `nav2_util`、`nav2_msgs`，通过 action 客户端调用 `nav2_bt_navigator`
- `nav2_simple_commander` 的 `follow_waypoints()` 即封装此 action
- 阅读价值：看"服务器组合服务器"的编排模式 + `WaypointTaskExecutor` 插件接口
