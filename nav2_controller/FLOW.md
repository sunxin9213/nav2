# nav2_controller 整体流程

## 一、包定位

局部控制服务器：对外提供 `FollowPath` action，把"跟踪路径"任务分发给控制器插件（`nav2_core::Controller`），并把速度指令发布到 `cmd_vel`。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `ControllerServer` | `controller_server.cpp` | lifecycle 服务器主体：控制循环、插件加载、cmd_vel 发布 |
| `SimpleProgressChecker` / `PoseProgressChecker` | `progress_checker.cpp` | 判断机器人是否卡住（一段时间位姿无进展 → 抛异常中止） |
| `SimpleGoalChecker` / `PositionGoalChecker` / `GoalCheckerSelector`? | `goal_checker.cpp` | 判定是否到达目标（xy 距离 + yaw 偏差阈值） |
| `OdomSmoother` | （nav2_util） | 当前速度反馈 |

## 三、主流程

```
on_configure
    ├─ 参数: controller_plugins / progress_checker_plugin / goal_checker_plugins
    ├─ 加载控制器插件 + progress checker + goal checker（均为插件化）
    └─ 创建 FollowPath action server、cmd_vel publisher、odom 订阅

收到 FollowPath goal
    └─ 控制循环（周期 1/controller_frequency）:
        1. TF 获取当前位姿（超时 → 报错）
        2. OdomSmoother 获取当前速度
        3. cmd_vel = plugin->computeVelocityCommands(pose, velocity, goal_checker)
        4. 发布 cmd_vel + feedback（distance_to_goal, speed）
        5. ProgressChecker.isRobotProgressing()
             └─ 卡住 → 抛异常 → action 失败（交给行为树恢复）
        6. GoalChecker.isGoalReached() → 到达 → 停车 → action 成功
    goal 被抢占/新目标 → 停车、切换插件
```

## 四、数据流

```
BT(FollowPathAction) ──path──→ ControllerServer ──cmd_vel──→ (velocity_smoother) ──→ 机器人
                                    ↑ 全局/局部 costmap（控制器插件内部使用）
                                    ↑ odom（速度反馈）
```

## 五、与其他包的关系

- 本身不含跟踪算法，是插件容器；所有 `nav2_core::Controller` 实现（dwb/rpp/mppi/graceful/rotation_shim）在此加载
- goal_checker 也被 `nav2_behaviors` 复用
