# nav2_rviz_plugins 整体流程

## 一、包定位

RViz2 插件（Qt 面板）：提供 nav2 的图形化调试/操作界面——下发目标、初始位姿、航点任务、可视化行为树状态等。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `Nav2Panel` | `src/nav2_panel.cpp` | 主面板：初始位姿/单目标/多点目标按钮、导航启停（ManageLifecycleNodes）、恢复行为按钮（spin/backup） |
| `GoalPoseUpdater` 等 | `src/goal_pose_updater.cpp` | RViz 3D 视图中点选 pose 后发布 `/initialpose` / `/goal_pose` 等 |
| BT 可视化 | `src/behavior_tree_panel.cpp`? | 订阅 `/behavior_tree_log`（BehaviorTreeLog 消息）实时显示树节点状态（含 Groot 风格树渲染 `bt_svg_graph`） |

（另含 `nav2_panel` 相关的 dock widget、initial pose / goal pose 工具条类）

## 三、交互流程

```
用户在 RViz 中操作
    ├─ 工具条点选 2D Goal Pose ──→ 发布 /goal_pose ──→ RViz 内置插件 / 上层任务
    ├─ Nav2Panel "Start Navigation" ──→ ManageLifecycleNodes(STARTUP) ──→ lifecycle_manager
    ├─ Nav2Panel "Initial Pose" ──→ /initialpose ──→ 定位
    ├─ Nav2Panel 恢复按钮 ──→ 直接调用 Spin/BackUp 等 action ──→ behavior_server
    └─ BT 面板订阅 /behavior_tree_log ──→ 渲染节点实时状态(SUCCESS/FAILURE/RUNNING)
```

## 四、与其他包的关系

- 消费 `nav2_msgs`（BehaviorTreeLog、ManageLifecycleNodes 等）；不参与控制链路
- 被 `nav2_bringup/rviz_launch.py` 自动加载
- 阅读价值：调 UI 或做自定义可视化时参考；功能调试阶段用得多
