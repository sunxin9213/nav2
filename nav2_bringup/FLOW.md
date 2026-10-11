# nav2_bringup 整体流程

## 一、包定位

nav2 的**启动与配置入口**：launch 文件 + 各场景默认参数 YAML，把前面所有包按正确拓扑拉起来。是理解"整个系统如何组装"的地图。

## 二、核心组件

| 文件 | 作用 |
|---|---|
| `launch/bringup_launch.py` | 总入口：按 use_composition 组合/分离方式启动定位组 + 导航组 |
| `launch/localization_launch.py` | 定位组：map_server + (amcl)，lifecycle_manager_localization 管理 |
| `launch/navigation_launch.py` | 导航组：bt_navigator、controller_server、planner_server、smoother_server、behavior_server、waypoint_follower、velocity_smoother、(collision_monitor)，lifecycle_manager_navigation 管理 |
| `launch/rviz_launch.py` | 启动 RViz + 官方配置（含 nav2_rviz_plugins 面板） |
| `launch/slam_launch.py` | SLAM 建图模式（在线建图代替 amcl+map_server） |
| `params/nav2_params.yaml` | **所有服务器的默认参数与插件选择**，读代码时对照必看 |
| `launch/unique_entity_launch.py` 等 | 单实体/多机器人（Multi-TF）变体启动 |

## 三、启动流程

```
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=... params_file:=...
    │
    ├─ RewrittenYaml(nav2_common) 处理参数文件（替换 use_sim_time 等）
    │
    ├─ 定位组 (localization_launch.py)
    │    map_server → 发布 map
    │    amcl → 定位
    │    lifecycle_manager_localization: autostart → configure+activate 上面节点
    │
    └─ 导航组 (navigation_launch.py)
         bt_navigator / controller / planner / smoother / behaviors /
         velocity_smoother / collision_monitor
         lifecycle_manager_navigation: 按序 configure+activate
              全部 ACTIVE → 导航可用（等待 NavigateToPose goal）
```

## 四、运行时节点拓扑

```
                 NavigateToPose action
用户/RViz ─────────────┬──────────────
                       ▼
                  bt_navigator ── 行为树
                 ┌───────┼─────────┬──────────────┐
                 ▼       ▼         ▼              ▼
            planner  smoother  controller   behaviors(恢复)
                 ▲                   │ cmd_vel
                 │                   ▼
              costmap(全局/局部)  velocity_smoother
                                     │
                                     ▼
                              collision_monitor ──→ 底盘
```

## 五、与其他包的关系

- 不含算法代码，纯 launch/params；依赖几乎所有功能包
- **强烈建议**：阅读任何服务器代码时，同步打开 `params/nav2_params.yaml` 查看其参数与所选插件
