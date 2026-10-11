# nav2_smoother 整体流程

## 一、包定位

路径平滑服务器：对外提供 `SmoothPath` action，把规划出的折线路径交给平滑器插件（`nav2_core::Smoother`）处理，使路径更适合控制器跟踪。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `SmootherServer` | `smoother_server.cpp` | lifecycle 服务器主体：action 分发、插件加载 |
| `SimpleSmoother` | `simple_smoother.cpp` | 官方内置简单平滑器（梯度下降式迭代平滑，无碰撞约束） |

## 三、主流程

```
on_configure
    ├─ 参数: smoother_plugins
    ├─ 加载平滑器插件（SimpleSmoother / ConstrainedSmoother / 自定义）
    └─ 创建 SmoothPath action server

收到 SmoothPath goal
    ├─ 按 smoother_id 选择插件（默认第一个）
    └─ smooth():
        1. 获取 costmap（check_for_collisions=true 时用于碰撞检查）
        2. plugin->smooth(path, max_smoothing_duration)
            ├─ 迭代式路径点松弛/优化
            └─ 超过 max_smoothing_duration → 提前返回（部分结果）
        3. 返回平滑后 path + smoothing_duration + was_completed
```

## 四、数据流

```
planner 折线路径 ──SmoothPath action──→ SmootherServer ──→ Smoother 插件
                                            ↑ costmap（碰撞检查）        │
                                            └──────── 平滑后路径 ←───────┘
（在行为树中由 SmoothPathAction 节点串在 ComputePathToPose 之后）
```

## 五、与其他包的关系

- 依赖 `nav2_core`、`nav2_util`；被 `nav2_behavior_tree` 的 `SmoothPathAction` 节点调用
- 插件对比：`SimpleSmoother`（本包，无约束） vs `nav2_constrained_smoother`（保持拓扑/碰撞安全）
