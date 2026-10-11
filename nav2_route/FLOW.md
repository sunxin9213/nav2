# nav2_route 整体流程

## 一、包定位

基于**路网图（graph）**的路由系统（新特性）：在预定义的节点/边路网上做全局路由（类似"车道级导航"），可替代/补充自由空间规划，并支持路口操作提示、动态封路等。

## 二、核心组件（`src/`）

| 组件 | 文件 | 职责 |
|---|---|---|
| `RouteServer` | `route_server.cpp` | lifecycle 服务器主体：action(ComputeRoute / ComputeAndTrackRoute)、服务(DynamicEdges / SetRouteGraph)、流程编排 |
| `GraphLoader` / `GraphSaver` | `graph_loader.cpp` 等 | 从 GeoJSON 等文件加载/保存路网图 |
| `NodeSpatialTree` | `node_spatial_tree.cpp` | 节点的 R-Tree 空间索引：由位姿快速查最近节点 |
| `RoutePlanner` | `route_planner.cpp` | 图上 A* 搜索（起点→终点最优节点/边序列） |
| `EdgeScorer` | `edge_scorer.cpp` + `plugins/` | 边代价计算框架 + 各 scorer 插件（距离、时间、自定义代价） |
| `GoalIntentExtractor` / `GoalIntentSearch` | `goal_intent_extractor.cpp` 等 | 将自由的 start/goal 位姿映射到图上的节点/边意图 |
| `RouteTracker` | `route_tracker.cpp` | ComputeAndTrackRoute 的跟踪循环：判断进度、偏航重规划、触发操作 |
| `PathConverter` | `path_converter.cpp` | Route（节点/边序列）→ nav_msgs/Path（供 controller 跟踪） |
| `OperationsManager` | `operations_manager.cpp` + `plugins/` | 路由操作框架：经过某条边/节点时触发（如"路口左转"提示） |

## 三、主流程

```
on_configure
    ├─ GraphLoader 加载路网图 → 构建 NodeSpatialTree
    ├─ 加载 EdgeScorer 插件 + Operations 插件
    └─ 创建 action / 服务

ComputeRoute action:
    1. use_poses=false → 用 start_id/goal_id；否则 GoalIntentExtractor
       把位姿匹配到图上（NodeSpatialTree 最近搜索）
    2. RoutePlanner 在图上 A* 搜索（EdgeScorer 给每条边打分）
    3. 返回 Route + 展开的 Path + planning_time（错误码见 .action 文件注释）

ComputeAndTrackRoute action（在上述基础上）:
    循环（每个周期）:
        ├─ TF 获取当前位姿 → 判断当前在哪个 edge/node
        ├─ OperationsManager: 经过特定边时触发操作（反馈中上报）
        ├─ 偏离路由/边被动态关闭 → 重新 RoutePlanner（rerouted=true）
        └─ 到达终点 → 完成
    feedback: last/next_node_id, current_edge_id, route, path, operations_triggered, rerouted

服务:
    DynamicEdges ──→ 运行时开/关边、调整边代价（封路、单行道管控）
    SetRouteGraph ──→ 热加载新图（换楼层/厂区）
```

## 四、数据流

```
GeoJSON 图文件 ──GraphLoader──→ Graph + R-Tree
位姿(TF) ──GoalIntentExtractor──→ 图上起终点
图 + EdgeScorer ──RoutePlanner──→ Route ──PathConverter──→ Path
                                                        └→ controller 跟踪
DynamicEdges/SetRouteGraph ──→ 动态修改图
```

## 五、与其他包的关系

- 依赖 `nav2_msgs`（Route 系列消息、路由 action）
- 输出 Path 后仍由 `nav2_controller` 执行跟踪；与 `nav2_planner`（自由空间规划）是并列的两种全局路由方案
