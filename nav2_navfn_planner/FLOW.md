# nav2_navfn_planner 整体流程

## 一、包定位

最经典的全局规划器插件：NavFn（Dijkstra / A*），基于 4/8 邻域栅格搜索，是理解 nav2 规划插件体系的**入门样例**。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `NavfnPlanner` | `navfn_planner.cpp` | 实现 `nav2_core::GlobalPlanner`，串联流程 |
| `navfn.h/.cpp` | 移植的 NavFn 核心 | 波前扩展搜索（potential 场）+ 路径提取 |

## 三、主流程

```
configure(parent, name, ...)      ← 读参数: use_astar / tolerance / allow_unknown
createPlan(start, goal):
    1. 从 costmap_ros 获取主栅格（costarr: 0~254 → 归一化代价）
    2. setCostmap() + 设置起点/终点
       （终点被占用时按 tolerance 参数向外扩环搜索最近可达终点）
    3. calcNavFnDijkstra(true) 或 calcNavFnAstar()
        ← 从起点扩展波前，计算每个栅格的 potential（到达代价场）
    4. calcPath() ← 从终点沿 potential 梯度下降回溯得到路径点
    5. publishPlan() (规划路径可视化)
    6. 返回 nav_msgs/Path
```

## 四、数据流

```
全局 costmap(0~254) ──归一化──→ costarr
    └─ Dijkstra/A* 扩展 ──→ potential 场 ──梯度回溯──→ 折线路径
```

## 五、与其他包的关系

- 作为插件被 `nav2_planner` 加载；无独立节点
- 对比阅读：`nav2_theta_star_planner`（任意角）、`nav2_smac_planner`（更强搜索）
