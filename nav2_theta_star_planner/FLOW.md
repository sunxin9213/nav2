# nav2_theta_star_planner 整体流程

## 一、包定位

任意角（Any-Angle）全局规划器插件：基于 Theta* 算法，A* 的改进版——允许父节点"视线"直连，路径更短更平直（不再是 45° 折线）。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `ThetaStarPlanner` | `theta_star_planner.cpp` | 实现 `nav2_core::GlobalPlanner`，A* + LOS(line of sight) 检查 |

## 三、主流程

```
configure(...)  ← 参数: use_astar / w_euc_cost(启发权重) / w_traversal_cost(代价权重)
                  / w_heuristic_cost / tolerance / allow_unknown

createPlan(start, goal):
    1. 获取全局 costmap → 代价数组（含 lethal 检查）
    2. A* open/close 集搜索:
         扩展邻居 s' 时, 额外检查 s 的父节点与 s' 是否"视线可达"(
            LOSCheck(): Bresenham 直线遍历, 途中无致命栅格)
         ├─ 可达 → s' 直接挂到祖父节点(g_value 更小, 路径更直)
         └─ 不可达 → 按普通 A* 挂到 s
    3. 启发函数 = 欧氏距离*w_euc + 终点栅格代价*w_heuristic
    4. 回溯得到路径 → (导航层面) 可再交给 smoother 平滑
```

## 四、数据流

```
全局 costmap ──→ 代价数组 + LOS 检查 ──→ Theta* 搜索 ──→ 路径
```

## 五、与其他包的关系

- 插件形式被 `nav2_planner` 加载；代码量小，适合在 navfn 之后读，体会"搜索空间松弛"的思路。
