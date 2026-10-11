# nav2_smac_planner 整体流程

## 一、包定位

nav2 最强的规划器插件家族：Smac（Smooth Motion generation for Autonomous Cars）规划的三个变体——**2D 栅格 / Hybrid-A* 混合（车辆运动学约束）/ Lattice 状态格（完整运动基元）**，是 nav2 中难度最高的包之一。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `SmacPlanner2D` | `smac_planner_2d.cpp` | 2D A*（带解析扩角、代价感知） |
| `SmacPlannerHybrid` | `smac_planner_hybrid.cpp` | Hybrid-A*：SE(2) 搜索，满足最小转弯半径（阿克曼/差速） |
| `SmacPlannerLattice` | `smac_planner_lattice.cpp` | 预生成运动基元（state lattice），支持任意约束 |
| `AStarAlgorithm<T>` | `a_star.hpp/.cpp` | 模板化 A* 核心（按 Node 类型实例化） |
| `Node2D/NodeHybrid/NodeLattice` | `node_*.hpp` | 三种搜索节点（邻居扩展、运动基元、解析扩展） |
| `CollisionChecker` | `collision_checker.cpp` | 基于 costmap（可配合 footprint 或圆）碰撞检查 |
| `CostmapDownsampler` | `costmap_downsampler.cpp` | 搜索前降采样加速 |
| `Smoother` | `smoother.cpp` | 搜索后对路径做共轭梯度平滑（含碰撞/曲率约束） |
| `smac_planner_lattice_generator` | 独立工具节点 | 离线生成 lattice 基元表 |

## 三、主流程（以 Hybrid 为例）

```
configure(...)  ← 参数: minimum_turning_radius / motion_model_for_search
                 (DUBIN/REEDS_SHEPP) / allow_unknown / downsample_costmap / smoother…

createPlan(start, goal):
    1. 初始化 AStarAlgorithm<NodeHybrid>
       （角度离散 bin 数、搜索空间、代价权重 w_traversal/w_heuristic…）
    2. 可选降采样 costmap
    3. a_star->setStarts/setGoals（终点不可达时 tolerance 扩环）
    4. a_star->createPath(...)
         ├─ 搜索扩展: 沿运动基元(左转/右转/直行 × 距离)生成邻居
         ├─ 每步 CollisionChecker 检查 footprint 是否碰撞
         └─ 接近终点时解析扩展(Reeds-Shepp/Dubins 曲线直连)加速收敛
    5. 回溯路径 → Smoother 共轭梯度平滑(带曲率/碰撞约束)
    6. 返回 path + planning_time
```

## 四、三个变体选择

```
差速/全向轮、无运动学顾虑     → SmacPlanner2D（或 NavFn/Theta*）
阿克曼等有最小转弯半径约束    → SmacPlannerHybrid
复杂约束、需预生成基元        → SmacPlannerLattice（需先生成基元表）
```

## 五、与其他包的关系

- 插件被 `nav2_planner` 加载；`Smoother` 与 `nav2_smoother` 包无关（这是搜索后内部平滑）
- 建议在 navfn / theta_star 之后阅读
