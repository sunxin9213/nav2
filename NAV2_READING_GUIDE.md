# Nav2 源码阅读顺序指南

> 适用范围：navigation2 全部 ROS 包（不含 nav2_amcl）
> 阅读思路：**先接口 → 再基础设施 → 再服务器 → 再插件 → 最后集成**

---

## 总览流程图

```
第一阶段：基础接口与工具
  nav2_msgs → nav2_common → nav2_util → nav2_core
        │
        ▼
第二阶段：核心数据结构与基础设施
  nav2_costmap_2d → nav2_voxel_grid → nav2_map_server → nav2_lifecycle_manager
        │
        ▼
第三阶段：任务编排（行为树体系）
  nav2_behavior_tree → nav2_bt_navigator → nav2_behaviors
        │
        ├──────────────────────────────┐
        ▼                              ▼
第四阶段：规划链                    第五阶段：控制链
  nav2_planner                       nav2_controller
  nav2_navfn_planner                 nav2_dwb_controller
  nav2_theta_star_planner            nav2_regulated_pure_pursuit_controller
  nav2_smac_planner                  nav2_mppi_controller
  nav2_route                         nav2_graceful_controller
                                     nav2_rotation_shim_controller
                                     nav2_velocity_smoother
        │                              │
        └──────────────┬───────────────┘
                       ▼
第六阶段：路径平滑与上层应用
  nav2_smoother → nav2_constrained_smoother → nav2_waypoint_follower
  → nav2_collision_monitor → nav2_simple_commander
                       │
                       ▼
第七阶段：集成与验证
  nav2_bringup → nav2_rviz_plugins → nav2_system_tests → navigation2 / tools
```

---

## 第一阶段：基础接口与工具（理解骨架）

| 顺序 | 包 | 阅读重点 |
|---|---|---|
| 1 | `nav2_msgs` | 所有跨包通信的消息定义（action/srv/msg），作为字典随查随用 |
| 2 | `nav2_common` | launch 复用宏、参数工具，很薄，快速过一遍 |
| 3 | `nav2_util` | 生命周期节点基类、TF/Odometry 工具、costmap 帮助类，几乎所有包都依赖它 |
| 4 | `nav2_core` | **重点**。定义了所有插件接口（`Planner`、`Controller`、`Smoother`、`Behavior`、`CostmapLayer` 等抽象基类），读懂它就理解了 nav2 的插件体系 |

---

## 第二阶段：核心数据结构与基础设施

| 顺序 | 包 | 阅读重点 |
|---|---|---|
| 5 | `nav2_costmap_2d` | **最重要的包之一**。`Costmap2D` 主类、`LayeredCostmap`、`Layer` 生命周期（`onInitialize / onActivate / updateBounds / updateCosts`）、static/obstacle/inflation 各 layer、costmap filter 机制 |
| 6 | `nav2_voxel_grid` | 三维体素网格，obstacle layer 的 3D 扩展使用，很小 |
| 7 | `nav2_map_server` | 地图加载/保存服务，同时学习 lifecycle 节点的标准写法 |
| 8 | `nav2_lifecycle_manager` | nav2 如何用 ROS2 lifecycle 统一管理十余个节点的启停 |

---

## 第三阶段：任务编排（行为树体系）

| 顺序 | 包 | 阅读重点 |
|---|---|---|
| 9 | `nav2_behavior_tree` | BT 节点库（action/condition/control/decorator），重点看 `NavigateToPose` 的默认 XML 与配套节点 |
| 10 | `nav2_bt_navigator` | BT 执行引擎，理解 navigator action 如何驱动整棵树 |
| 11 | `nav2_behaviors` | 恢复行为服务器（spin、backup、wait 等），`nav2_core::Behavior` 接口的官方参考实现 |

---

## 第四阶段：规划链（Planner Server + 各规划器插件）

由简到繁：**navfn → theta_star → smac**，route 独立成体系最后看。

| 顺序 | 包 | 阅读重点 |
|---|---|---|
| 12 | `nav2_planner` | 规划服务器本体，理解 action 如何分发给插件 |
| 13 | `nav2_navfn_planner` | 最简单的 Dijkstra/A* 实现，插件入门样例 |
| 14 | `nav2_theta_star_planner` | 任意角规划，代码量小 |
| 15 | `nav2_smac_planner` | 最强的规划器（2D / Hybrid-A* / Lattice），难度最高 |
| 16 | `nav2_route` | 新的路网图（graph）路由系统，配合 `Route` 系列消息阅读 |

---

## 第五阶段：控制链（Controller Server + 各控制器插件）

由简到繁：**DWB/RPP → mppi**。

| 顺序 | 包 | 阅读重点 |
|---|---|---|
| 17 | `nav2_controller` | 控制服务器，重点看 odom 平滑、progress checker、goal checker 的组合方式 |
| 18 | `nav2_dwb_controller` | 传统采样评分式控制器，带 critic 体系，理解"采样-评分"范式的好例子 |
| 19 | `nav2_regulated_pure_pursuit_controller` | Pure Pursuit + 速度调节，几何式控制器代表，代码短 |
| 20 | `nav2_mppi_controller` | 当前主推的优化式控制器，概念多（轨迹采样、critic、optimization），建议在 DWB 之后看 |
| 21 | `nav2_graceful_controller` | 融合规划与跟踪的小型控制器 |
| 22 | `nav2_rotation_shim_controller` | 旋转包装器，看"shim"如何包装其他控制器 |
| 23 | `nav2_velocity_smoother` | cmd_vel 平滑与限加速度的收口环节 |

---

## 第六阶段：路径平滑与上层应用

| 顺序 | 包 | 阅读重点 |
|---|---|---|
| 24 | `nav2_smoother` | 平滑服务器，一对着插件看 |
| 25 | `nav2_constrained_smoother` | 官方平滑器插件实现 |
| 26 | `nav2_waypoint_follower` | 航点巡航，bt_navigator 的简单组合应用 |
| 27 | `nav2_collision_monitor` | 独立于 BT 的安全层，直接订阅激光/声呐做紧急安全动作 |
| 28 | `nav2_simple_commander` | Python API（`BasicNavigator`），上层业务使用 nav2 的最直观例子 |

---

## 第七阶段：集成与验证

| 包 | 说明 |
|---|---|
| `nav2_bringup` | launch 文件 + 各默认参数 YAML，把前面所有包串起来的"配置地图"，建议第三阶段后就开始对照着看 |
| `nav2_rviz_plugins` | 可视化调试工具 |
| `nav2_system_tests` | 集成测试，看它如何构造场景验证整条链路 |
| `navigation2` / `tools` | metapackage 和 CI 工具，最后扫一眼即可 |

---

## 建议的主线阅读方法

1. **抓两条主线**：
   - **请求流**：`NavigateToPose` action → bt_navigator → planner → controller → velocity_smoother → cmd_vel
   - **数据流**：map_server → costmap_2d layers → 各算法消费
2. 每读一个 server，先看它实现的 `nav2_core` 接口和它暴露的 action/srv（配合 `nav2_msgs` 中的注释），再看插件加载逻辑。
3. 插件包按"先简单后复杂"排：
   - 规划器：navfn → theta_star → smac
   - 控制器：DWB / RPP → mppi
