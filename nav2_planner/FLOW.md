# nav2_planner 整体流程

## 一、包定位

全局规划服务器：对外提供 `ComputePathToPose` / `ComputePathThroughPoses` action，把请求分发给具体的规划器插件（`nav2_core::GlobalPlanner`）。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `PlannerServer` | `planner_server.cpp` | lifecycle 服务器主体：action 处理、插件加载与切换、costmap 访问 |

## 三、主流程

```
on_configure
    ├─ 声明参数: planner_plugins / expected_planner_frequency
    ├─ 按 planner_plugins 参数逐个加载规划器插件并调用 configure()
    └─ 初始化两个 action server（ToPose / ThroughPoses）

收到 ComputePathToPose goal
    ├─ use_start=false → TF 查询机器人当前位姿作为起点
    ├─ 按 goal->planner_id 选择插件（默认第一个）
    └─ computePlan():
          1. 等待/获取全局 costmap（CostmapBinder / costmap_ros 互斥锁）
          2. path = plugin->createPlan(start, goal)
          3. 异常(PlannerException) → action 失败
          4. 返回 path + planning_time

ComputePathThroughPoses: 插件若支持多途经点则一次调用
    （否则逐段调用 createPlan 再拼接）
```

## 四、数据流

```
bt_navigator(ComputePathToPoseAction) ──→ PlannerServer
                                              │ 全局 costmap (nav2_costmap_2d)
                                              ▼
                                         GlobalPlanner 插件
                          (navfn / theta_star / smac / 自定义)
```

## 五、与其他包的关系

- 本身**不含任何算法**，只是插件容器；读它重点看 action 分发与 costmap 生命周期管理
- 注册插件方式示例（params yaml）:
  ```yaml
  planner_server:
    ros__parameters:
      planner_plugins: ["GridBased"]
      GridBased:
        plugin: "nav2_navfn_planner/NavfnPlanner"
  ```
