# nav2_core 整体流程

## 一、包定位

**纯接口包**（只有抽象基类头文件，无实现、无节点），定义了 nav2 全部插件体系的行为契约。读懂本包 = 理解 nav2 的插件化架构。

## 二、核心接口（抽象基类）

| 接口 | 文件 | 实现方（插件） | 调用方（服务器） |
|---|---|---|---|
| `GlobalPlanner` | `global_planner.hpp` | navfn / theta_star / smac / ... | nav2_planner |
| `Controller` | `controller.hpp` | dwb / rpp / mppi / graceful / rotation_shim | nav2_controller |
| `Smoother` | `smoother.hpp` | simple / constrained | nav2_smoother |
| `Behavior` | `behavior.hpp` | spin / backup / drive_on_heading / wait / assisted_teleop | nav2_behaviors |
| `CostmapLayer` | `costmap_layer.hpp` | static / obstacle / voxel / inflation / range / denoise / filters | nav2_costmap_2d |
| `CostmapFilter` | `costmap_filter.hpp` | keepout / speed filter | nav2_costmap_2d |
| `GoalChecker` | `goal_checker.hpp` | simple_goal_checker / position_goal_checker / ... | nav2_controller / nav2_behaviors |
| `ProgressChecker` | `progress_checker.hpp` | progress_checker / pose_progress_checker | nav2_controller |
| `WaypointTaskExecutor` | `waypoint_task_executor.hpp` | wait_at_waypoint 等 | nav2_waypoint_follower |

## 三、接口约定的统一生命周期

```
服务器 on_configure
    └─ plugin_loader 创建插件实例
        └─ plugin->configure(parent, name, ...)   ← 一次性初始化（读参数、建订阅）
服务器 on_activate
    └─ plugin->activate()                         ← 可重复启停（激活订阅/发布）
服务器 on_deactivate
    └─ plugin->deactivate()
服务器 on_cleanup
    └─ plugin->cleanup()
```

## 四、典型调用约定（以 Controller 为例）

```
controller_server 每个 control 周期:
    pose = TF 查询机器人位姿
    velocity = odom_smoother 当前速度
    cmd_vel  = plugin->computeVelocityCommands(pose, velocity, goal_checker)
```

## 五、异常体系

`exception.hpp` 定义 `PlannerException`、`ControllerException`、`SmootherException`、`BehaviorException` 等，服务器捕获后转换为 action 的失败结果（各 action 中对应的错误码）。

## 六、与其他包的关系

- 依赖 `nav2_costmap_2d` 等少量类型定义；被所有服务器包和插件包依赖。
- **建议第一个精读**，之后读任何插件/服务器都能对号入座。
