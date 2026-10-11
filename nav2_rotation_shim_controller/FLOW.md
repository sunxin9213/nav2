# nav2_rotation_shim_controller 整体流程

## 一、包定位

控制器**包装器（shim）**：不改写任何跟踪算法，只在主控制器之前插入"先原地转正"逻辑——当机器人朝向与路径起始方向偏差过大时，先原地旋转对准，再交给被包装的主控制器跟踪。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `RotationShimController` | `rotation_shim_controller.cpp` | 实现 `nav2_core::Controller`，内部持有另一个主控制器实例 |

## 三、主流程

```
configure(...)
    └─ 按参数 primary_controller 创建并配置主控制器
       （如 RegulatedPurePursuitController / MPPI…）

computeVelocityCommands(pose, velocity, goal_checker):
    1. 计算路径起始段方向角 vs 机器人当前朝向的偏差
    2. |偏差| > angular_dist_threshold ?
        ├─ 是 → 进入旋转模式:
        │     w = K_θ * 偏差（限幅）
        │     v = 0
        │     （若前进角 > 90° 可选 forward_sampling_point 采更远点判断）
        │     偏差收敛后 → 切回主控制器
        └─ 否 → delegate → primary_controller->computeVelocityCommands(...)
    3. 输出 cmd_vel（旋转期间也会做碰撞检查，可选）
```

## 四、数据流

```
FollowPath goal ──→ RotationShim
                      ├─ 偏差大 → 原地旋转指令
                      └─ 偏差小 → 主控制器(RPP/MPPI/…) → cmd_vel
```

## 五、与其他包的关系

- 作为 `nav2_core::Controller` 插件被 `nav2_controller` 加载；参数中嵌套声明主控制器插件
- 典型组合：RPP（起步大偏差时容易切摆）+ RotationShim 先转正
- 是学习"装饰器模式插件"的最小样例
