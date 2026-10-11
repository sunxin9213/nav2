# nav2_graceful_controller 整体流程

## 一、包定位

几何式局部控制器：**Graceful Controller**——使用"平滑控制律（Smooth Control Law）"跟踪路径，并在目标不可直接到达（被障碍阻挡/朝向不对）时执行"优雅"的避让机动（如后退转向），适合紧凑空间。

## 二、核心组件（`src/`）

| 组件 | 文件 | 职责 |
|---|---|---|
| `GracefulController` | `graceful_controller.cpp` | 主插件（`nav2_core::Controller`），串联各部件 |
| `PathHandler` | `path_handler.cpp` | 路径预处理（截断、坐标变换、局部路径维护） |
| `SmoothControlLaw` | `smooth_control_law.cpp` | 核心控制律：计算速度与曲率（非线性反馈律） |
| `ParameterHandler` | `parameter_handler.cpp` | 动态参数管理 |
| `utils.cpp` | 工具 | 角度处理、几何计算 |

## 三、主流程

```
computeVelocityCommands(pose, velocity, goal_checker):
    1. PathHandler: 路径转到机器人系，取局部段
    2. 尝试用 SmoothControlLaw 跟踪:
        - 根据目标点方向/距离计算线速度与转向
        - 速度随曲率/接近程度衰减
    3. 碰撞检查（costmap + footprint）:
        ├─ 前方路径被阻挡 且 允许恢复机动
        │     └─ 触发 graceful 机动: 反向/旋转组合控制律，
        │        先倒退腾出空间再重新对准（期间速度受限）
        └─ 无碰撞 → 正常跟踪输出
    4. GoalChecker 判定到达（接近终点时纯旋转对准目标朝向）
    5. 返回 cmd_vel
```

## 四、数据流

```
path → SmoothControlLaw(v, curvature) ──碰撞OK──→ cmd_vel
              │
              └─碰撞 → graceful 恢复机动（倒车+转向）→ 重新跟踪
```

## 五、与其他包的关系

- 插件被 `nav2_controller` 加载
- 与 RPP 对照：均为几何式，但 graceful 强调"目标受阻时的优雅处理"；行为树的 BackUp 是一次性恢复动作，这里是控制器内建的连续机动。
