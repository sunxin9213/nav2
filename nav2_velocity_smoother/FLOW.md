# nav2_velocity_smoother 整体流程

## 一、包定位

速度指令平滑器：位于控制器与底盘之间的**最后一道收口**。订阅原始 `cmd_vel`，按加速度/加加速度约束平滑后发布，消除控制器的抖动与突变。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `VelocitySmoother` | `velocity_smoother.cpp` | lifecycle 节点：平滑状态机 + 限幅 |

## 三、主流程

```
on_configure
    ├─ 参数: max_accel/decel、max_decel、平滑周期 smoothing_frequency
    │        velocity_timeout（指令超时停车）、deadband_velocity、feedback(OPEN_LOOP/CLOSED_LOOP)
    └─ 输入/输出话题: cmd_vel (订阅) → cmd_vel_smoothed (发布)

运行（周期 smoothing_frequency）:
    1. 收到新目标速度 target（若 velocity_timeout 内无指令 → 目标置零）
    2. OPEN_LOOP: 直接积分上一输出 / CLOSED_LOOP: 结合 odom 反馈修正
    3. 状态机逐轴(vx,vy,w)限加速度:
         当前值 → target 之间按 (de)celeration * dt 步进
         （加速用 max_accel，减速用 max_decel，各自可设负向不同值）
    4. deadband: 小于阈值的分量清零
    5. 发布 cmd_vel_smoothed
```

## 四、数据流

```
controller_server ──cmd_vel──→ VelocitySmoother ──cmd_vel_smoothed──→ 底盘
                                   ↑ odom（CLOSED_LOOP 模式）
```

## 五、与其他包的关系

- 处于控制链末端；在 bringup 中位于 collision_monitor 之前/之后取决于安全策略
- 注意链路顺序：controller → velocity_smoother → collision_monitor → 底盘（常见配置）
