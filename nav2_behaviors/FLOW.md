# nav2_behaviors 整体流程

## 一、包定位

恢复/辅助行为服务器：实现原地旋转、倒车、直行、等待、辅助遥控等原子动作，是行为树中 RecoveryNode 分支的执行者，也是 `nav2_core::Behavior` 接口的官方参考实现。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `BehaviorServer` | `behavior_server.cpp` | lifecycle 节点：加载行为插件、SimpleActionServer、发布 cmd_vel、碰撞检查、参数（odom_topic 等） |
| `Spin` | `plugins/spin.cpp` | 原地旋转 target_yaw（反馈：已转角位移） |
| `BackUp` | `plugins/back_up.cpp` | 直线倒退（负向速度） |
| `DriveOnHeading` | `plugins/drive_on_heading.cpp` | 沿当前朝向直行 |
| `Wait` | `plugins/wait.cpp` | 原地等待指定时长 |
| `AssistedTeleop` | `plugins/assisted_teleop.cpp` | 遥控中继：订阅 cmd_vel，超时自动停 |

## 三、单个行为的标准执行流程

```
action goal (如 Spin)
    └─ BehaviorServer 分发到对应插件
        插件 on_configure/on_activate（生命周期同 nav2_core 约定）
        └─ 运行循环（每个周期）:
             1. TF 获取当前位姿 / OdomSmoother 获取当前速度
             2. 计算本周期 twist（P 控制到目标）
             3. 碰撞检查（collision_checker，基于局部 costmap）
                 └─ 有碰撞 → 提前中止，返回失败
             4. 发布 cmd_vel + feedback
             5. GoalChecker 判定是否完成
        完成 → 停车（publish zero twist）→ 返回 total_elapsed_time
    超过 time_allowance → 中止（防卡死）
    action 被抢占 → 停车并交出控制权
```

## 四、数据流

```
BT 节点(SpinAction 等) ──goal──→ BehaviorServer ──cmd_vel──→ 机器人
                                     ↑ 局部 costmap（碰撞检查）
                                     ↑ odom（速度反馈）
```

## 五、与其他包的关系

- 依赖 `nav2_core`、`nav2_util`、`nav_2d_utils`（少量消息转换）
- 被 `nav2_behavior_tree` 的 action 节点调用；是编写自定义 Behavior 插件的最佳模板（对照 `nav2_core/behavior.hpp` 阅读）。
