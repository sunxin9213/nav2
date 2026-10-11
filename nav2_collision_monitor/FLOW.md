# nav2_collision_monitor 整体流程

## 一、包定位

独立于行为树/规划控制链的**安全层**：直接消费传感器原始数据（激光/点云/声呐）与机器人运动学信息，在危险迫近时立即干预速度指令（停止/减速/按时间接近），不经过 costmap，响应最快。

## 二、核心组件（`src/`）

| 组件 | 文件 | 职责 |
|---|---|---|
| `CollisionMonitorNode` | `collision_monitor_node.cpp` | 主节点：编排数据源→多边形→动作的流程 |
| `Source`（基类）及子类 | `source.cpp` / `scan.cpp` / `pointcloud.cpp` / `range.cpp` | 传感器数据源：统一转换为点云（并做 TF 变换、max_range 裁剪） |
| `Polygon`（基类）及子类 | `polygon.cpp` / `circle.cpp` | 安全区几何（多边形/圆），按速度动态外扩（speed_add_base/speed_scaling） |
| `Kinematics` | `kinematics.cpp` | 封装机器人当前速度（订阅 cmd_vel 反馈或 odom） |

## 三、主流程

```
on_configure
    ├─ 加载数据源（scan/pointcloud/range 列表）
    ├─ 加载多级安全区（ Polygon/Circle 列表，按 action_type 分级:
    │    SLOWDOWN(慢) → APPROACH(按碰撞时间接近) → STOP(立即停)）
    └─ 创建输入/输出速度话题 + CollisionMonitorState 发布

运行（每帧传感器数据 / 周期）:
    1. 各 Source 将数据转成统一点云（机器人系）
    2. 各 Polygon:
         ├─ 按当前速度动态调整范围（速度越快越早触发）
         ├─ 逐点做多边形内/圆内判定
         └─ 命中 → 记录 (polygon, action_type)
    3. 取最严厉动作:
         ├─ STOP       → 输出速度置零
         ├─ SLOWDOWN   → 输出速度 × slowdown_ratio
         └─ APPROACH   → 按 time_before_collision 约束:
                          保证到最近障碍的时间不低于设定值 → 限制速度
    4. 发布修正后的 cmd_vel_out + CollisionMonitorState（触发的 polygon/动作）
```

## 四、数据流

```
scan/pointcloud/range ──→ 点云 ─┐
                                 ├→ 多级安全区判定 → STOP/SLOWDOWN/APPROACH
cmd_vel(上游控制器) ────────────┤        │
                                 │        ├→ cmd_vel_out（修正后）→ 底盘
                                 │        └→ CollisionMonitorState（监控）
```

## 五、与其他包的关系

- 不依赖 costmap / BT，独立进程；通常链路：controller → velocity_smoother → **collision_monitor** → 底盘
- 与 costmap ObstacleLayer 的区别：costmap 服务于规划（有延迟），collision monitor 是指令级紧急安全（毫秒级）
