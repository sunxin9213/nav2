# nav2_dwb_controller 整体流程

## 一、包定位

传统"采样-评分"式局部控制器（ROS1 DWA 的继承者）：在一个周期内采样大量候选速度指令、前向仿真得到轨迹、逐条打分取最优。是理解局部规划"评分范式"的经典实现。

## 二、子包结构

| 子包 | 内容 |
|---|---|
| `dwb_core` | `DWBLocalPlanner`（主插件）+ 轨迹生成/评分接口定义 |
| `dwb_plugins` | 轨迹生成器：`StandardTrajectoryGenerator`、`XYThetaIterator`、`LimitedAccelGenerator`、`KinematicParameters`（速度/加速度采样范围） |
| `dwb_critics` | 评分器（critic）插件集合，见下表 |
| `nav_2d_utils` / `nav_2d_msgs` | 2D 消息/工具兼容层 |

### dwb_critics 一览

| Critic | 惩罚什么 |
|---|---|
| `obstacle_footprint` / `base_obstacle` | 轨迹撞上障碍/穿致命栅格 |
| `map_grid`（`goal_dist`/`path_dist`/`goal_align`/`path_align`） | 偏离路径/目标的栅格距离场惩罚 |
| `prefer_forward` | 后退 |
| `rotate_to_goal` | 接近目标时的姿态对齐 |
| `twirling` | 原地乱转 |
| `oscillation` | 震荡 |
| `constrained` | 超出运动学约束 |

## 三、主流程

```
computeVelocityCommands(pose, velocity, goal_checker):
    1. 轨迹生成器: 在 (vx, vy, w) 采样空间内遍历候选指令
       （受 KinematicParameters 的 min/max 速度、加速度限制）
    2. 对每个候选指令: forward simulate（按 sim_time、仿真模型积分出轨迹）
    3. 逐个 critic 评分（加权求和，w_scale*score）
        score = Σ critic->scoreTrajectory(traj)
    4. 最优轨迹对应的指令 → 作为 cmd_vel 返回
    5. 所有轨迹都非法（全被 critic 否决）→ 抛异常 → action 失败
```

## 四、数据流

```
path + costmap + 当前速度
    └─ 采样 vx/vy/w → 前向仿真轨迹 → critics 打分 → 最优 cmd_vel
```

## 五、与其他包的关系

- 作为 `nav2_core::Controller` 插件被 `nav2_controller` 加载
- 与 mppi 对照阅读：DWB 每 8周期 换一次最优（硬选择），MPPI 是加权平均（软选择）
