# nav2_mppi_controller 整体流程

## 一、包定位

nav2 当前**主推**的局部控制器：MPPI（Model Predictive Path Integral）——基于 GPU/CPU 并行的轨迹采样优化，软选择（加权平均）控制序列，天然平滑、支持任意运动学模型。

## 二、核心组件（`src/`）

| 组件 | 文件 | 职责 |
|---|---|---|
| `MPPIController` | `controller.cpp` | 主插件（`nav2_core::Controller`），组装各部件 |
| `Optimizer` | `optimizer.cpp` | MPPI 核心：控制序列迭代优化（采样→仿真→打分→加权平均→shift） |
| `NoiseGenerator` | `noise_generator.cpp` | 对当前控制序列叠加噪声批量生成候选（cuda/方差异构） |
| `Critics` | `critics/` | 评分器插件，见下表 |
| `CriticManager` | `critic_manager.cpp` | 加载并按序调用 critics |
| `PathHandler` | `path_handler.cpp` | 路径预处理：截断、坐标系变换、局部路径段维护 |
| `ParametersHandler` | `parameters_handler.cpp` | 动态参数管理（支持运行时回调更新） |
| `TrajectoryVisualizer` | `trajectory_visualizer.cpp` | 候选轨迹可视化 |

### critics 一览

`obstacles_critic`（障碍）、`goal_critic`（终点距离）、`goal_angle_critic`（终点朝向）、`path_align_critic`（路径对齐）、`path_angle_critic`（朝向对齐）、`path_follow_critic`（路径跟随）、`prefer_forward_critic`（前向偏好）、`velocity_deadband_critic`（速度死区）、`constraint_critic`（运动学约束）

## 三、主流程

```
computeVelocityCommands(pose, velocity, goal_checker):
    1. PathHandler: 转换路径到代价/机器人帧，截取局部段
    2. Optimizer 一个优化循环（每周期迭代多次）:
        a. NoiseGenerator: 在当前控制序列(control_trajectory)上叠加
           噪声 → batch_size × time_steps 个候选序列
        b. 对每个候选序列前向仿真（运动模型积分）得到轨迹
        c. CriticManager: 所有 critic 对轨迹打分（加权）
        d. softmax(γ^(-S)) 权重 → 对全部候选序列做加权平均
           → 得到新的"最优"控制序列（软选择，天然平滑）
        e. 序列前移一步（shift），下一周期继续滚动优化
    3. 取序列首元素作为 cmd_vel 输出
    4. 视化候选/最优轨迹（debug 模式）
    5. 全部轨迹不可行 → 抛异常 → action 失败
```

## 四、数据流

```
path + costmap + odom
   └─ 噪声采样控制序列 → 前向仿真轨迹 → critics 打分
        → softmax 加权平均 → 滚动更新 → cmd_vel
```

## 五、与其他包的关系

- 插件被 `nav2_controller` 加载；也是理解"优化式"控制器的样板
- 与 DWB 对比：采样在**控制序列**上（而非单步），且是加权平均而非最优硬选择
