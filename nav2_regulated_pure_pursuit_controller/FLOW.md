# nav2_regulated_pure_pursuit_controller 整体流程

## 一、包定位

几何式局部控制器：**Regulated Pure Pursuit（RPP）**。在经典纯跟踪基础上增加速度调节（曲率、障碍距离、接近目标时减速），并支持倒车，代码简洁，适合作为几何控制器入门。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `RegulatedPurePursuitController` | `regulated_pure_pursuit_controller.cpp` | 主插件（`nav2_core::Controller`） |

## 三、主流程

```
computeVelocityCommands(pose, velocity, goal_checker):
    1. 路径预处理: transformPOSE 到机器人坐标系, 剪掉身后已走过的部分
    2. 找 lookahead 点（按 desired_lookahead_dist 在路径上取点；
       支持自适应 lookadad 距离/曲率缩放）
    3. 曲率计算: curvature = 2*y_l / l²（纯跟踪几何公式）
    4. 线速度调节（regulated）:
        ├─ curvature_regulated: 曲率大（急弯）→ 按 reg_scaling 线性降速
        ├─ cost_regulated: 距障碍近（inflation 代价/距离查询）→ 降速
        ├─ approach_regulated: 接近路径终点 → 按 sqrt(2*accel*dist) 减速
        └─ 受 min/max/allowed_wild 起停约束
    5. 角速度: w = v * curvature（纯跟踪转向律）
    6. 接近终点且 use_rotate_to_heading → 原地对准目标朝向
    7. 支持倒车: 检测路径朝向 → 反转符号
    8. 返回 cmd_vel
```

## 四、数据流

```
path → 找 lookadad 点 → 曲率 → (调节后) v 与 w → cmd_vel
costmap（障碍距离查询，用于降速）
```

## 五、与其他包的关系

- 插件被 `nav2_controller` 加载
- 常与 `nav2_rotation_shim_controller` 组合（先转向对准路径再跟踪）
- 对照阅读：`nav2_graceful_controller`（同为几何式，控制律不同）
