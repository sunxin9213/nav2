# nav2_behavior_tree 整体流程

## 一、包定位

行为树（BT）**节点插件库** + BT 执行基础设施。提供把 nav2 各服务器 action/srv 封装成 BT 节点的全部积木，供 `nav2_bt_navigator` 组装成导航行为树。

## 二、核心组件

### 1. BT 基础设施

| 组件 | 职责 |
|---|---|
| `BtActionServer` (`bt_action_server.hpp`) | 封装 nav2_util::SimpleActionServer + BehaviorTree：收 action goal → 每周期 tick 树 → 反馈 |
| `BtServiceNode` / `BtServiceCancelNode` | 把 ROS service 调用封装为 BT 节点 |
| `bt_conversions` / `bt_utils` | BT 端口与 ROS 消息类型转换、XML 加载工具 |

### 2. BT 节点插件（`plugins/`）

| 类别 | 代表节点 | 作用 |
|---|---|---|
| **action** (30个) | `ComputePathToPoseAction`、`FollowPathAction`、`SpinAction`、`ClearCostmapAction`… | 异步调用各服务器 action，结果写入黑板 |
| **condition** (12个) | `GoalReached`、`IsStuck`、`PathExpiringTimerCondition`、`WouldAControllerRecoveryHelp`… | 判断当前态势，决定树走向 |
| **control** | `PipelineSequence`、`RecoveryNode`、`RecoveryFallback` | 自定义控制节点：规划→执行流水线、失败时依次尝试恢复行为 |
| **decorator** | `RateController`、`DistanceController`、`GoalUpdater`、`SpeedController`、`SingleTriggerNode` | 控制子节点触发频率 / 动态改目标 |

## 三、典型执行流程

```
BtActionServer 收到 NavigateToPose goal
    └─ 加载 BT XML（树结构）
        └─ 每个周期 tick:
              PipelineSequence
               ├─ RateController(1Hz) → ComputePathToPoseAction   ← 异步等规划结果
               │                        └─ 结果写黑板 "path"
               ├─ DistanceController → GoalUpdater
               └─ FollowPathAction ← 读黑板 "path"，跟踪路径
            任一环节 FAILURE
               └─ RecoveryNode: 依次尝试 ClearCostmap → Spin → BackUp → Wait
```

## 四、数据流

```
黑板(blackboard) 是节点间数据总线:
    goal(POSE) ──→ ComputePathToPose ──→ path(PATH) ──→ FollowPath
                                            └──→ SmoothPath（可选）
```

## 五、与其他包的关系

- 依赖 `nav2_util`、`nav2_msgs`、BehaviorTree.CPP 三方库
- 被 `nav2_bt_navigator` 加载使用；默认树 XML 在 `nav2_bt_navigator/behavior_trees/`
- 阅读入口：`PipelineSequence` + `RecoveryNode` + 任意一个 `*Action` 节点
