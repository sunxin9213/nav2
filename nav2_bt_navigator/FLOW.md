# nav2_bt_navigator 整体流程

## 一、包定位

导航任务编排核心：对外提供 `NavigateToPose` / `NavigateThroughPoses` 两个 action，对内驱动行为树（由 `nav2_behavior_tree` 积木组成）完成"规划→跟踪→恢复"闭环。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `Navigator`（模板基类） | `bt_navigator.hpp/.cpp` | 通用逻辑：加载 BT XML、注册插件、BtActionServer、默认 BT 生成、goal 校验 |
| `NavigateToPoseNavigator` | `navigate_to_pose_navigator.hpp/.cpp` | 单点导航：预置默认树 `navigate_to_pose_w_replanning_and_recovery.xml` |
| `NavigateThroughPosesNavigator` | `navigate_through_poses_navigator.hpp/.cpp` | 多点导航：预置默认树 `navigate_through_poses_w_replanning_and_recovery.xml` |
| 默认行为树 XML | `behavior_trees/*.xml` | 官方默认导航树（带重规划+恢复），**读代码前先读这两个 XML** |

## 三、主流程

```
on_configure
    ├─ 加载 BT 插件库（nav2_behavior_tree 全部节点）
    ├─ 读取 default_nav_to_pose_bt_xml / ..._through_poses_... 参数
    └─ 初始化 BtActionServer<action::NavigateToPose>

收到 NavigateToPose goal
    ├─ goal->behavior_tree 为空 → 用默认树 XML；否则用用户指定 XML
    ├─ 校验 goal（帧、时间戳）
    └─ BtActionServer 循环:
          每 10ms tick 一次行为树:
            ComputePathToPose ─→ (SmoothPath) ─→ FollowPath
                 ↑ RateController 周期重规划          │
                 └──── 失败时 RecoveryNode: 恢复行为 ←┘
          周期发布 feedback（当前位姿、剩余距离、恢复次数、耗时…）

多目标 NavigateThroughPoses 区别:
    goal 校验后维护 poses 队列 → 数量>1 时使用 ThroughPoses 规划
    → 仅剩最后一个点时退化为 ToPose 规划
```

## 四、数据流

```
用户/上层 ──NavigateToPose──→ bt_navigator ──树节点──→ planner/controller/behavior/smoother servers
                                  │
                                  └──feedback──→ 用户（也可发 /behavior_tree_log 供 RViz 显示）
```

## 五、与其他包的关系

- 依赖 `nav2_behavior_tree`（节点库）、`nav2_util`；被 `nav2_waypoint_follower` 调用
- 阅读入口：默认 XML → `Navigator::initialize` → `BtActionServer` 的 goal callback
