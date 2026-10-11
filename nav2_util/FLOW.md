# nav2_util 整体流程

## 一、包定位

基础工具库（header-mostly + 少量 cpp），为所有功能包提供生命周期、动作服务、TF、里程计等通用能力。**所有包的共同依赖**。

## 二、核心组件

| 组件 | 头文件 | 作用 |
|---|---|---|
| `LifecycleNode` | `lifecycle_node.hpp` | 封装 rclcpp_lifecycle 节点，提供 on_configure/on_activate/on_deactivate/on_cleanup 接口，是各服务器节点的基类 |
| `LifecycleServiceClient` | `lifecycle_service_client.hpp` | 远程调用其他节点的 lifecycle service（lifecycle_manager 用） |
| `SimpleActionServer` | `simple_action_server.hpp` | 轻量 action 服务端模板（单目标、可抢占），planner/controller/smoother/behaviors 等服务器全部基于它 |
| `OdomSmoother` | `odom_smoother.hpp` | 订阅 odom 并维护滑动窗口，输出平滑后的速度/角速度，供控制器和碰撞监测使用 |
| TF 工具 | `tf_utils.hpp` | TF 帮助函数（等待变换、位姿转换等） |
| 节点工具 | `node_utils.hpp` | 参数声明（declare_parameter_if_not_declared）、CallbackGroup 管理等 |
| 几何/字符串工具 | `geometry_utils.hpp` / `string_utils.hpp` / `robot_utils.hpp` | 距离计算、字符串分割、时间戳处理等小工具 |
| `NodeThread` | `node_thread.hpp` | 为回调组单独开线程执行 |

## 三、典型使用流程（以 SimpleActionServer 为例）

```
功能包节点继承 LifecycleNode
    │
    ├─ on_configure: 声明参数、创建 publishers/subscribers
    ├─ on_activate:  激活 SimpleActionServer，开始接受 goal
    │                    │ 收到 goal ─→ 回调 (如 PlannerServer::computePlan)
    │                    │ 执行期间可 publish feedback
    │                    └─ 成功/失败 ─→ 返回 result
    └─ on_deactivate: 停止接受 goal
```

## 四、与其他包的关系

- 被 `nav2_core`、所有服务器包、`nav2_behavior_tree` 依赖。
- 阅读各服务器代码前，先看 `simple_action_server.hpp` 和 `lifecycle_node.hpp` 能省很多事。
