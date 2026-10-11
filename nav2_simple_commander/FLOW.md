# nav2_simple_commander 整体流程

## 一、包定位

Python 上层 API：`BasicNavigator` 封装 nav2 全部 action/srv 客户端，让业务脚本几行代码即可完成导航任务，**不参与运行时系统本身**。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `BasicNavigator` | `basic_navigator.py` | 封装 NavigateToPose / NavigateThroughPoses / FollowWaypoints / 各 srv 的客户端与等待逻辑 |
| 示例脚本 | `examples/*.py` | go_to_pose、follow_waypoints、pick and place demo 等用法演示 |
| `line_iterator.py` | `line_iterator.py` | 栅格直线遍历小工具 |

## 三、典型使用流程

```
import nav2_simple_commander...
nav = BasicNavigator()

1. 等待就绪:  nav.waitUntilNav2Active(localizer=…, navigator=…)
      └─ 轮询 lifecycle manager 状态 + amcl / map_server / bt_navigator
2. 设置初始位姿（可选）: nav.setInitialPose(pose)
3. 单点导航:  nav.goToPose(pose) → nav.isTaskComplete() 轮询
4. 多点导航:  nav.followWaypoints(poses)
5. 等待/查询: nav.getFeedback() / nav.getResult() / nav.cancelTask()
6. 其他封装:
      nav.getPathPose() / nav.goThroughPoses() / nav.spin() / nav.backup()
      nav.clearLocalCostmap() / nav.clearGlobalCostmap()
      nav.saveMap() / nav.loadMap()（封装 map_server 服务）
```

## 四、数据流

```
Python 业务脚本 ──BasicNavigator──→ bt_navigator / waypoint_follower / map_server / lifecycle_manager
                     └─ rclpy action/srv 客户端（异步轮询模式）
```

## 五、与其他包的关系

- 只依赖 `nav2_msgs`（生成的 Python 接口）+ rclpy；是上层应用/测试使用 nav2 的最直观范本
- `nav2_system_tests` 中的大量 Python 测试即基于同类模式编写
