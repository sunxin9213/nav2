# nav2_system_tests 整体流程

## 一、包定位

nav2 的**端到端集成测试**包：用 launch 拉起完整导航栈 + 仿真环境，通过 Python/C++ 测试脚本验证各能力（规划、控制、恢复、航点等）在真实运行拓扑下是否正常，是 CI 的主力。

## 二、组件结构

| 目录/文件 | 内容 |
|---|---|
| `src/test_planning_costmaps.py`?（等 `test_*.py` / `*.cpp`） | 各能力测试：planning（成本图/插件）、controlling（各控制器）、recovery（恢复行为）、behaviors、waypoint_follower、bt_navigator、route 等 |
| `models/` | 测试用机器人 URDF/SDF |
| `launch/`? + 各测试自带 launch | 每个测试独立拉起 bringup 子集 |
| `scripts/`? | 辅助脚本 |

## 三、单个测试的标准流程

```
1. 启动阶段:
   pytest 测试类 (setUpClass)
     └─ launch 完整导航栈（复用 nav2_bringup 的 launch/params）
        + 测试专用 costmap/参数/机器人模型
2. 准备阶段:
   等待 nav2 完全 ACTIVE（lifecycle manager 状态查询）
   设置初始位姿（amcl）
3. 执行阶段:
   BasicNavigator / action 客户端下发目标
   （如: goToPose → 等待完成 / 主动注入场景考验恢复行为）
4. 断言阶段:
   机器人到达目标位置（TF 查询位姿 vs 目标误差阈值）
   或 action 结果符合预期（成功/超时/错误码）
5. 收尾: teardown 销毁进程组
```

## 四、测试组织方式

```
按能力分目录/文件:
  planning  ──→ 各规划器在不同 costmap 下的路径可行性
  controlling ─→ 各控制器在标准场景下的跟踪效果（到达误差/用时）
  recovery  ──→ 阻塞场景下 Spin/BackUp/清除 costmap 是否脱困
  behaviors / waypoints / navigator / route …
```

## 五、与其他包的关系

- 依赖 `nav2_bringup`（环境）、`nav2_simple_commander`（Python API）
- 阅读价值：想快速了解"某个能力该如何被验证/参数如何配置"时，此处是最好的示例库；也常用于复现 issue 的最小环境
