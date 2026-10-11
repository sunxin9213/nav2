# nav2_lifecycle_manager 整体流程

## 一、包定位

导航系统"总开关"：按依赖顺序统一启动/停止/重置十余个 lifecycle 节点，并提供 bond 心跳保活机制。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `LifecycleManager` | `lifecycle_manager.cpp` | 管理节点列表，实现 startup/shutdown 等流程，暴露 `nav2_msgs/srv/ManageLifecycleNodes` 服务 |
| `bond` 心跳 | 同上 | activate 后创建 bond，被管理节点断心跳即触发自动关闭整组节点 |

## 三、核心流程（startup）

```
main: 读取 node_names（按依赖顺序）+ autostart 参数
    │
startup():
    for 每个 node（顺序执行）:
        1. lifecycle_client.configure()   ← 等待 UNINACTIVE
        2. createBond()                   ← 建立心跳连接
        3. lifecycle_client.activate()    ← 等待 ACTIVE
    全部成功 → 服务/话题就绪，导航可用

对应命令（ManageLifecycleNodes 服务）:
    STARTUP(0)  按序 configure+activate 全部节点
    PAUSE(1)    逆序 deactivate 全部节点
    RESUME(2)   按序 activate 全部节点
    RESET(3)    逆序 deactivate → cleanup → 再 configure+activate
    SHUTDOWN(4) 逆序 deactivate → cleanup → shutdown
```

## 四、心跳保护流程

```
任意被管理节点崩溃 → bond 断开
    └─ LifecycleManager 检测到 bond broken
        └─ 触发 shutdown 或重启整组（参数 bond_timeout / attempt_respawn_reconnection）
```

## 五、与其他包的关系

- 通过 `nav2_util::LifecycleServiceClient` 远程调用各节点的 lifecycle service（不直接链接各包）。
- `nav2_bringup` 中 launch 两个 manager：`lifecycle_manager_localization`（map_server + amcl）和 `lifecycle_manager_navigation`（导航全家桶）。
- 是理解"nav2 为什么跑起来要先 activate"的钥匙。
