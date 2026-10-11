# nav2_common 整体流程

## 一、包定位

公共构建工具包，提供 launch 复用宏、CMake 宏和少量通用 C++ 头文件，**不含运行时节点**。

## 二、核心组件

| 组件 | 位置 | 作用 |
|---|---|---|
| `RewrittenYaml` | `launch/` (Python) | launch 启动时按规则替换 YAML 参数文件中的值（如 use_sim_time、命名空间替换），是 bringup 参数复用的关键 |
| `node_utils` | `launch/` (Python) | launch 中复用的节点启动辅助函数 |
| CMake 模块 | `cmake/` | 查找三方依赖（如 FindGLOG 等）供其他包使用 |
| 通用工具头 | `include/nav2_common/` | 少量跨包共享的内联工具 |

## 三、使用流程

```
nav2_bringup (launch)
    │  使用 RewrittenYaml 生成最终参数文件
    ▼
各功能包 (CMake)
    │  使用 cmake 模块找到三方依赖
    ▼
编译期工具，运行时不参与
```

## 四、与其他包的关系

- 依赖链底层包；主要为 `nav2_bringup` 和各功能包的构建过程服务。
