# nav2_costmap_2d 整体流程

## 一、包定位

nav2 的**核心数据基础设施**：维护一张带代价的 2D 栅格地图，由多个"图层（Layer）"叠加而成，供规划器和控制器消费障碍/膨胀信息。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `Costmap2D` | `costmap_2d.cpp` | 纯数据结构：栅格数组、分辨率、原点、坐标/world 互转、复制/Resize |
| `LayeredCostmap` | `layered_costmap.cpp` | 图层编排器：管理 Layer 列表，驱动 updateBounds→updateCosts |
| `Layer` (基类) | `layer.cpp` | 图层抽象（继承 `nav2_core::CostmapLayer`），定义生命周期与更新接口 |
| `Costmap2DROS` | `costmap_2d_ros.cpp` | ROS 封装：lifecycle 节点、参数、地图更新定时器、TF、发布 |
| 各图层插件 | `plugins/` | 见下表 |
| 代价地图过滤器 | `plugins/costmap_filters/` | keepout / speed filter（配合 CostmapFilterInfo） |
| `ClearCostmapService` | `costmap_2d_publisher.cpp` 同目录 | 提供 ClearCostmap* 三个服务 |

### 图层插件一览

| 图层 | 作用 |
|---|---|
| `StaticLayer` | 从 map 话题载入静态地图 |
| `ObstacleLayer` | 订阅 scan/pointcloud 做障碍标记与射线清除 |
| `VoxelLayer` | 3D 版障碍层（依赖 nav2_voxel_grid） |
| `InflationLayer` | 按指数衰减模型对障碍膨胀，产生 cost 0~254 |
| `RangeSensorLayer` | 声呐/红外等单点测距传感器图层 |
| `DenoiseLayer` | 障碍去噪（去除孤立噪点） |

## 三、主运行流程

```
on_configure
    ├─ 声明参数（分辨率、机器人半径、图层列表 plugins、filters 等）
    ├─ 创建 LayeredCostmap
    └─ 逐个加载 layer 插件并调用 layer->onInitialize()
on_activate
    └─ 启动 map update loop（定时器，默认 10Hz）
每个更新周期:
    1. 遍历所有 layer: layer->updateBounds()   ← 求本层有改动的区域并扩展 bounds
    2. 聚合所有 bounds 的并集
    3. 遍历所有 layer: layer->updateCosts()    ← 把本层数据写入主栅格（叠加规则：
                                                  覆盖/取最大值/按 costmap_filter 语义）
    4. Costmap2DPublisher 发布 Costmap/OccupancyGrid（可视化和下游使用）
```

## 四、数据流

```
map_server ──(map)──→ StaticLayer ─┐
scan/pointcloud ────→ ObstacleLayer ┤        ┌→ planner_server（全局 costmap）
                                    ├─ 主栅格 ├→ controller_server（局部 costmap）
footprint/robot ───→ （全局/局部区别：┘        └→ RViz 可视化
   全局: 全图+跟随机器人滚动；局部: 滑动窗口）

CostmapFilterInfo + filter mask ──→ keepout_filter（写致命代价）/ speed_filter（发 SpeedLimit）
```

## 五、与其他包的关系

- 依赖：`nav2_voxel_grid`（VoxelLayer）、`nav2_msgs`、`nav2_util`
- 被依赖：planner / controller / collision_monitor 等几乎所有算法包
- 全局与局部 costmap 是同一套代码，靠参数（rolling_window 等）区分
