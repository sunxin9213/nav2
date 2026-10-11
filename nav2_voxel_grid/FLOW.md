# nav2_voxel_grid 整体流程

## 一、包定位

三维体素栅格数据结构库，**无节点**，为 `nav2_costmap_2d` 的 `VoxelLayer` 提供 3D 障碍存储。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `VoxelGrid` | `voxel_grid.cpp` / `.hpp` | 体素栅格本体：列式存储，每列 32bit（z 方向最多 32 层） |
| `VoxelStatus` | 同上 | 三态：`MARKED`（占用）/ `FREE`（空闲）/ `UNKNOWN` |

## 三、核心流程（mark & clear）

```
传感器一帧点云
    │
    ├─ markVoxel: 命中障碍 → 将对应 (x,y) 列的 z 位 置 1
    │
    └─ clearVoxel / 射线清除（Bresenham 直线遍历）:
        从传感器原点沿视线方向逐列清位
        （和 ObstacleLayer 的 2D raytrace 等价的 3D 版）

查询:
    getVoxel(x,y,z) / getVoxelColumn(x,y) → MARKED / FREE / UNKNOWN
```

## 四、数据布局

```
data: uint32 数组，长度 size_x * size_y
每个 uint32 的 bit i 表示该列高度 i 的体素是否被占用
```

## 五、与其他包的关系

- 被 `nav2_costmap_2d`（VoxelLayer）依赖；`nav2_msgs/VoxelGrid` 消息用于其 RViz 可视化。
- 包很小，读懂 mark/clear 即可。
