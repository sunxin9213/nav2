# nav2_map_server 整体流程

## 一、包定位

地图服务包：加载/保存占用栅格地图（OccupancyGrid），为 StaticLayer 与定位提供地图数据。也提供处理地图文件的 IO 库。

## 二、核心组件

| 组件 | 文件 | 职责 |
|---|---|---|
| `MapServer` | `map_server/map_server.cpp` | lifecycle 节点：加载地图并发布 `map` 话题 + `load_map` 服务 |
| `MapSaver` | `map_saver/map_saver.cpp` | lifecycle 节点：订阅 `map` 保存为文件 + `save_map` 服务（也可独立 CLI 运行） |
| `loadMapFromFile` / `saveMapToFile` | `map_io.cpp` | 地图 IO：YAML 元数据 + 图像(pgm/png/bmp) ↔ OccupancyGrid 互转（含三值/scale/raw 模式与阈值处理） |

## 三、MapServer 运行流程

```
on_configure
    ├─ 读取 yaml_path 参数
    ├─ loadMapFromFile(): 读 YAML 元数据 → 读图像 → 按阈值(threshold_occupied/free)
    │   二值化为 {0(空闲), 100(占用), -1(未知)}
    └─ 创建 latched map publisher + load_map 服务
on_activate
    └─ 发布一次 map（latched，后来者也能收到）
load_map 服务 (nav2_msgs/srv/LoadMap)
    └─ 按请求的 map_url 重新加载并返回结果码
```

## 四、MapSaver 运行流程

```
save_map 服务 / CLI
    └─ 订阅接收一帧 map
        └─ saveMapToFile(): 按 map_mode(free_thresh/occupied_thresh)
            写出 图像文件 + YAML 元数据
```

## 五、数据流

```
磁盘(yaml+pgm) ──MapServer──→ map 话题 ──→ costmap StaticLayer / 定位 / RViz
map 话题 ──MapSaver──→ 磁盘(yaml+图像)
```

## 六、与其他包的关系

- 依赖 `nav2_util`（lifecycle）、`nav2_msgs`（LoadMap/SaveMap）
- 被 `nav2_bringup` 拉起；是整个导航系统的地图来源。
