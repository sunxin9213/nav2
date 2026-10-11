# 二维栅格代价地图：每个栅格单元关联一个代价值
# 适用场景：GetCostmap 服务返回的完整代价地图数据，用于获取/传输 costmap 的内容

std_msgs/Header header

# 代价地图的元数据信息（尺寸、分辨率、原点等）
CostmapMetaData metadata

# 代价数据，按行优先（row-major）顺序排列，从 (0,0) 单元格开始
uint8[] data
