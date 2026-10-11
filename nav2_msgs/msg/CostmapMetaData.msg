# 代价地图的基本特征元数据
# 适用场景：作为 Costmap.msg 的组成部分以及 GetCostmap 服务的请求参数，描述代价地图的规格信息

# 静态地图加载的时间
builtin_interfaces/Time map_load_time

# 代价地图最后一次更新的时间
builtin_interfaces/Time update_time

# 对应的图层名称
string layer

# 地图分辨率 [m/cell]（每个栅格单元对应的实际距离，单位米）
float32 resolution

# 水平方向（x 方向）的栅格数量
uint32 size_x

# 垂直方向（y 方向）的栅格数量
uint32 size_y

# 代价地图的原点 [m, m, rad]
# 即地图中栅格 (0,0) 在真实世界中的位姿
geometry_msgs/Pose origin
