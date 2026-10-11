# 三维体素栅格消息
# 适用场景：由 nav2_voxel_grid / costmap 的 voxel layer 发布，
# 表示三维空间中的障碍物占用情况（可在 RViz 中进行三维可视化）
std_msgs/Header header
uint32[] data                     # 体素数据，按一维数组编码的三维栅格占用标记
geometry_msgs/Point32 origin      # 体素栅格原点在真实世界中的位置
geometry_msgs/Vector3 resolutions # x/y/z 三个方向的体素分辨率（米/体素）
uint32 size_x                     # x 方向体素数量
uint32 size_y                     # y 方向体素数量
uint32 size_z                     # z 方向体素数量
