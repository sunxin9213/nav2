# 代价地图过滤器信息：由代价地图过滤器（costmap filter）发布，说明过滤器的类型与换算参数
# 适用场景：keepout（禁行区）/限速等过滤器向 costmap 发布过滤掩码配置，
# 使 costmap 能够根据掩码计算安全区或速度限制
std_msgs/Header header
# 所使用过滤器插件的类型
# 0: keepout/lanes filter
# 1: speed limit filter in % of maximum speed
# 2: speed limit filter in absolute values (m/s)
uint8 type
# 过滤掩码（filter mask）话题的名称
string filter_mask_topic
# 将 OccupancyGrid 数据转换到过滤空间的基准偏移量和乘数系数
# 用于把占用栅格数据值转换到其他数值空间：
# space = data * multiplier + base
float32 base
float32 multiplier
