# 路由图中某条边的代价调整信息
# 适用场景：配合 nav2_msgs/srv/DynamicEdges 服务使用，动态调整路由（route）图中边的通行代价
uint16 edgeid # 边的 ID
float32 cost  # 该边调整后的代价值
