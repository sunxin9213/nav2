# 路由图中的一条边
# 适用场景：作为 Route.msg 的子消息，描述连接两个节点的边及其几何端点
uint16 edgeid             # 边的 ID
geometry_msgs/Point start # 边起点坐标
geometry_msgs/Point end   # 边终点坐标
