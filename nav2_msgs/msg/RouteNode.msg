# 路由图中的一个节点
# 适用场景：作为 Route.msg 的子消息，描述路由图节点的 ID 及其空间位置
uint16 nodeid                # 节点的 ID
geometry_msgs/Point position # 节点在图中的空间坐标位置
