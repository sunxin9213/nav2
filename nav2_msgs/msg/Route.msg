# 路由消息：在路由图上计算得到的一条完整路由
# 适用场景：由 nav2_route 发布，表示从起点到终点依次经过的节点和边序列，
# 用于 ComputeRoute/ComputeAndTrackRoute 动作的结果及路由跟踪
std_msgs/Header header
float32 route_cost    # 整条路由的总代价
RouteNode[] nodes     # 路由上有序排列的节点集合
RouteEdge[] edges     # 连接各节点的有序边集合
