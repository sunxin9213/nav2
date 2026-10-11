# 行为树中单个节点的一次状态变化事件
# 适用场景：作为 BehaviorTreeLog 的子消息，记录行为树中某个节点状态切换的细节
builtin_interfaces/Time timestamp  # 该事件发生的时间戳，通常为系统墙上时钟时间（wall clock time）
string node_name                   # 发生状态变化的节点名称
string previous_status             # 变化前的节点状态：IDLE、RUNNING、SUCCESS 或 FAILURE
string current_status              # 变化后的节点状态：IDLE、RUNNING、SUCCESS 或 FAILURE
