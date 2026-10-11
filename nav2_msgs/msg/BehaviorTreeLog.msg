# 行为树日志消息：批量记录一段时间内行为树中节点状态的变化事件
# 适用场景：由 bt_navigator 定期发布，用于监控、调试或离线回放行为树的执行过程
builtin_interfaces/Time timestamp    # 本条日志消息发出的 ROS 时间戳
BehaviorTreeStatusChange[] event_log # 自上次日志发布以来的节点状态变化事件列表
