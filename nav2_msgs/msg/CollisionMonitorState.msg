# 碰撞监测器（Collision Monitor）当前触发状态的消息
# 适用场景：由 collision_monitor 发布，用于监控当前哪个安全多边形被触发以及采取了何种安全动作

# 碰撞监测器可执行的动作类型
uint8 DO_NOTHING=0 # 无动作（未触发任何安全区域）
uint8 STOP=1       # 立即停止机器人
uint8 SLOWDOWN=2   # 按当前运行速度的百分比减速
uint8 APPROACH=3   # 在碰撞前保持恒定的时间间隔（按时间接近减速）
uint8 action_type  # 当前触发的动作类型（取值为上述常量之一）

# 触发的安全多边形名称
string polygon_name
