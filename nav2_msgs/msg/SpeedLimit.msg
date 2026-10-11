# 速度限制消息
# 适用场景：由限速相关的 costmap filter 或外部节点发布给控制器（controller），
# 使机器人在进入限速区等场景时动态执行限速
std_msgs/Header header
# 为 true 时 speed_limit 表示百分比限速，为 false 时表示绝对值限速
bool percentage
# 允许的最大速度（根据 percentage 的取值，为最大速度的百分比或 m/s 绝对值）
# 无限制时设置为 0.0
float64 speed_limit
