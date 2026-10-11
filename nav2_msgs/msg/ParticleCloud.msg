# 粒子云消息：包含一组粒子的位姿和权重
# 适用场景：由 AMCL 发布，用于可视化/查看粒子滤波定位结果的分布情况

std_msgs/Header header

# 粒子云中的粒子数组
Particle[] particles
