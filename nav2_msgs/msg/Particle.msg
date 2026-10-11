# 粒子滤波器产生的单个粒子（位姿及其权重）
# 适用场景：AMCL 粒子滤波中的单个采样粒子，用于组成 ParticleCloud 粒子云

geometry_msgs/Pose pose # 该粒子代表的机器人位姿假设

float64 weight          # 粒子权重，表示该位姿假设的可信程度
