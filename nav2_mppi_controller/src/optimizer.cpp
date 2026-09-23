// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_mppi_controller/optimizer.hpp"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

// 中文说明（文件级）：
// 本文件实现 mppi::Optimizer，即 MPPI（模型预测路径积分控制）的核心优化器。
// 算法主循环（见 evalControl / optimize）：
//   噪声采样 → 运动模型推演（rollout）→ critic 评分 → softmax 加权更新控制序列
// 数学要点（见 updateControlSequence）：
//   代价中附加控制量与噪声的内积正则项（信息论项，来自路径积分理论），
//   随后按 w_i = exp(-(J_i - min J)/T) / Σ exp(...) 计算软最大权重，
//   新控制序列 = Σ w_i * (u_i + ε_i)，即所有噪声控制序列的加权平均。

namespace mppi
{

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

// 初始化优化器：由 MPPIController::configure 调用（生命周期 on_configure）
void Optimizer::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  ParametersHandler * param_handler)
{
  // 保存节点、名称、代价地图与参数处理器的引用
  parent_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  parameters_handler_ = param_handler;

  auto node = parent_.lock();
  logger_ = node->get_logger();

  // 读取并解析全部 MPPI 超参数（含运动模型选择）
  getParams();

  // 配置 critic 管理器（加载各代价函数插件）与噪声生成器（启动后台线程）
  critic_manager_.on_configure(parent_, name_, costmap_ros_, parameters_handler_);
  noise_generator_.initialize(settings_, isHolonomic(), name_, parameters_handler_);

  // 分配并清零所有张量，恢复初始状态
  reset();
}

// 关闭：停止噪声生成线程（进程结束时由析构函数调用）
void Optimizer::shutdown()
{
  noise_generator_.shutdown();
}

// 读取优化器全部参数（带默认值）
void Optimizer::getParams()
{
  std::string motion_model_name;

  auto & s = settings_;
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter("");
  // model_dt：单步推演时长（秒）；time_steps：预测时域步数；
  // batch_size：并行采样轨迹数；iteration_count：每周期迭代次数
  getParam(s.model_dt, "model_dt", 0.05f);
  getParam(s.time_steps, "time_steps", 56);
  getParam(s.batch_size, "batch_size", 1000);
  getParam(s.iteration_count, "iteration_count", 1);
  // temperature：softmax 温度；gamma：信息论正则项系数
  getParam(s.temperature, "temperature", 0.3f);
  getParam(s.gamma, "gamma", 0.015f);
  // 控制约束（速度上下限）与采样噪声标准差
  getParam(s.base_constraints.vx_max, "vx_max", 0.5);
  getParam(s.base_constraints.vx_min, "vx_min", -0.35);
  getParam(s.base_constraints.vy, "vy_max", 0.5);
  getParam(s.base_constraints.wz, "wz_max", 1.9);
  getParam(s.sampling_std.vx, "vx_std", 0.2);
  getParam(s.sampling_std.vy, "vy_std", 0.2);
  getParam(s.sampling_std.wz, "wz_std", 0.4);
  // 优化失败（所有轨迹碰撞）时的重试上限
  getParam(s.retry_attempt_limit, "retry_attempt_limit", 1);

  // 运动模型：DiffDrive（差速）/ Omni（全向）/ Ackermann（阿克曼）
  getParam(motion_model_name, "motion_model", std::string("DiffDrive"));

  s.constraints = s.base_constraints;
  setMotionModel(motion_model_name);
  // 动态参数变更后（回调触发时）重置优化器状态
  parameters_handler_->addPostCallback([this]() {reset();});

  // 读取 controller_server 的控制频率（静态参数），
  // 用于决定控制序列是否需要滚动（shift）到下一周期
  double controller_frequency;
  getParentParam(controller_frequency, "controller_frequency", 0.0, ParameterType::Static);
  setOffset(controller_frequency);
}

// 根据控制频率与推演步长决定是否启用控制序列滚动
void Optimizer::setOffset(double controller_frequency)
{
  const double controller_period = 1.0 / controller_frequency;
  constexpr double eps = 1e-6;

  if ((controller_period + eps) < settings_.model_dt) {
    // 控制周期 < 推演步长：轨迹预测落后于控制输出，仅警告
    RCLCPP_WARN(
      logger_,
      "Controller period is less then model dt, consider setting it equal");
  } else if (abs(controller_period - settings_.model_dt) < eps) {
    // 控制周期 == 推演步长：可无缝滚动控制序列（推荐配置）
    RCLCPP_INFO(
      logger_,
      "Controller period is equal to model dt. Control sequence "
      "shifting is ON");
    settings_.shift_control_sequence = true;
  } else {
    // 控制周期 > 推演步长：状态误差累积，直接抛错要求用户修正配置
    throw std::runtime_error(
            "Controller period more then model dt, set it equal to model dt");
  }
}

// 重置优化问题到初始条件：清零状态、控制序列、历史与代价
void Optimizer::reset()
{
  // 按当前批次/时域大小重新分配所有张量
  state_.reset(settings_.batch_size, settings_.time_steps);
  control_sequence_.reset(settings_.time_steps);
  // 清空最近 4 个周期的控制历史（用于 Savitzky-Golay 平滑）
  control_history_[0] = {0.0, 0.0, 0.0};
  control_history_[1] = {0.0, 0.0, 0.0};
  control_history_[2] = {0.0, 0.0, 0.0};
  control_history_[3] = {0.0, 0.0, 0.0};

  // 恢复基础速度约束（清除运行时速度限制）
  settings_.constraints = settings_.base_constraints;

  costs_ = xt::zeros<float>({settings_.batch_size});
  generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);

  // 同步重置噪声生成器
  noise_generator_.reset(settings_, isHolonomic());
  RCLCPP_INFO(logger_, "Optimizer reset");
}

// 计算控制指令：MPPI 主入口，由控制器每周期调用（含失败回退机制）
geometry_msgs::msg::TwistStamped Optimizer::evalControl(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  // 第 1 步：准备本周期数据（位姿、速度、路径张量、清零代价等）
  prepare(robot_pose, robot_speed, plan, goal_checker);

  // 第 2 步：执行 MPPI 优化；若 critic 标记失败（如所有轨迹碰撞），
  // 则重置并重试（fallback 内部有次数上限，超限抛异常）
  do {
    optimize();
  } while (fallback(critics_data_.fail_flag));

  // 第 3 步：对控制序列做 Savitzky-Golay 滤波平滑，抑制抖动
  utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
  // 第 4 步：提取序列首个（或滚动后第 2 个）控制量转为 TwistStamped
  auto control = getControlFromSequenceAsTwist(plan.header.stamp);

  // 第 5 步（滚动时域）：将控制序列向前移动一步，作为下周期初值
  if (settings_.shift_control_sequence) {
    shiftControlSequence();
  }

  return control;
}

// MPPI 核心迭代：采样噪声轨迹 → 评分 → 更新控制序列
void Optimizer::optimize()
{
  for (size_t i = 0; i < settings_.iteration_count; ++i) {
    // 2a. 用噪声控制序列 rollout 生成 batch_size 条候选轨迹
    generateNoisedTrajectories();
    // 2b. 由 critic_manager 依次调用所有 critic 计算每条轨迹的代价
    critic_manager_.evalTrajectoriesScores(critics_data_);
    // 2c. softmax 加权平均，更新最优控制序列
    updateControlSequence();
  }
}

// 失败回退：当所有轨迹均不可行时，重置优化器并重试
bool Optimizer::fallback(bool fail)
{
  static size_t counter = 0;

  if (!fail) {
    // 本轮成功：清零计数器，无需回退
    counter = 0;
    return false;
  }

  // 失败：重置优化问题（清零控制序列与噪声）后重试
  reset();

  if (++counter > settings_.retry_attempt_limit) {
    // 超过重试上限：放弃并向上层抛错（触发导航系统恢复行为）
    counter = 0;
    throw std::runtime_error("Optimizer fail to compute path");
  }

  return true;
}

// 每周期准备阶段：填充机器人当前状态与路径数据
void Optimizer::prepare(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  // 记录当前位姿（rollout 起点）与速度（序列首元素初值）
  state_.pose = robot_pose;
  state_.speed = robot_speed;
  // 全局路径转换为张量形式，便于 critic 向量化计算
  path_ = utils::toTensor(plan);
  // 清零上一轮的代价累计
  costs_.fill(0);

  // 重置 critic 共享数据：失败标志、目标检查器、运动模型、
  // 最远到达路径点索引与路径点有效性
  critics_data_.fail_flag = false;
  critics_data_.goal_checker = goal_checker;
  critics_data_.motion_model = motion_model_;
  critics_data_.furthest_reached_path_point.reset();
  critics_data_.path_pts_valid.reset();
}

// 控制序列滚动：将序列左移一步（丢弃已执行的首元素，末尾重复上一元素）
void Optimizer::shiftControlSequence()
{
  using namespace xt::placeholders;  // NOLINT
  // roll(-1) 将所有元素左移一位，首元素被移出（即将在本周期执行）
  control_sequence_.vx = xt::roll(control_sequence_.vx, -1);
  control_sequence_.wz = xt::roll(control_sequence_.wz, -1);


  // 末尾空出的位置复制倒数第二个元素（保持速度连续性，避免阶跃）
  xt::view(control_sequence_.vx, -1) =
    xt::view(control_sequence_.vx, -2);

  xt::view(control_sequence_.wz, -1) =
    xt::view(control_sequence_.wz, -2);


  // 全向底盘还需滚动 vy 序列
  if (isHolonomic()) {
    control_sequence_.vy = xt::roll(control_sequence_.vy, -1);
    xt::view(control_sequence_.vy, -1) =
      xt::view(control_sequence_.vy, -2);
  }
}

// 生成噪声轨迹：用噪声控制序列推演出 batch_size 条候选轨迹
void Optimizer::generateNoisedTrajectories()
{
  // 将（旧噪声 + 当前最优控制）写入 state 的控制张量，
  // 并通知噪声线程为下一周期重新生成噪声（异步）
  noise_generator_.setNoisedControls(state_, control_sequence_);
  noise_generator_.generateNextNoises();
  // 由运动模型预测各时间步速度（考虑加速度等约束）
  updateStateVelocities(state_);
  // 前向积分速度得到位姿序列，即 rollout 轨迹
  integrateStateVelocities(generated_trajectories_, state_);
}

// 是否为全向底盘（决定是否处理 vy 分量）
bool Optimizer::isHolonomic() const {return motion_model_->isHolonomic();}

// 对控制序列施加硬约束（速度上下限裁剪 + 运动模型特有约束）
void Optimizer::applyControlSequenceConstraints()
{
  auto & s = settings_;

  // 全向底盘：裁剪横向速度 vy
  if (isHolonomic()) {
    control_sequence_.vy = xt::clip(control_sequence_.vy, -s.constraints.vy, s.constraints.vy);
  }

  // 裁剪线速度与角速度到允许范围（含倒车负值下限）
  control_sequence_.vx = xt::clip(control_sequence_.vx, s.constraints.vx_min, s.constraints.vx_max);
  control_sequence_.wz = xt::clip(control_sequence_.wz, -s.constraints.wz, s.constraints.wz);

  // 运动模型特有的约束（如 Ackermann 最小转弯半径限制）
  motion_model_->applyConstraints(control_sequence_);
}

// 更新状态速度：先填初值，再由运动模型逐时间步传播
void Optimizer::updateStateVelocities(
  models::State & state) const
{
  updateInitialStateVelocities(state);
  propagateStateVelocitiesFromInitials(state);
}

// 填充每个时间步的初始速度：用机器人真实当前速度作为第 0 步
void Optimizer::updateInitialStateVelocities(
  models::State & state) const
{
  // 所有批次（各行）的第 0 列都设为当前实测速度，保证轨迹从真实状态出发
  xt::noalias(xt::view(state.vx, xt::all(), 0)) = state.speed.linear.x;
  xt::noalias(xt::view(state.wz, xt::all(), 0)) = state.speed.angular.z;

  if (isHolonomic()) {
    xt::noalias(xt::view(state.vy, xt::all(), 0)) = state.speed.linear.y;
  }
}

// 由初始速度经运动模型预测整条时间线上的速度
void Optimizer::propagateStateVelocitiesFromInitials(
  models::State & state) const
{
  motion_model_->predict(state);
}

// 将速度序列积分成轨迹位姿（单条轨迹版本，用于可视化最优轨迹）
// 运动学：x' = vx*cos(yaw) - vy*sin(yaw)，y' = vx*sin(yaw) + vy*cos(yaw)
void Optimizer::integrateStateVelocities(
  xt::xtensor<float, 2> & trajectory,
  const xt::xtensor<float, 2> & sequence) const
{
  // 初始朝向取自机器人当前位姿
  float initial_yaw = tf2::getYaw(state_.pose.pose.orientation);

  // 列约定：0=vx，1=wz，2=vy（全向时才有意义）
  const auto vx = xt::view(sequence, xt::all(), 0);
  const auto vy = xt::view(sequence, xt::all(), 2);
  const auto wz = xt::view(sequence, xt::all(), 1);

  auto traj_x = xt::view(trajectory, xt::all(), 0);
  auto traj_y = xt::view(trajectory, xt::all(), 1);
  auto traj_yaws = xt::view(trajectory, xt::all(), 2);

  // 朝向 = 初始朝向 + 累计角速度积分（cumsum 累加和）
  xt::noalias(traj_yaws) = cumsum_1d(wz * settings_.model_dt) + initial_yaw;

  // 预分配 cos/sin 缓冲区避免多次求值
  auto && yaw_cos = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());
  auto && yaw_sin = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());

  const auto yaw_offseted = xt::view(traj_yaws, xt::range(1, _));

  // 第 0 步用初始朝向的三角函数，其余步用前一步朝向
  xt::noalias(xt::view(yaw_cos, 0)) = cosf(initial_yaw);
  xt::noalias(xt::view(yaw_sin, 0)) = sinf(initial_yaw);
  xt::noalias(xt::view(yaw_cos, xt::range(1, _))) = xt::cos(yaw_offseted);
  xt::noalias(xt::view(yaw_sin, xt::range(1, _))) = xt::sin(yaw_offseted);

  // 机体坐标系速度旋转到世界坐标系
  auto && dx = xt::eval(vx * yaw_cos);
  auto && dy = xt::eval(vx * yaw_sin);

  if (isHolonomic()) {
    // 全向底盘需加入 vy 的旋转投影项
    dx = dx - vy * yaw_sin;
    dy = dy + vy * yaw_cos;
  }

  // 位置 = 初始位置 + 速度累加积分（欧拉积分）
  xt::noalias(traj_x) = state_.pose.pose.position.x + cumsum_1d(dx * settings_.model_dt);
  xt::noalias(traj_y) = state_.pose.pose.position.y + cumsum_1d(dy * settings_.model_dt);
}

// 将批量速度状态积分成批量轨迹位姿（rollout 主版本，二维张量按批次并行）
void Optimizer::integrateStateVelocities(
  models::Trajectories & trajectories,
  const models::State & state) const
{
  const float initial_yaw = tf2::getYaw(state.pose.pose.orientation);

  // 每条轨迹的朝向 = 初始朝向 + 各步角速度累加（按时间轴 cumsum）
  xt::noalias(trajectories.yaws) =
    cumsum_2d(state.wz * settings_.model_dt, 1) + initial_yaw;

  // 使用前一时刻朝向做积分（range(0,-1) 取 0..N-2，即右移一位）
  const auto yaws_cutted = xt::view(trajectories.yaws, xt::all(), xt::range(0, -1));

  auto && yaw_cos = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
  auto && yaw_sin = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
  // 第 0 列用初始朝向；后续列用前一步朝向的 cos/sin
  xt::noalias(xt::view(yaw_cos, xt::all(), 0)) = cosf(initial_yaw);
  xt::noalias(xt::view(yaw_sin, xt::all(), 0)) = sinf(initial_yaw);
  xt::noalias(xt::view(yaw_cos, xt::all(), xt::range(1, _))) = xt::cos(yaws_cutted);
  xt::noalias(xt::view(yaw_sin, xt::all(), xt::range(1, _))) = xt::sin(yaws_cutted);

  // 机体速度 → 世界系位移增量
  auto && dx = xt::eval(state.vx * yaw_cos);
  auto && dy = xt::eval(state.vx * yaw_sin);

  if (isHolonomic()) {
    // 全向底盘的横向速度投影
    dx = dx - state.vy * yaw_sin;
    dy = dy + state.vy * yaw_cos;
  }

  // 位置 = 初始位置 + 位移增量沿时间轴累加（欧拉积分）
  xt::noalias(trajectories.x) = state.pose.pose.position.x +
    cumsum_2d(dx * settings_.model_dt, 1);
  xt::noalias(trajectories.y) = state.pose.pose.position.y +
    cumsum_2d(dy * settings_.model_dt, 1);
}

// 获取当前最优控制序列推演出的轨迹（用于 RViz 可视化 optimal 轨迹）
xt::xtensor<float, 2> Optimizer::getOptimizedTrajectory()
{
  // 组装 [time_steps, 2或3] 的控制序列张量（vx, wz, [vy]）
  auto && sequence =
    xt::xtensor<float, 2>::from_shape({settings_.time_steps, isHolonomic() ? 3u : 2u});
  auto && trajectories = xt::xtensor<float, 2>::from_shape({settings_.time_steps, 3});

  xt::noalias(xt::view(sequence, xt::all(), 0)) = control_sequence_.vx;
  xt::noalias(xt::view(sequence, xt::all(), 1)) = control_sequence_.wz;

  if (isHolonomic()) {
    xt::noalias(xt::view(sequence, xt::all(), 2)) = control_sequence_.vy;
  }

  // 用单条轨迹积分函数推演最优轨迹
  integrateStateVelocities(trajectories, sequence);
  return std::move(trajectories);
}

// MPPI 核心更新公式：softmax 加权平均所有噪声控制序列，得到新控制序列
void Optimizer::updateControlSequence()
{
  auto & s = settings_;
  // 提取“有界噪声”：ε = state 控制量 - 名义控制序列
  // （state 中存的是被约束裁剪后的实际控制，故其与名义序列之差即有效噪声）
  auto bounded_noises_vx = state_.cvx - control_sequence_.vx;
  auto bounded_noises_wz = state_.cwz - control_sequence_.wz;
  // 信息论正则项（路径积分理论中的 control energy 项）：
  // cost += γ/σ² * Σ u·ε，惩罚大控制量并使更新向最优轨迹收敛
  xt::noalias(costs_) +=
    s.gamma / powf(s.sampling_std.vx, 2) * xt::sum(
    xt::view(control_sequence_.vx, xt::newaxis(), xt::all()) * bounded_noises_vx, 1, immediate);
  xt::noalias(costs_) +=
    s.gamma / powf(s.sampling_std.wz, 2) * xt::sum(
    xt::view(control_sequence_.wz, xt::newaxis(), xt::all()) * bounded_noises_wz, 1, immediate);

  // 全向底盘附加 vy 的正则项
  if (isHolonomic()) {
    auto bounded_noises_vy = state_.cvy - control_sequence_.vy;
    xt::noalias(costs_) +=
      s.gamma / powf(s.sampling_std.vy, 2) * xt::sum(
      xt::view(control_sequence_.vy, xt::newaxis(), xt::all()) * bounded_noises_vy,
      1, immediate);
  }

  // ---- softmax 计算权重 ----
  // 1) 代价平移：减去最小值防止 exp 溢出（不改变 softmax 结果）
  auto && costs_normalized = costs_ - xt::amin(costs_, immediate);
  // 2) 指数项：exp(-(J - minJ)/T)，T 为温度（越小越接近硬选最优）
  auto && exponents = xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
  // 3) 归一化得到概率权重 w_i
  auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
  // 扩展为列向量以便与 [batch, time] 张量逐元素相乘
  auto && softmaxes_extened = xt::eval(xt::view(softmaxes, xt::all(), xt::newaxis()));

  // 新控制序列 = Σ_i w_i * u_i（所有噪声控制序列按权重的加权平均）
  xt::noalias(control_sequence_.vx) = xt::sum(state_.cvx * softmaxes_extened, 0, immediate);
  xt::noalias(control_sequence_.wz) = xt::sum(state_.cwz * softmaxes_extened, 0, immediate);
  if (isHolonomic()) {
    xt::noalias(control_sequence_.vy) = xt::sum(state_.cvy * softmaxes_extened, 0, immediate);
  }

  // 对更新后的序列施加速度/运动学硬约束
  applyControlSequenceConstraints();
}

// 从控制序列提取本周期要发送的速度指令（TwistStamped）
geometry_msgs::msg::TwistStamped Optimizer::getControlFromSequenceAsTwist(
  const builtin_interfaces::msg::Time & stamp)
{
  // 若启用序列滚动，则提取第 2 个元素（第 1 个已在本周期执行并移出）
  unsigned int offset = settings_.shift_control_sequence ? 1 : 0;

  auto vx = control_sequence_.vx(offset);
  auto wz = control_sequence_.wz(offset);

  if (isHolonomic()) {
    // 全向底盘输出含 vy 的三轴速度
    auto vy = control_sequence_.vy(offset);
    return utils::toTwistStamped(vx, vy, wz, stamp, costmap_ros_->getBaseFrameID());
  }

  // 差速/阿克曼底盘输出 vx、wz 两轴
  return utils::toTwistStamped(vx, wz, stamp, costmap_ros_->getBaseFrameID());
}

// 根据参数字符串选择运动模型（工厂方法）
void Optimizer::setMotionModel(const std::string & model)
{
  if (model == "DiffDrive") {
    // 差速驱动：vx + wz，无 vy
    motion_model_ = std::make_shared<DiffDriveMotionModel>();
  } else if (model == "Omni") {
    // 全向驱动：vx + vy + wz
    motion_model_ = std::make_shared<OmniMotionModel>();
  } else if (model == "Ackermann") {
    // 阿克曼转向：受最小转弯半径约束（读取参数 minimum_turning_radius）
    motion_model_ = std::make_shared<AckermannMotionModel>(parameters_handler_, name_);
  } else {
    // 未知模型名：直接抛错
    throw std::runtime_error(
            std::string(
              "Model " + model + " is not valid! Valid options are DiffDrive, Omni, "
              "or Ackermann"));
  }
}

// 设置速度上限（由速度限制器/代价地图过滤器回调触发）
void Optimizer::setSpeedLimit(double speed_limit, bool percentage)
{
  auto & s = settings_;
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // 无速度限制：恢复基础约束
    s.constraints.vx_max = s.base_constraints.vx_max;
    s.constraints.vx_min = s.base_constraints.vx_min;
    s.constraints.vy = s.base_constraints.vy;
    s.constraints.wz = s.base_constraints.wz;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      //（速度限制以百分比给出：按比例缩放各速度约束）
      double ratio = speed_limit / 100.0;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    } else {
      // Speed limit is expressed in absolute value
      //（速度限制以绝对值 m/s 给出：按 vx_max 的比例缩放其余约束）
      double ratio = speed_limit / s.base_constraints.vx_max;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    }
  }
}

// 获取本周期生成的候选轨迹（供可视化器发布全部采样轨迹）
models::Trajectories & Optimizer::getGeneratedTrajectories()
{
  return generated_trajectories_;
}

}  // namespace mppi
