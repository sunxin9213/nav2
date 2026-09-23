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

// 本文件实现 CriticManager（评价器管理器）：
// MPPI 控制器通过 pluginlib 动态加载用户在参数 "critics" 中指定的各个
// 轨迹评价器（CriticFunction 插件），并在每个控制周期内依次调用它们
// 对整批采样轨迹打分。所有评价器的分数累加到 CriticData::costs 中，
// 供优化器随后做 softmax 加权平均。任一评价器可置位 fail_flag 提前终止。
#include "nav2_mppi_controller/critic_manager.hpp"

namespace mppi
{

// 生命周期配置入口：保存节点/代价地图/参数处理器引用，再读取参数并加载插件
void CriticManager::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, ParametersHandler * param_handler)
{
  parent_ = parent;               // 生命周期节点弱引用，避免延长节点寿命
  costmap_ros_ = costmap_ros;     // 代价地图（障碍物/代价值来源）
  name_ = name;                   // 本控制器插件名（如 FollowPath）
  auto node = parent_.lock();
  logger_ = node->get_logger();   // 日志器
  parameters_handler_ = param_handler;  // 参数处理器（含动态参数回调）

  getParams();   // 读取 critics 列表参数
  loadCritics(); // 按列表逐个加载评价器插件
}

// 读取本管理器自身的参数：启用哪些评价器（Static 表示运行期不可动态修改）
void CriticManager::getParams()
{
  auto node = parent_.lock();
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(critic_names_, "critics", std::vector<std::string>{}, ParameterType::Static);
}

// 通过 pluginlib ClassLoader 动态实例化每个评价器并完成其配置
void CriticManager::loadCritics()
{
  // 懒加载：首次调用时创建 ClassLoader。
  // createUnmanagedInstance 返回裸指针，由 unique_ptr 接管所有权，
  // 避免 pluginlib 默认实例在析构时二次 delete 的问题
  if (!loader_) {
    loader_ = std::make_unique<pluginlib::ClassLoader<critics::CriticFunction>>(
      "nav2_mppi_controller", "mppi::critics::CriticFunction");
  }

  critics_.clear();  // 支持重复配置：先清空旧实例
  for (auto name : critic_names_) {
    // 拼接完整插件类名，如 "mppi::critics::ConstraintCritic"
    std::string fullname = getFullName(name);
    auto instance = std::unique_ptr<critics::CriticFunction>(
      loader_->createUnmanagedInstance(fullname));
    critics_.push_back(std::move(instance));
    // 用 "FollowPath.<name>" 作为该评价器自己的参数命名空间
    critics_.back()->on_configure(
      parent_, name_, name_ + "." + name, costmap_ros_,
      parameters_handler_);
    RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
  }
}

// 将简称（如 "ConstraintCritic"）映射为完整类名（带 mppi::critics:: 前缀）
std::string CriticManager::getFullName(const std::string & name)
{
  return "mppi::critics::" + name;
}

// 核心打分入口：按加载顺序依次调用每个评价器的 score()。
// 所有评价器共享同一个 CriticData（含轨迹、路径、代价输出等）。
// fail_flag 一旦被置位（如检测到碰撞），立即中断后续评价，
// 让优化器进入失败/降级处理分支
void CriticManager::evalTrajectoriesScores(
  CriticData & data) const
{
  for (size_t q = 0; q < critics_.size(); q++) {
    if (data.fail_flag) {
      break;  // 已有评价器判定失败，提前退出
    }
    critics_[q]->score(data);
  }
}

}  // namespace mppi
