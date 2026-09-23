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

#include <memory>
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"

namespace mppi
{

// 配置：创建调试发布器并读取降采样参数
void TrajectoryVisualizer::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  const std::string & frame_id, ParametersHandler * parameters_handler)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  frame_id_ = frame_id;
  // 创建两个调试话题发布器（深度 1，调试数据无需缓存排队）
  trajectories_publisher_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectories", 1);
  transformed_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 1);
  parameters_handler_ = parameters_handler;

  auto getParam = parameters_handler->getParamGetter(name + ".TrajectoryVisualizer");

  // trajectory_step：每隔多少条候选轨迹取一条显示（默认 5，降低消息量）
  getParam(trajectory_step_, "trajectory_step", 5);
  // time_step：每条轨迹每隔多少时间步取一个点（默认 3）
  getParam(time_step_, "time_step", 3);

  reset();
}

// 清理：释放发布器资源
void TrajectoryVisualizer::on_cleanup()
{
  trajectories_publisher_.reset();
  transformed_path_pub_.reset();
}

// 激活：生命周期发布器转为激活态，开始实际发布
void TrajectoryVisualizer::on_activate()
{
  trajectories_publisher_->on_activate();
  transformed_path_pub_->on_activate();
}

// 停用：暂停发布（节点进入 inactive 状态时）
void TrajectoryVisualizer::on_deactivate()
{
  trajectories_publisher_->on_deactivate();
  transformed_path_pub_->on_deactivate();
}

// 添加单条轨迹（通常是最优轨迹）为散点标记：起点绿渐变至终点蓝
void TrajectoryVisualizer::add(
  const xt::xtensor<float, 2> & trajectory, const std::string & marker_namespace)
{
  auto & size = trajectory.shape()[0];  // 轨迹点数（时间步数）
  if (!size) {
    return;  // 空轨迹直接跳过
  }

  // 为每个轨迹点创建一个 marker
  auto add_marker = [&](auto i) {
      // 颜色随进度插值：component = i/size，用于绿色→蓝色渐变表示时间先后
      float component = static_cast<float>(i) / static_cast<float>(size);

      // 高度略抬（0.06）便于在 RViz 平面地图上观察
      auto pose = utils::createPose(trajectory(i, 0), trajectory(i, 1), 0.06);
      // 终点标记放大显示，突出轨迹终点
      auto scale =
        i != size - 1 ?
        utils::createScale(0.03, 0.03, 0.07) :
        utils::createScale(0.07, 0.07, 0.09);
      auto color = utils::createColor(0, component, component, 1);
      auto marker = utils::createMarker(
        marker_id_++, pose, scale, color, frame_id_, marker_namespace);
      points_->markers.push_back(marker);
    };

  for (size_t i = 0; i < size; i++) {
    add_marker(i);
  }
}

// 添加批量候选轨迹为散点标记：按 trajectory_step_/time_step_ 双重降采样
void TrajectoryVisualizer::add(
  const models::Trajectories & trajectories, const std::string & marker_namespace)
{
  auto & shape = trajectories.x.shape();  // [batch_size, time_steps]
  const float shape_1 = static_cast<float>(shape[1]);
  // 预分配 marker 数组容量，避免反复扩容
  points_->markers.reserve(floor(shape[0] / trajectory_step_) * floor(shape[1] * time_step_));

  // 外层按轨迹条数降采样，内层按时间步降采样
  for (size_t i = 0; i < shape[0]; i += trajectory_step_) {
    for (size_t j = 0; j < shape[1]; j += time_step_) {
      const float j_flt = static_cast<float>(j);
      // 颜色编码时间：早期偏蓝、后期偏绿，反映轨迹推进方向
      float blue_component = 1.0f - j_flt / shape_1;
      float green_component = j_flt / shape_1;

      auto pose = utils::createPose(trajectories.x(i, j), trajectories.y(i, j), 0.03);
      auto scale = utils::createScale(0.03, 0.03, 0.03);
      auto color = utils::createColor(0, green_component, blue_component, 1);
      auto marker = utils::createMarker(
        marker_id_++, pose, scale, color, frame_id_, marker_namespace);

      points_->markers.push_back(marker);
    }
  }
}

// 重置：清空累积的标记并归零 id 计数
void TrajectoryVisualizer::reset()
{
  marker_id_ = 0;
  points_ = std::make_unique<visualization_msgs::msg::MarkerArray>();
}

// 发布本周期累积的所有调试消息
void TrajectoryVisualizer::visualize(const nav_msgs::msg::Path & plan)
{
  // 仅在有订阅者时发布轨迹标记（节省带宽），发布后 points_ 被移走
  if (trajectories_publisher_->get_subscription_count() > 0) {
    trajectories_publisher_->publish(std::move(points_));
  }

  // 重建标记容器，准备下一周期
  reset();

  // 同样按需发布变换后的全局路径
  if (transformed_path_pub_->get_subscription_count() > 0) {
    auto plan_ptr = std::make_unique<nav_msgs::msg::Path>(plan);
    transformed_path_pub_->publish(std::move(plan_ptr));
  }
}

}  // namespace mppi
