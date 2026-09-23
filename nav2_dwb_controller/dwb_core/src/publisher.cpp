/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "dwb_core/publisher.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav2_util/node_utils.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::max;
using std::string;
using nav2_util::declare_parameter_if_not_declared;

// 本文件实现 DWBPublisher（DWB 调试信息发布器）：
// 集中发布全局/局部路径、评分评价消息、候选轨迹 Marker 与
// 评价器逐格评分点云。所有发布都遵循"无订阅者不发布"的惰性策略，
// 避免无谓的消息序列化开销。
namespace dwb_core
{

// 构造：保存节点弱引用与插件名，获取时钟
DWBPublisher::DWBPublisher(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
: node_(parent),
  plugin_name_(plugin_name)
{
  auto node = node_.lock();
  clock_ = node->get_clock();
}

// 生命周期配置：声明/读取各发布开关参数，创建全部发布器
nav2_util::CallbackReturn
DWBPublisher::on_configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // 声明各调试话题的发布开关（路径类默认开启，点云默认关闭）
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_evaluation",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_global_plan",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_transformed_plan",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_local_plan",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_trajectories",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_cost_grid_pc",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".marker_lifetime",
    rclcpp::ParameterValue(0.1));

  node->get_parameter(plugin_name_ + ".publish_evaluation", publish_evaluation_);
  node->get_parameter(plugin_name_ + ".publish_global_plan", publish_global_plan_);
  node->get_parameter(plugin_name_ + ".publish_transformed_plan", publish_transformed_);
  node->get_parameter(plugin_name_ + ".publish_local_plan", publish_local_plan_);
  node->get_parameter(plugin_name_ + ".publish_trajectories", publish_trajectories_);
  node->get_parameter(plugin_name_ + ".publish_cost_grid_pc", publish_cost_grid_pc_);

  // 创建六个发布器（话题名固定，队列深度 1）
  eval_pub_ = node->create_publisher<dwb_msgs::msg::LocalPlanEvaluation>("evaluation", 1);
  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  transformed_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 1);
  local_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 1);
  cost_grid_pc_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("cost_cloud", 1);

  double marker_lifetime = 0.0;
  node->get_parameter(plugin_name_ + ".marker_lifetime", marker_lifetime);
  marker_lifetime_ = rclcpp::Duration::from_seconds(marker_lifetime);

  return nav2_util::CallbackReturn::SUCCESS;
}

// 激活：逐个激活生命周期发布器（激活后才能真正发出消息）
nav2_util::CallbackReturn
DWBPublisher::on_activate()
{
  eval_pub_->on_activate();
  global_pub_->on_activate();
  transformed_pub_->on_activate();
  local_pub_->on_activate();
  marker_pub_->on_activate();
  cost_grid_pc_pub_->on_activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DWBPublisher::on_deactivate()
{
  eval_pub_->on_deactivate();
  global_pub_->on_deactivate();
  transformed_pub_->on_deactivate();
  local_pub_->on_deactivate();
  marker_pub_->on_deactivate();
  cost_grid_pc_pub_->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

// 清理：释放全部发布器对象
nav2_util::CallbackReturn
DWBPublisher::on_cleanup()
{
  eval_pub_.reset();
  global_pub_.reset();
  transformed_pub_.reset();
  local_pub_.reset();
  marker_pub_.reset();
  cost_grid_pc_pub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

// 发布评分评价：先发评价消息（有订阅且开启时），再发轨迹 Marker
void
DWBPublisher::publishEvaluation(std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results)
{
  if (results) {
    if (publish_evaluation_ && eval_pub_->get_subscription_count() > 0) {
      auto msg = std::make_unique<dwb_msgs::msg::LocalPlanEvaluation>(*results);
      eval_pub_->publish(std::move(msg));
    }
    publishTrajectories(*results);
  }
}

// 将全部候选轨迹绘制为 LINE_STRIP Marker：
// 合法轨迹按分数好坏红绿渐变（最优偏绿），非法轨迹（负分）为黑色
void
DWBPublisher::publishTrajectories(const dwb_msgs::msg::LocalPlanEvaluation & results)
{
  if (marker_pub_->get_subscription_count() < 1) {return;}

  if (!publish_trajectories_) {return;}
  auto ma = std::make_unique<visualization_msgs::msg::MarkerArray>();
  visualization_msgs::msg::Marker m;

  if (results.twists.size() == 0) {return;}

  geometry_msgs::msg::Point pt;

  // 基础 Marker 属性：线带、线宽 2mm、存活 marker_lifetime_ 秒
  m.header = results.header;
  m.type = m.LINE_STRIP;
  m.pose.orientation.w = 1;
  m.scale.x = 0.002;
  m.color.a = 1.0;
  m.lifetime = marker_lifetime_;

  // 最优/最差分数用于把分数归一化到 [0,1] 的显示色阶
  double best_cost = results.twists[results.best_index].total;
  double worst_cost = results.twists[results.worst_index].total;
  double denominator = worst_cost - best_cost;

  if (std::fabs(denominator) < 1e-9) {
    denominator = 1.0;  // 所有分数相同，避免除零
  }

  unsigned currentValidId = 0;
  unsigned currentInvalidId = 0;
  string validNamespace("ValidTrajectories");      // 合法轨迹命名空间
  string invalidNamespace("InvalidTrajectories");  // 非法轨迹命名空间
  for (unsigned int i = 0; i < results.twists.size(); i++) {
    const dwb_msgs::msg::TrajectoryScore & twist = results.twists[i];
    double displayLevel = (twist.total - best_cost) / denominator;  // 归一化色阶
    if (twist.total >= 0) {
      // 合法轨迹：红绿渐变（displayLevel=0 最优偏绿）
      m.color.r = displayLevel;
      m.color.g = 1.0 - displayLevel;
      m.color.b = 0;
      m.color.a = 1.0;
      m.ns = validNamespace;
      m.id = currentValidId;
      ++currentValidId;
    } else {
      // 非法轨迹（负分）：黑色
      m.color.r = 0;
      m.color.g = 0;
      m.color.b = 0;
      m.color.a = 1.0;
      m.ns = invalidNamespace;
      m.id = currentInvalidId;
      ++currentInvalidId;
    }
    m.points.clear();
    // 轨迹位姿转为 3D 点序列（z=0 平面）
    for (unsigned int j = 0; j < twist.traj.poses.size(); ++j) {
      pt.x = twist.traj.poses[j].x;
      pt.y = twist.traj.poses[j].y;
      pt.z = 0;
      m.points.push_back(pt);
    }
    ma->markers.push_back(m);
  }
  marker_pub_->publish(std::move(ma));
}

// 发布局部轨迹（选中轨迹的位姿序列转为 Path 消息）
void
DWBPublisher::publishLocalPlan(
  const std_msgs::msg::Header & header,
  const dwb_msgs::msg::Trajectory2D & traj)
{
  if (!publish_local_plan_) {return;}

  auto path =
    std::make_unique<nav_msgs::msg::Path>(
    nav_2d_utils::poses2DToPath(
      traj.poses, header.frame_id,
      header.stamp));

  if (local_pub_->get_subscription_count() > 0) {
    local_pub_->publish(std::move(path));
  }
}

// 发布评价器逐格评分点云：把每个评价器对代价地图每个格子的评分
// （通过 addCriticVisualization 获取）编码为 PointCloud2 的附加通道，
// 并加权累加出 total_cost 总分通道，便于 RViz 按通道着色调试
void
DWBPublisher::publishCostGrid(
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  const std::vector<TrajectoryCritic::Ptr> critics)
{
  if (cost_grid_pc_pub_->get_subscription_count() < 1) {return;}

  if (!publish_cost_grid_pc_) {return;}

  auto cost_grid_pc = std::make_unique<sensor_msgs::msg::PointCloud2>();
  cost_grid_pc->header.frame_id = costmap_ros->getGlobalFrameID();
  cost_grid_pc->header.stamp = clock_->now();

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros->getCostmap();
  double x_coord, y_coord;
  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();

  // 收集各评价器的逐格评分通道，并按 scale 加权累加总成本
  std::vector<std::pair<std::string, std::vector<float>>> cost_channels;
  std::vector<float> total_cost(size_x * size_y, 0.0);

  for (TrajectoryCritic::Ptr critic : critics) {
    unsigned int channel_index = cost_channels.size();
    critic->addCriticVisualization(cost_channels);
    if (channel_index == cost_channels.size()) {
      // No channels were added, so skip to next critic
      continue;  // 该评价器不提供可视化数据
    }
    double scale = critic->getScale();
    for (unsigned int i = 0; i < size_x * size_y; i++) {
      total_cost[i] += cost_channels[channel_index].second[i] * scale;
    }
  }

  // 附加总成本通道
  cost_channels.push_back(std::make_pair("total_cost", total_cost));

  // 填充点云元数据：字段 = x,y,z + 各评分通道，均为 FLOAT32（4 字节）
  cost_grid_pc->width = size_x * size_y;
  cost_grid_pc->height = 1;
  cost_grid_pc->fields.resize(3 + cost_channels.size());  // x,y,z, + cost channels
  cost_grid_pc->is_dense = true;
  cost_grid_pc->is_bigendian = false;

  int offset = 0;
  for (size_t i = 0; i < cost_grid_pc->fields.size(); ++i, offset += 4) {
    cost_grid_pc->fields[i].offset = offset;
    cost_grid_pc->fields[i].count = 1;
    cost_grid_pc->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    if (i >= 3) {
      cost_grid_pc->fields[i].name = cost_channels[i - 3].first;  // 评分通道名
    }
  }

  cost_grid_pc->fields[0].name = "x";
  cost_grid_pc->fields[1].name = "y";
  cost_grid_pc->fields[2].name = "z";

  cost_grid_pc->point_step = offset;  // 每点字节数
  cost_grid_pc->row_step = cost_grid_pc->point_step * cost_grid_pc->width;
  cost_grid_pc->data.resize(cost_grid_pc->row_step * cost_grid_pc->height);

  // 为每个字段创建写迭代器
  std::vector<sensor_msgs::PointCloud2Iterator<float>> cost_grid_pc_iter;

  for (size_t i = 0; i < cost_grid_pc->fields.size(); ++i) {
    sensor_msgs::PointCloud2Iterator<float> iter(*cost_grid_pc, cost_grid_pc->fields[i].name);
    cost_grid_pc_iter.push_back(iter);
  }

  // 逐格填充：格子索引转世界坐标写入 (x,y,z)，评分写入对应通道
  unsigned int j = 0;
  for (unsigned int cy = 0; cy < size_y; cy++) {
    for (unsigned int cx = 0; cx < size_x; cx++) {
      costmap->mapToWorld(cx, cy, x_coord, y_coord);
      *cost_grid_pc_iter[0] = x_coord;
      *cost_grid_pc_iter[1] = y_coord;
      *cost_grid_pc_iter[2] = 0.0;   // z value

      for (size_t i = 3; i < cost_grid_pc_iter.size(); ++i) {
        *cost_grid_pc_iter[i] = cost_channels[i - 3].second[j];
        ++cost_grid_pc_iter[i];
      }
      ++cost_grid_pc_iter[0];
      ++cost_grid_pc_iter[1];
      ++cost_grid_pc_iter[2];
      j++;
    }
  }

  cost_grid_pc_pub_->publish(std::move(cost_grid_pc));
}

// 以下三个函数分别发布全局/变换后/局部路径（转发到通用实现）
void
DWBPublisher::publishGlobalPlan(const nav_2d_msgs::msg::Path2D plan)
{
  publishGenericPlan(plan, *global_pub_, publish_global_plan_);
}

void
DWBPublisher::publishTransformedPlan(const nav_2d_msgs::msg::Path2D plan)
{
  publishGenericPlan(plan, *transformed_pub_, publish_transformed_);
}

void
DWBPublisher::publishLocalPlan(const nav_2d_msgs::msg::Path2D plan)
{
  publishGenericPlan(plan, *local_pub_, publish_local_plan_);
}

// 通用路径发布：无订阅者或开关关闭时直接返回，避免无效序列化
void
DWBPublisher::publishGenericPlan(
  const nav_2d_msgs::msg::Path2D plan,
  rclcpp::Publisher<nav_msgs::msg::Path> & pub, bool flag)
{
  if (pub.get_subscription_count() < 1) {return;}
  if (!flag) {return;}
  auto path = std::make_unique<nav_msgs::msg::Path>(nav_2d_utils::pathToPath(plan));
  pub.publish(std::move(path));
}

}  // namespace dwb_core
