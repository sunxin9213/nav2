// Copyright (c) 2022 Samsung R&D Institute Russia
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

// ============================================================================
// 文件说明（中文）：
// Source（观测数据源基类）的实现。
// 模块定位：nav2_collision_monitor 中所有传感器数据源的公共基类。
// 碰撞监视器从多种数据源（激光扫描 Scan、点云 PointCloud、
// 红外/超声测距 Range）获取障碍物观测；本基类统一管理：
//   - 数据源名称、话题（getCommonParameters）、TF 句柄与坐标系；
//   - 数据有效性判断（sourceValid：超过 source_timeout 未更新则忽略）；
//   - 使能开关（enabled 参数，支持运行时动态修改）。
// 子类只需实现数据回调与点提取逻辑（见 scan.cpp / pointcloud.cpp / range.cpp）。
// ============================================================================

#include "nav2_collision_monitor/source.hpp"

#include <exception>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

// ---------------------------------------------------------------------------
// 构造函数：保存数据源名称、TF 缓冲、机器人本体/全局坐标系、
// TF 容差、数据超时时间与"基座漂移校正"开关。
// ---------------------------------------------------------------------------
Source::Source(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
: node_(node), source_name_(source_name), tf_buffer_(tf_buffer),
  base_frame_id_(base_frame_id), global_frame_id_(global_frame_id),
  transform_tolerance_(transform_tolerance), source_timeout_(source_timeout),
  base_shift_correction_(base_shift_correction)
{
}

Source::~Source()
{
}

// ---------------------------------------------------------------------------
// 配置：注册动态参数回调（运行时修改 enabled 等参数立即生效）。
// ---------------------------------------------------------------------------
bool Source::configure()
{
  auto node = node_.lock();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&Source::dynamicParametersCallback, this, std::placeholders::_1));

  return true;
}

// ---------------------------------------------------------------------------
// 读取所有数据源共有的参数：
//   <source_name>.topic  数据话题（默认 "scan"，针对激光雷达）
//   <source_name>.enabled 是否启用该数据源
// ---------------------------------------------------------------------------
void Source::getCommonParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".topic",
    rclcpp::ParameterValue("scan"));  // Set deafult topic for laser scanner
  source_topic = node->get_parameter(source_name_ + ".topic").as_string();

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".enabled", rclcpp::ParameterValue(true));
  enabled_ = node->get_parameter(source_name_ + ".enabled").as_bool();
}

// ---------------------------------------------------------------------------
// 判断数据源是否有效：最近一帧数据的时间戳距当前时刻超过
// source_timeout_ 则视为过期（传感器掉线/数据陈旧），告警并返回 false。
// ---------------------------------------------------------------------------
bool Source::sourceValid(
  const rclcpp::Time & source_time,
  const rclcpp::Time & curr_time) const
{
  // Source is considered as not valid, if latest received data timestamp is earlier
  // than current time by source_timeout_ interval
  const rclcpp::Duration dt = curr_time - source_time;
  if (dt > source_timeout_) {
    RCLCPP_WARN(
      logger_,
      "[%s]: Latest source and current collision monitor node timestamps differ on %f seconds. "
      "Ignoring the source.",
      source_name_.c_str(), dt.seconds());
    return false;
  }

  return true;
}

// 返回数据源是否启用
bool Source::getEnabled() const
{
  return enabled_;
}

// ---------------------------------------------------------------------------
// 动态参数回调：处理运行时的参数修改请求。
// 目前仅支持布尔型 enabled 参数（动态启停数据源）。
// ---------------------------------------------------------------------------
rcl_interfaces::msg::SetParametersResult
Source::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      if (param_name == source_name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_collision_monitor
