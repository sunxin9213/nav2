// Copyright (c) 2020 Samsung Research Russia
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

// TODO(AlexeyMerzlyakov): This dummy info publisher should be removed
// after Semantic Map Server having the same functionality will be developed.

// ============================================================================
// 文件说明（中文）：
// CostmapFilterInfoServer（代价地图过滤器信息服务器）的实现。
// 模块定位：nav2_map_server 中的辅助生命周期节点，服务于
// Nav2 代价地图过滤器（costmap filter）体系。
//
// 代价地图过滤器机制简介：
//   Nav2 通过"过滤器掩码（filter mask）"图层修改全局/局部代价地图，
//   实现禁行区（keepout）、限速区（speed limit）、偏好方向等功能。
//   过滤器需要的元信息由 nav2_msgs::msg::CostmapFilterInfo 消息描述：
//     type         过滤器类型编号（0=禁行 keepout，1=限速 speed 等）
//     filter_mask_topic 掩码地图话题（订阅它可得到掩码栅格）
//     base / multiplier 掩码值到目标量的线性变换参数：
//                      目标值 = base + multiplier * 掩码值
//   本节点在激活时通过 transient_local QoS 发布一次该信息
//   （迟加入的过滤器也能收到），掩码地图本身由 map_server 提供。
// ============================================================================

#include "nav2_map_server/costmap_filter_info_server.hpp"

#include <string>
#include <memory>
#include <utility>

namespace nav2_map_server
{

// ---------------------------------------------------------------------------
// 构造函数：声明过滤器信息相关的节点参数
//   filter_info_topic 信息发布话题名（默认 costmap_filter_info）
//   type              过滤器类型编号
//   mask_topic        掩码地图话题名（默认 filter_mask）
//   base / multiplier 掩码值的线性变换基准值与倍率
// ---------------------------------------------------------------------------
CostmapFilterInfoServer::CostmapFilterInfoServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("costmap_filter_info_server", "", options)
{
  declare_parameter("filter_info_topic", "costmap_filter_info");
  declare_parameter("type", 0);
  declare_parameter("mask_topic", "filter_mask");
  declare_parameter("base", 0.0);
  declare_parameter("multiplier", 1.0);
}

CostmapFilterInfoServer::~CostmapFilterInfoServer()
{
}

// ---------------------------------------------------------------------------
// 生命周期回调 on_configure：
// 创建 CostmapFilterInfo 发布器（QoS: KeepLast(1) + transient_local +
// reliable——latched 语义，新订阅者也能收到最后一帧），
// 并从参数填充待发布的消息内容。
// ---------------------------------------------------------------------------
nav2_util::CallbackReturn
CostmapFilterInfoServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  std::string filter_info_topic = get_parameter("filter_info_topic").as_string();

  publisher_ = this->create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // 组装 CostmapFilterInfo 消息
  msg_ = nav2_msgs::msg::CostmapFilterInfo();
  msg_.header.frame_id = "";
  msg_.header.stamp = now();
  msg_.type = get_parameter("type").as_int();                          // 过滤器类型
  msg_.filter_mask_topic = get_parameter("mask_topic").as_string();    // 掩码话题名
  msg_.base = static_cast<float>(get_parameter("base").as_double());   // 变换基准
  msg_.multiplier = static_cast<float>(get_parameter("multiplier").as_double());  // 变换倍率

  return nav2_util::CallbackReturn::SUCCESS;
}

// 激活：激活发布器并发布一帧过滤器信息，随后创建 bond 连接
nav2_util::CallbackReturn
CostmapFilterInfoServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  publisher_->on_activate();
  auto costmap_filter_info = std::make_unique<nav2_msgs::msg::CostmapFilterInfo>(msg_);
  publisher_->publish(std::move(costmap_filter_info));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

// 停用：停用发布器并销毁 bond 连接
nav2_util::CallbackReturn
CostmapFilterInfoServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  publisher_->on_deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

// 清理：释放发布器
nav2_util::CallbackReturn
CostmapFilterInfoServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  publisher_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

// 关闭：仅打印日志
nav2_util::CallbackReturn
CostmapFilterInfoServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace nav2_map_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
// 注册组件节点宏：可作为 rclcpp_components 组件动态加载运行
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_map_server::CostmapFilterInfoServer)
