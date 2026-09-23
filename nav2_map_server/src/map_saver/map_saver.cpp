/*
 * Copyright (c) 2020 Samsung Research Russia
 * Copyright 2019 Rover Robotics
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// ============================================================================
// 文件说明（中文）：
// MapSaver（地图保存器节点）的实现。
// 模块定位：nav2_map_server 包中的地图保存服务节点，以 ROS2 生命周期节点
// 形式运行，对外提供 nav2_msgs/srv/SaveMap 服务（服务名 map_saver/save_map）。
// 收到保存请求后，临时订阅指定的地图话题（默认 "map"），等待一帧
// OccupancyGrid 消息到达，再调用 map_io 的 saveMapToFile 将其写为
// 图像文件 + YAML 元数据文件。
//
// ROS2 技术要点：
//   - QoS：地图话题常以 transient_local（瞬态本地）+ reliable 发布
//     （迟加入的订阅者也能收到最后一帧地图），本节点按参数
//     map_subscribe_transient_local 配置订阅端 QoS 与之匹配；
//   - 使用独立的 MutuallyExclusive 回调组 + SingleThreadedExecutor
//     spin_until_future_complete，实现"在服务回调内同步等待一帧话题消息"，
//     避免与节点主执行器死锁，并受 save_map_timeout 超时保护。
// ============================================================================

#include "nav2_map_server/map_saver.hpp"

#include <string>
#include <memory>
#include <stdexcept>
#include <functional>
#include <mutex>

using namespace std::placeholders;

namespace nav2_map_server
{
// ---------------------------------------------------------------------------
// 构造函数：声明节点参数
//   save_map_timeout            等待地图消息的超时时间（秒，默认 2.0）
//   free_thresh_default         空闲阈值默认值（请求未指定时使用）
//   occupied_thresh_default     占据阈值默认值
//   map_subscribe_transient_local 是否以 transient_local QoS 订阅地图话题
// ---------------------------------------------------------------------------
MapSaver::MapSaver(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("map_saver", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare the node parameters
  declare_parameter("save_map_timeout", 2.0);
  declare_parameter("free_thresh_default", 0.25);
  declare_parameter("occupied_thresh_default", 0.65);
  declare_parameter("map_subscribe_transient_local", true);
}

MapSaver::~MapSaver()
{
}

// ---------------------------------------------------------------------------
// 生命周期回调 on_configure：读取参数并创建 save_map 服务。
// ---------------------------------------------------------------------------
nav2_util::CallbackReturn
MapSaver::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Make name prefix for services
  // 服务名前缀：节点名 + "/"（即 map_saver/save_map）
  const std::string service_prefix = get_name() + std::string("/");

  save_map_timeout_ = std::make_shared<rclcpp::Duration>(
    rclcpp::Duration::from_seconds(get_parameter("save_map_timeout").as_double()));
  free_thresh_default_ = get_parameter("free_thresh_default").as_double();
  occupied_thresh_default_ = get_parameter("occupied_thresh_default").as_double();
  map_subscribe_transient_local_ = get_parameter("map_subscribe_transient_local").as_bool();

  // Create a service that saves the occupancy grid from map topic to a file
  // 创建 SaveMap 服务，回调绑定到 saveMapCallback
  save_map_service_ = create_service<nav2_msgs::srv::SaveMap>(
    service_prefix + save_map_service_name_,
    std::bind(&MapSaver::saveMapCallback, this, _1, _2, _3));

  return nav2_util::CallbackReturn::SUCCESS;
}

// 激活：创建 bond 连接（向生命周期管理器报告存活状态）
nav2_util::CallbackReturn
MapSaver::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

// 停用：销毁 bond 连接
nav2_util::CallbackReturn
MapSaver::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

// 清理：释放服务资源
nav2_util::CallbackReturn
MapSaver::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  save_map_service_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

// 关闭：仅打印日志
nav2_util::CallbackReturn
MapSaver::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// SaveMap 服务回调：把服务请求字段（地图话题、保存路径、图像格式、
// 阈值、模式）转成 SaveParameters 结构，调用 saveMapTopicToFile 执行，
// 并把成功/失败写入 response->result。
// ---------------------------------------------------------------------------
void MapSaver::saveMapCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::SaveMap::Request> request,
  std::shared_ptr<nav2_msgs::srv::SaveMap::Response> response)
{
  // Set input arguments and call saveMapTopicToFile()
  SaveParameters save_parameters;
  save_parameters.map_file_name = request->map_url;        // 保存文件路径（不含扩展名）
  save_parameters.image_format = request->image_format;   // 图像格式（pgm/png...）
  save_parameters.free_thresh = request->free_thresh;     // 空闲阈值
  save_parameters.occupied_thresh = request->occupied_thresh;  // 占据阈值
  try {
    // 解析模式字符串（trinary/scale/raw）
    save_parameters.mode = map_mode_from_string(request->map_mode);
  } catch (std::invalid_argument &) {
    // 无法识别的模式：回退默认 trinary 并告警
    save_parameters.mode = MapMode::Trinary;
    RCLCPP_WARN(
      get_logger(), "Map mode parameter not recognized: '%s', using default value (trinary)",
      request->map_mode.c_str());
  }

  response->result = saveMapTopicToFile(request->map_topic, save_parameters);
}

// ---------------------------------------------------------------------------
// 核心实现：订阅地图话题并等待一帧消息，然后保存到文件。
// 流程：
//   1. 补全默认值（话题名 "map"、阈值默认值）；
//   2. 用 promise/future 机制接收一帧 OccupancyGrid；
//   3. 按参数构造 QoS（transient_local + reliable + keep_last(1)）；
//   4. 在独立回调组 + 单线程执行器中 spin_until_future_complete
//      （带超时），避免阻塞节点主执行器；
//   5. 拿到地图后调用 saveMapToFile（map_io）写盘。
// ---------------------------------------------------------------------------
bool MapSaver::saveMapTopicToFile(
  const std::string & map_topic,
  const SaveParameters & save_parameters)
{
  // Local copies of map_topic and save_parameters that could be changed
  // 局部拷贝：后续补全默认值时可能修改
  std::string map_topic_loc = map_topic;
  SaveParameters save_parameters_loc = save_parameters;

  RCLCPP_INFO(
    get_logger(), "Saving map from \'%s\' topic to \'%s\' file",
    map_topic_loc.c_str(), save_parameters_loc.map_file_name.c_str());

  try {
    // Correct map_topic_loc if necessary
    // 话题名未指定时默认订阅 "map"
    if (map_topic_loc == "") {
      map_topic_loc = "map";
      RCLCPP_WARN(
        get_logger(), "Map topic unspecified. Map messages will be read from \'%s\' topic",
        map_topic_loc.c_str());
    }

    // Set default for MapSaver node thresholds parameters
    // 请求中阈值为 0（未指定）时使用节点参数中的默认值
    if (save_parameters_loc.free_thresh == 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Free threshold unspecified. Setting it to default value: %f",
        free_thresh_default_);
      save_parameters_loc.free_thresh = free_thresh_default_;
    }
    if (save_parameters_loc.occupied_thresh == 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Occupied threshold unspecified. Setting it to default value: %f",
        occupied_thresh_default_);
      save_parameters_loc.occupied_thresh = occupied_thresh_default_;
    }

    // promise/future：订阅回调收到地图消息后 set_value，
    // 等待端通过 future 同步获取结果
    std::promise<nav_msgs::msg::OccupancyGrid::SharedPtr> prom;
    std::future<nav_msgs::msg::OccupancyGrid::SharedPtr> future_result = prom.get_future();
    // A callback function that receives map message from subscribed topic
    auto mapCallback = [&prom](
      const nav_msgs::msg::OccupancyGrid::SharedPtr msg) -> void {
        prom.set_value(msg);
      };

    // QoS 配置：地图话题通常以 transient_local 发布
    // （发布者保留最后一帧，迟到的订阅者也能收到），
    // 订阅端需匹配相同的可靠性策略才能建立连接
    rclcpp::QoS map_qos(10);  // initialize to default
    if (map_subscribe_transient_local_) {
      map_qos.transient_local();
      map_qos.reliable();
      map_qos.keep_last(1);
    }

    // Create new CallbackGroup for map_sub
    // 为订阅创建独立回调组：服务回调运行在默认执行器上，
    // 若用同一执行器同步等待会死锁，故用独立组 + 新执行器
    auto callback_group = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);

    auto option = rclcpp::SubscriptionOptions();
    option.callback_group = callback_group;
    auto map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_loc, map_qos, mapCallback, option);

    // Create SingleThreadedExecutor to spin map_sub in callback_group
    // 单线程执行器仅处理该回调组，直到收到地图或超时
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_callback_group(callback_group, get_node_base_interface());
    // Spin until map message received
    auto timeout = save_map_timeout_->to_chrono<std::chrono::nanoseconds>();
    auto status = executor.spin_until_future_complete(future_result, timeout);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      // 超时仍未收到地图消息：报错并失败返回
      RCLCPP_ERROR(get_logger(), "Failed to spin map subscription");
      return false;
    }
    // map_sub is no more needed
    map_sub.reset();
    // Map message received. Saving it to file
    // 拿到地图消息，调用 map_io 的保存函数写盘
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg = future_result.get();
    if (saveMapToFile(*map_msg, save_parameters_loc)) {
      RCLCPP_INFO(get_logger(), "Map saved successfully");
      return true;
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to save the map");
      return false;
    }
  } catch (std::exception & e) {
    // 任何异常（参数校验失败、写盘错误等）统一转为失败返回
    RCLCPP_ERROR(get_logger(), "Failed to save the map: %s", e.what());
    return false;
  }

  return false;
}

}  // namespace nav2_map_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
// 注册组件节点宏：可作为 rclcpp_components 组件动态加载运行
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_map_server::MapSaver)
