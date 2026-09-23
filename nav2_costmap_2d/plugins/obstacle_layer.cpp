/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Steve Macenski
 *********************************************************************/
// ============================================================================
// 中文注释：障碍层（ObstacleLayer）实现文件
//
// 【文件功能】
// 本文件实现 Nav2 代价地图插件体系中的"障碍层"（2D 感知层）。它订阅
// 传感器数据（LaserScan 激光扫描 或 PointCloud2 点云），把实时观测到的
// 障碍物写入代价地图；同时通过"射线追踪（raytrace）"机制清除传感器
// 视线范围内已经消失的障碍（标记 marking / 清除 clearing 两种观测）。
//
// 【在 Nav2 中的位置】
// LayeredCostmap 分层体系的感知主力：
//   - 全局代价地图：StaticLayer + ObstacleLayer + InflationLayer
//   - 局部代价地图：ObstacleLayer（滚动窗口）+ InflationLayer
// 本层继承自 CostmapLayer（其又继承 Layer 和 Costmap2D），
// 因此自身持有一份与主地图同尺寸的局部栅格缓存 costmap_。
//
// 【核心概念】
// 1. observation_sources 参数：形如 "scan: {...} pointcloud: {...}"，
//    每个源可配置话题、坐标系、marking/clearing、距离上下限等；
//    本文件解析该字符串并为每个源创建 ObservationBuffer + 订阅。
// 2. marking（标记）：把点云中满足高度/距离条件的点写为 LETHAL_OBSTACLE。
//    clearing（清除）：从传感器原点向每个观测点做射线追踪，
//    沿途格子若无障碍则置为 FREE_SPACE——"我看到那里没有东西"。
// 3. message_filters + tf2_ros::MessageFilter：只有当消息能被 TF 变换到
//    目标坐标系时才触发回调，避免处理无法定位的传感器数据。
// 4. sensor QoS：使用 rmw_qos_profile_sensor_data（best-effort，深度 50），
//    匹配传感器数据的实时性需求。
// 5. 与主地图合并（combination_method）：0=覆盖 Overwrite、1=取最大值
//    Maximum（默认，更安全）、其他=不合并。
// ============================================================================
#include "nav2_costmap_2d/obstacle_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

// pluginlib 导出宏：注册 ObstacleLayer 为 Layer 插件，供 YAML 配置动态加载
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::ObstacleLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

using nav2_costmap_2d::ObservationBuffer;
using nav2_costmap_2d::Observation;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

// 析构：注销动态参数回调与所有消息过滤器通知器
ObstacleLayer::~ObstacleLayer()
{
  dyn_params_handler_.reset();
  for (auto & notifier : observation_notifiers_) {
    notifier.reset();
  }
}

// ============================================================================
// onInitialize()：插件初始化——读取参数、解析 observation_sources、
// 为每个传感器源创建观测缓冲区和话题订阅
// ============================================================================
void ObstacleLayer::onInitialize()
{
  bool track_unknown_space;
  double transform_tolerance;

  // The topics that we'll subscribe to from the parameter server
  // 观测源描述字符串，如 "scan: {topic: /scan, data_type: LaserScan, ...}"
  std::string topics_string;

  // ---- 参数声明（层私有参数，实际名为 "<层名>.<参数>"）----
  declareParameter("enabled", rclcpp::ParameterValue(true));                       // 是否启用本层
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));    // 是否清除足迹下代价
  declareParameter("min_obstacle_height", rclcpp::ParameterValue(0.0));            // 障碍最小高度（米）
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));            // 障碍最大高度（米）
  declareParameter("combination_method", rclcpp::ParameterValue(1));               // 与主地图合并方式
  declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));// 观测源配置串

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // ---- 参数读取 ----
  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
  node->get_parameter(name_ + "." + "min_obstacle_height", min_obstacle_height_);
  node->get_parameter(name_ + "." + "max_obstacle_height", max_obstacle_height_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  node->get_parameter("track_unknown_space", track_unknown_space);   // 全局参数：是否保留未知
  node->get_parameter("transform_tolerance", transform_tolerance);   // 全局参数：TF 容差
  node->get_parameter(name_ + "." + "observation_sources", topics_string);

  // 注册动态参数回调（运行时调参）
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ObstacleLayer::dynamicParametersCallback,
      this,
      std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "Subscribed to Topics: %s", topics_string.c_str());

  // 是否为滚动窗口模式（局部代价地图跟随机器人移动）
  rolling_window_ = layered_costmap_->isRolling();

  // 未知区域处理：默认代价取 NO_INFORMATION 或 FREE_SPACE
  if (track_unknown_space) {
    default_value_ = NO_INFORMATION;
  } else {
    default_value_ = FREE_SPACE;
  }

  // 按主地图尺寸初始化本层内部 costmap_ 缓存
  ObstacleLayer::matchSize();
  current_ = true;
  was_reset_ = false;

  // 记录全局坐标系（订阅的传感器数据最终都要变换到该坐标系）
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // 订阅选项：把回调放到专用回调组，避免与节点其他回调互相阻塞
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;

  // now we need to split the topics based on whitespace which we can use a stringstream for
  // 按空白切分 observation_sources，得到一个个源名（如 "scan"、"pointcloud"）
  std::stringstream ss(topics_string);

  std::string source;
  while (ss >> source) {
    // get the parameters for the specific topic
    // 每个源的私有参数（参数名形如 "<层名>.<源名>.<参数>"）
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking;

    // ---- 声明该源的参数及默认值 ----
    declareParameter(source + "." + "topic", rclcpp::ParameterValue(source));                       // 话题名
    declareParameter(source + "." + "sensor_frame", rclcpp::ParameterValue(std::string("")));       // 传感器坐标系
    declareParameter(source + "." + "observation_persistence", rclcpp::ParameterValue(0.0));        // 观测保留时长（秒）
    declareParameter(source + "." + "expected_update_rate", rclcpp::ParameterValue(0.0));           // 期望更新频率（判失联）
    declareParameter(source + "." + "data_type", rclcpp::ParameterValue(std::string("LaserScan")));// 消息类型
    declareParameter(source + "." + "min_obstacle_height", rclcpp::ParameterValue(0.0));            // 源级高度过滤下限
    declareParameter(source + "." + "max_obstacle_height", rclcpp::ParameterValue(0.0));            // 源级高度过滤上限
    declareParameter(source + "." + "inf_is_valid", rclcpp::ParameterValue(false));                 // Inf 是否视为有效测距
    declareParameter(source + "." + "marking", rclcpp::ParameterValue(true));                       // 是否用于标记障碍
    declareParameter(source + "." + "clearing", rclcpp::ParameterValue(false));                     // 是否用于清除障碍
    declareParameter(source + "." + "obstacle_max_range", rclcpp::ParameterValue(2.5));             // 标记障碍最大距离
    declareParameter(source + "." + "obstacle_min_range", rclcpp::ParameterValue(0.0));             // 标记障碍最小距离
    declareParameter(source + "." + "raytrace_max_range", rclcpp::ParameterValue(3.0));             // 清除射线最大距离
    declareParameter(source + "." + "raytrace_min_range", rclcpp::ParameterValue(0.0));             // 清除射线最小距离

    // ---- 读取该源的参数 ----
    node->get_parameter(name_ + "." + source + "." + "topic", topic);
    node->get_parameter(name_ + "." + source + "." + "sensor_frame", sensor_frame);
    node->get_parameter(
      name_ + "." + source + "." + "observation_persistence",
      observation_keep_time);
    node->get_parameter(
      name_ + "." + source + "." + "expected_update_rate",
      expected_update_rate);
    node->get_parameter(name_ + "." + source + "." + "data_type", data_type);
    node->get_parameter(name_ + "." + source + "." + "min_obstacle_height", min_obstacle_height);
    node->get_parameter(name_ + "." + source + "." + "max_obstacle_height", max_obstacle_height);
    node->get_parameter(name_ + "." + source + "." + "inf_is_valid", inf_is_valid);
    node->get_parameter(name_ + "." + source + "." + "marking", marking);
    node->get_parameter(name_ + "." + source + "." + "clearing", clearing);

    // 数据类型只支持 PointCloud2 和 LaserScan 两种
    if (!(data_type == "PointCloud2" || data_type == "LaserScan")) {
      RCLCPP_FATAL(
        logger_,
        "Only topics that use point cloud2s or laser scans are currently supported");
      throw std::runtime_error(
              "Only topics that use point cloud2s or laser scans are currently supported");
    }

    // get the obstacle range for the sensor
    // 读取标记（obstacle）距离上下限——超出该距离的点不会写入障碍
    double obstacle_max_range, obstacle_min_range;
    node->get_parameter(name_ + "." + source + "." + "obstacle_max_range", obstacle_max_range);
    node->get_parameter(name_ + "." + source + "." + "obstacle_min_range", obstacle_min_range);

    // get the raytrace ranges for the sensor
    // 读取清除（raytrace）距离上下限——清除射线只在该范围内执行
    double raytrace_max_range, raytrace_min_range;
    node->get_parameter(name_ + "." + source + "." + "raytrace_min_range", raytrace_min_range);
    node->get_parameter(name_ + "." + source + "." + "raytrace_max_range", raytrace_max_range);


    RCLCPP_DEBUG(
      logger_,
      "Creating an observation buffer for source %s, topic %s, frame %s",
      source.c_str(), topic.c_str(),
      sensor_frame.c_str());

    // create an observation buffer
    // 为该源创建观测缓冲区：负责把传感器数据变换到 global_frame_、
    // 做高度/距离过滤、按保留时长缓存观测
    observation_buffers_.push_back(
      std::shared_ptr<ObservationBuffer
      >(
        new ObservationBuffer(
          node, topic, observation_keep_time, expected_update_rate,
          min_obstacle_height,
          max_obstacle_height, obstacle_max_range, obstacle_min_range, raytrace_max_range,
          raytrace_min_range, *tf_,
          global_frame_,
          sensor_frame, tf2::durationFromSec(transform_tolerance))));

    // check if we'll add this buffer to our marking observation buffers
    // marking 源加入"标记缓冲区"列表——updateBounds 时用于写入障碍
    if (marking) {
      marking_buffers_.push_back(observation_buffers_.back());
    }

    // check if we'll also add this buffer to our clearing observation buffers
    // clearing 源加入"清除缓冲区"列表——updateBounds 时用于射线清障
    if (clearing) {
      clearing_buffers_.push_back(observation_buffers_.back());
    }

    RCLCPP_DEBUG(
      logger_,
      "Created an observation buffer for source %s, topic %s, global frame: %s, "
      "expected update rate: %.2f, observation persistence: %.2f",
      source.c_str(), topic.c_str(),
      global_frame_.c_str(), expected_update_rate, observation_keep_time);

    // 传感器数据 QoS：best-effort、深度 50（容忍少量丢包，保证实时性）
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 50;

    // create a callback for the topic
    // 按消息类型分别创建订阅 + tf2_ros::MessageFilter
    if (data_type == "LaserScan") {
      // LaserScan 分支：message_filters 订阅 + TF 消息过滤器
      // MessageFilter 会缓存消息直到 TF 能把它变换到 global_frame_，
      // 变换不可用时消息被丢弃而不是送到回调，保证回调里 TF 一定可用
      auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
          rclcpp_lifecycle::LifecycleNode>>(node, topic, custom_qos_profile, sub_opt);
      sub->unsubscribe();   // 先取消订阅，等 activate() 时再启用（生命周期管理）

      auto filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        *sub, *tf_, global_frame_, 50,          // 队列长度 50
        node->get_node_logging_interface(),
        node->get_node_clock_interface(),
        tf2::durationFromSec(transform_tolerance));

      // inf_is_valid=true：把 +Inf 距离当作"打到最远处"的有效测量（用于清除）
      if (inf_is_valid) {
        filter->registerCallback(
          std::bind(
            &ObstacleLayer::laserScanValidInfCallback, this, std::placeholders::_1,
            observation_buffers_.back()));

      } else {
        filter->registerCallback(
          std::bind(
            &ObstacleLayer::laserScanCallback, this, std::placeholders::_1,
            observation_buffers_.back()));
      }

      observation_subscribers_.push_back(sub);

      observation_notifiers_.push_back(filter);
      observation_notifiers_.back()->setTolerance(rclcpp::Duration::from_seconds(0.05));

    } else {
      // PointCloud2 分支：处理方式与 LaserScan 类似
      auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
          rclcpp_lifecycle::LifecycleNode>>(node, topic, custom_qos_profile, sub_opt);
      sub->unsubscribe();

      // inf_is_valid 对点云没有意义（点云无 Inf 概念），给出警告
      if (inf_is_valid) {
        RCLCPP_WARN(
          logger_,
          "obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      auto filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        *sub, *tf_, global_frame_, 50,
        node->get_node_logging_interface(),
        node->get_node_clock_interface(),
        tf2::durationFromSec(transform_tolerance));

      filter->registerCallback(
        std::bind(
          &ObstacleLayer::pointCloud2Callback, this, std::placeholders::_1,
          observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }

    // 指定了 sensor_frame 时，让 MessageFilter 同时等待
    // global_frame_ 和 sensor_frame_ 两个目标坐标系的 TF 可用
    if (sensor_frame != "") {
      std::vector<std::string> target_frames;
      target_frames.push_back(global_frame_);
      target_frames.push_back(sensor_frame);
      observation_notifiers_.back()->setTargetFrames(target_frames);
    }
  }
}

// ============================================================================
// 动态参数回调：运行时可调整高度过滤、启用开关、合并方式等
// ============================================================================
rcl_interfaces::msg::SetParametersResult
ObstacleLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      // 高度上下限可直接热更新
      if (param_name == name_ + "." + "min_obstacle_height") {
        min_obstacle_height_ = parameter.as_double();
      } else if (param_name == name_ + "." + "max_obstacle_height") {
        max_obstacle_height_ = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      // 启用开关：重新启用时标记数据不最新，等待重新填充
      if (param_name == name_ + "." + "enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        if (enabled_) {
          current_ = false;
        }
      } else if (param_name == name_ + "." + "footprint_clearing_enabled") {
        footprint_clearing_enabled_ = parameter.as_bool();
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      // 合并方式可热切换（0=覆盖 1=最大值）
      if (param_name == name_ + "." + "combination_method") {
        combination_method_ = parameter.as_int();
      }
    }
  }

  result.successful = true;
  return result;
}

// ============================================================================
// laserScanCallback()：LaserScan 消息回调
// 把激光扫描投影为点云（并变换到 global_frame_），存入观测缓冲区
// ============================================================================
void
ObstacleLayer::laserScanCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr message,
  const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer)
{
  // project the laser into a point cloud
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = message->header;

  // project the scan into a point cloud
  // 优先做"带 TF 变换的投影"（高保真，输出在全局坐标系）；
  // TF 失败时退化为本地投影（无坐标变换）
  try {
    projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "High fidelity enabled, but TF returned a transform exception to frame %s: %s",
      global_frame_.c_str(),
      ex.what());
    projector_.projectLaser(*message, cloud);
  } catch (std::runtime_error & ex) {
    // 消息本身畸形（如 ranges 与 intensities 长度不匹配），直接丢弃
    RCLCPP_WARN(
      logger_,
      "transformLaserScanToPointCloud error, it seems the message from laser is malformed."
      " Ignore this message. what(): %s",
      ex.what());
    return;
  }

  // buffer the point cloud
  // 加锁写入观测缓冲区（与更新线程互斥）
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

// ============================================================================
// laserScanValidInfCallback()：带 Inf 处理的 LaserScan 回调
// inf_is_valid=true 时，把 +Inf（未打到障碍）改写为 range_max-ε，
// 使"最远可见距离"也能参与射线清除
// ============================================================================
void
ObstacleLayer::laserScanValidInfCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr raw_message,
  const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer)
{
  // Filter positive infinities ("Inf"s) to max_range.
  float epsilon = 0.0001;  // a tenth of a millimeter
  sensor_msgs::msg::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++) {
    float range = message.ranges[i];
    // 正无穷 → 替换为最大量程减一个极小量
    if (!std::isfinite(range) && range > 0) {
      message.ranges[i] = message.range_max - epsilon;
    }
  }

  // project the laser into a point cloud
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = message.header;

  // project the scan into a point cloud
  // 与 laserScanCallback 相同的投影流程：优先 TF 变换，失败降级
  try {
    projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "High fidelity enabled, but TF returned a transform exception to frame %s: %s",
      global_frame_.c_str(), ex.what());
    projector_.projectLaser(message, cloud);
  } catch (std::runtime_error & ex) {
    RCLCPP_WARN(
      logger_,
      "transformLaserScanToPointCloud error, it seems the message from laser is malformed."
      " Ignore this message. what(): %s",
      ex.what());
    return;
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

// ============================================================================
// pointCloud2Callback()：PointCloud2 消息回调——直接缓冲点云
// （无需投影转换，点云本身就是 3D 点集）
// ============================================================================
void
ObstacleLayer::pointCloud2Callback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
  const std::shared_ptr<ObservationBuffer> & buffer)
{
  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(*message);
  buffer->unlock();
}

// ============================================================================
// updateBounds()：分层更新第一阶段——
// 处理滚动窗口原点平移、清除观测（射线追踪）、标记新障碍，
// 并计算本层影响的世界坐标边界
// ============================================================================
void
ObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  // 滚动窗口模式：把本层地图原点平移到以机器人为中心
  // （地图数据会随原点平移而搬移，见 Costmap2D::updateOrigin）
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
  // 合并额外边界（其他层请求本层重绘的区域）
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  // get the marking observations
  // 收集所有 marking 源的观测（isCurrent 判断传感器是否超时失联）
  current = current && getMarkingObservations(observations);

  // get the clearing observations
  // 收集所有 clearing 源的观测
  current = current && getClearingObservations(clearing_observations);

  // update the global current status
  current_ = current;   // 任一缓冲区过时则本层不"最新"

  // raytrace freespace
  // 清除阶段：对每条清除观测做射线追踪，把视线内无障碍格置为 FREE_SPACE
  for (unsigned int i = 0; i < clearing_observations.size(); ++i) {
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  // 标记阶段：遍历每条 marking 观测的每个点，过滤后写入 LETHAL_OBSTACLE
  for (std::vector<Observation>::const_iterator it = observations.begin();
    it != observations.end(); ++it)
  {
    const Observation & obs = *it;

    const sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);

    // 距离上下限换算成格数（与射线清除算法保持同一度量）
    const unsigned int max_range_cells = cellDistance(obs.obstacle_max_range_);
    const unsigned int min_range_cells = cellDistance(obs.obstacle_min_range_);

    // 传感器原点（机器人/激光位置）转为栅格坐标，失败则跳过该观测
    unsigned int x0, y0;
    if (!worldToMap(obs.origin_.x, obs.origin_.y, x0, y0)) {
      RCLCPP_DEBUG(logger_, "Sensor origin is out of map bounds");
      continue;
    }

    // PointCloud2 迭代器：直接在扁平的字节数组上按字段名 "x"/"y"/"z" 遍历
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      double px = *iter_x, py = *iter_y, pz = *iter_z;

      // if the obstacle is too low, we won't add it
      // 高度过滤：太低的点（地面、台阶）不算障碍
      if (pz < min_obstacle_height_) {
        RCLCPP_DEBUG(logger_, "The point is too low");
        continue;
      }

      // if the obstacle is too high or too far away from the robot we won't add it
      // 太高的点（天花板、桌面以上）对 2D 导航无意义
      if (pz > max_obstacle_height_) {
        RCLCPP_DEBUG(logger_, "The point is too high");
        continue;
      }

      // now we need to compute the map coordinates for the observation
      // 世界坐标 → 本层栅格坐标（考虑分辨率与原点）
      unsigned int mx, my;
      if (!worldToMap(px, py, mx, my)) {
        RCLCPP_DEBUG(logger_, "Computing map coords failed");
        continue;
      }

      // compute the distance from the hitpoint to the pointcloud's origin
      // Calculate the distance in cell space to match the ray trace algorithm
      // used for clearing obstacles (see Costmap2D::raytraceLine).
      // 计算点到传感器原点的距离（格），与清除算法使用同一度量
      const int dx = static_cast<int>(mx) - static_cast<int>(x0);
      const int dy = static_cast<int>(my) - static_cast<int>(y0);
      const unsigned int dist = static_cast<unsigned int>(
        std::hypot(static_cast<double>(dx), static_cast<double>(dy)));

      // if the point is far enough away... we won't consider it
      // 超出标记最大距离的点忽略（远处定位误差大）
      if (dist > max_range_cells) {
        RCLCPP_DEBUG(logger_, "The point is too far away");
        continue;
      }

      // if the point is too close, do not consider it
      // 小于最小距离的点忽略（通常是机器人自身或传感器噪声）
      if (dist < min_range_cells) {
        RCLCPP_DEBUG(logger_, "The point is too close");
        continue;
      }

      // 通过全部过滤：写入致命障碍，并把该点扩展进更新边界
      // 一维下标 index = mx + my * size_x
      unsigned int index = getIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;
      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }

  // 足迹顶点也扩展进边界（配合足迹清除）
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

// ============================================================================
// updateFootprint()：把机器人足迹顶点扩展进更新边界
// ============================================================================
void
ObstacleLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {return;}
  // 足迹多边形按当前位姿旋转平移到世界坐标
  transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

// ============================================================================
// updateCosts()：分层更新第二阶段——把本层 costmap_ 合并进主地图
// ============================================================================
void
ObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  // if not current due to reset, set current now after clearing
  // 刚执行过 reset()（例如 clear costmap 服务）：清空后视为重新"最新"
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }

  // 足迹清除：机器人自身占据区域置为自由空间，防止被自身卡死
  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }

  // 三种合并方式：
  //   0=Overwrite：本层数据直接覆盖主地图对应区域；
  //   1=Maximum（默认）：逐格取较大值，保留更危险的信息，更安全；
  //   其他：不合并（本层独立，通常仅调试用）
  switch (combination_method_) {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

// ============================================================================
// addStaticObservation()：注入"静态观测"（不随时间过期的观测）
// 供外部 API（如 clear costmap 服务或测试）手动添加标记/清除观测
// ============================================================================
void
ObstacleLayer::addStaticObservation(
  nav2_costmap_2d::Observation & obs,
  bool marking, bool clearing)
{
  if (marking) {
    static_marking_observations_.push_back(obs);
  }
  if (clearing) {
    static_clearing_observations_.push_back(obs);
  }
}

// ============================================================================
// clearStaticObservations()：清空静态观测
// ============================================================================
void
ObstacleLayer::clearStaticObservations(bool marking, bool clearing)
{
  if (marking) {
    static_marking_observations_.clear();
  }
  if (clearing) {
    static_clearing_observations_.clear();
  }
}

// ============================================================================
// getMarkingObservations()：从所有 marking 缓冲区收集观测
// 返回值：所有缓冲区是否"当前"（传感器没有超时失联）
// ============================================================================
bool
ObstacleLayer::getMarkingObservations(std::vector<Observation> & marking_observations) const
{
  bool current = true;
  // get the marking observations
  // 逐个加锁读取（缓冲区可能正被订阅回调线程写入）
  for (unsigned int i = 0; i < marking_buffers_.size(); ++i) {
    marking_buffers_[i]->lock();
    marking_buffers_[i]->getObservations(marking_observations);
    current = marking_buffers_[i]->isCurrent() && current;
    marking_buffers_[i]->unlock();
  }
  // 追加静态（手动注入的）标记观测
  marking_observations.insert(
    marking_observations.end(),
    static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

// ============================================================================
// getClearingObservations()：从所有 clearing 缓冲区收集观测（逻辑同上）
// ============================================================================
bool
ObstacleLayer::getClearingObservations(std::vector<Observation> & clearing_observations) const
{
  bool current = true;
  // get the clearing observations
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i) {
    clearing_buffers_[i]->lock();
    clearing_buffers_[i]->getObservations(clearing_observations);
    current = clearing_buffers_[i]->isCurrent() && current;
    clearing_buffers_[i]->unlock();
  }
  // 追加静态（手动注入的）清除观测
  clearing_observations.insert(
    clearing_observations.end(),
    static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}

// ============================================================================
// raytraceFreespace()：射线清除——障碍消失机制的核心
// 从传感器原点向清除观测中的每个点引一条线，沿途没有障碍的格子
// 置为 FREE_SPACE（"激光穿过去了，说明那里没有东西"）
// ============================================================================
void
ObstacleLayer::raytraceFreespace(
  const Observation & clearing_observation, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  // 传感器原点（观测生成位置）与点云
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  const sensor_msgs::msg::PointCloud2 & cloud = *(clearing_observation.cloud_);

  // get the map coordinates of the origin of the sensor
  // 原点必须在地图内，否则无法发射射线
  unsigned int x0, y0;
  if (!worldToMap(ox, oy, x0, y0)) {
    RCLCPP_WARN(
      logger_,
      "Sensor origin at (%.2f, %.2f) is out of map bounds (%.2f, %.2f) to (%.2f, %.2f). "
      "The costmap cannot raytrace for it.",
      ox, oy,
      origin_x_, origin_y_,
      origin_x_ + getSizeInMetersX(), origin_y_ + getSizeInMetersY());
    return;
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  // 预计算地图边界（世界坐标），用于裁剪超出地图的射线端点
  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;


  // 传感器原点本身也要扩展进更新边界
  touch(ox, oy, min_x, min_y, max_x, max_y);

  // for each point in the cloud, we want to trace a line from the origin
  // and clear obstacles along it
  // 遍历点云中每个点，向其发射一条清除射线
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    double wx = *iter_x;
    double wy = *iter_y;

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    // 射线方向向量 (a,b)：对越界端点做线性缩放，把射线截断到地图内
    double a = wx - ox;
    double b = wy - oy;

    // the minimum value to raytrace from is the origin
    // 端点在地图左侧 → 按比例缩放到 x=origin_x 边界上
    if (wx < origin_x) {
      double t = (origin_x - ox) / a;
      wx = origin_x;
      wy = oy + b * t;
    }
    // 端点在地图下侧 → 缩放到 y=origin_y 边界上
    if (wy < origin_y) {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // the maximum value to raytrace to is the end of the map
    // 端点超出右侧边界 → 截断到 map_end_x（留 1mm 余量防越界）
    if (wx > map_end_x) {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    // 端点超出上侧边界 → 截断到 map_end_y
    if (wy > map_end_y) {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    // 缩放后的端点转为栅格坐标
    unsigned int x1, y1;

    // check for legality just in case
    if (!worldToMap(wx, wy, x1, y1)) {
      continue;
    }

    // 清除射线的距离上下限（格数）
    unsigned int cell_raytrace_max_range = cellDistance(clearing_observation.raytrace_max_range_);
    unsigned int cell_raytrace_min_range = cellDistance(clearing_observation.raytrace_min_range_);
    // MarkCell：raytraceLine 的仿函数，把沿途格子设为 FREE_SPACE
    MarkCell marker(costmap_, FREE_SPACE);
    // and finally... we can execute our trace to clear obstacles along that line
    // 执行 Bresenham 直线追踪，从 (x0,y0) 到 (x1,y1) 清除沿途格子
    raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_max_range, cell_raytrace_min_range);

    // 把射线有效终点扩展进更新边界（让主地图知道这里被清除了）
    updateRaytraceBounds(
      ox, oy, wx, wy, clearing_observation.raytrace_max_range_,
      clearing_observation.raytrace_min_range_, min_x, min_y, max_x,
      max_y);
  }
}

// ============================================================================
// activate()：生命周期激活——恢复传感器订阅
// ============================================================================
void
ObstacleLayer::activate()
{
  // 清空消息过滤器中缓存的旧消息
  for (auto & notifier : observation_notifiers_) {
    notifier->clear();
  }

  // if we're stopped we need to re-subscribe to topics
  // 重新订阅所有传感器话题（deactivate 时被取消订阅）
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->subscribe();
    }
  }
  // 重置缓冲区时间戳，避免因停机期被判为传感器失联
  resetBuffersLastUpdated();
}

// ============================================================================
// deactivate()：生命周期去激活——暂停所有传感器订阅（不再收数据）
// ============================================================================
void
ObstacleLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->unsubscribe();
    }
  }
}

// ============================================================================
// updateRaytraceBounds()：把清除射线的有效终点扩展进更新边界
// 射线端点若超出 max_range 会按比例回缩到最大清除距离处
// ============================================================================
void
ObstacleLayer::updateRaytraceBounds(
  double ox, double oy, double wx, double wy, double max_range, double min_range,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  double dx = wx - ox, dy = wy - oy;
  double full_distance = hypot(dx, dy);
  // 短于最小清除距离的射线不产生边界更新
  if (full_distance < min_range) {
    return;
  }
  // 端点超出最大清除距离时按比例截断（scale<=1）
  double scale = std::min(1.0, max_range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

// ============================================================================
// reset()：完全复位本层——清空内部地图与观测缓存
// （由 clear_costmap 服务的恢复区域功能触发）
// ============================================================================
void
ObstacleLayer::reset()
{
  resetMaps();               // 清空本层 costmap_ 为默认值
  resetBuffersLastUpdated(); // 重置观测缓冲区时间戳
  current_ = false;
  was_reset_ = true;         // 标记"刚复位"，updateCosts 中恢复 current_
}

// ============================================================================
// resetBuffersLastUpdated()：重置所有观测缓冲区的"最后更新时间"
// 使 isCurrent() 重新从现在开始计时
// ============================================================================
void
ObstacleLayer::resetBuffersLastUpdated()
{
  for (unsigned int i = 0; i < observation_buffers_.size(); ++i) {
    if (observation_buffers_[i]) {
      observation_buffers_[i]->resetLastUpdated();
    }
  }
}

}  // namespace nav2_costmap_2d
