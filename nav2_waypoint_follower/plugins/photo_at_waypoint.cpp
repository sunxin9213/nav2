// Copyright (c) 2020 Fetullah Atas
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

#include "nav2_waypoint_follower/plugins/photo_at_waypoint.hpp"

#include <string>
#include <memory>

#include "pluginlib/class_list_macros.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_waypoint_follower
{
PhotoAtWaypoint::PhotoAtWaypoint()
{
}

PhotoAtWaypoint::~PhotoAtWaypoint()
{
}

void PhotoAtWaypoint::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  // 拍照插件在初始化阶段准备最新图像缓存、输出目录和相机订阅；
  // 到达 waypoint 时只保存缓存中的最近一帧，不在导航 Action 中等待新的图像到达。
  auto node = parent.lock();

  curr_frame_msg_ = std::make_shared<sensor_msgs::msg::Image>();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".enabled",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".image_topic",
    rclcpp::ParameterValue("/camera/color/image_raw"));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".save_dir",
    rclcpp::ParameterValue("/tmp/waypoint_images"));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".image_format",
    rclcpp::ParameterValue("png"));

  std::string save_dir_as_string;
  node->get_parameter(plugin_name + ".enabled", is_enabled_);
  node->get_parameter(plugin_name + ".image_topic", image_topic_);
  node->get_parameter(plugin_name + ".save_dir", save_dir_as_string);
  node->get_parameter(plugin_name + ".image_format", image_format_);

  // get inputted save directory and make sure it exists, if not log and create  it
  // 输出目录是插件能否正常工作的前置条件；创建失败时禁用插件，
  // 让上层不会继续执行一个必然无法保存图像的任务。
  save_dir_ = save_dir_as_string;
  try {
    if (!std::filesystem::exists(save_dir_)) {
      RCLCPP_WARN(
        logger_,
        "Provided save directory for photo at waypoint plugin does not exist,"
        "provided directory is: %s, the directory will be created automatically.",
        save_dir_.c_str()
      );
      if (!std::filesystem::create_directory(save_dir_)) {
        RCLCPP_ERROR(
          logger_,
          "Failed to create directory!: %s required by photo at waypoint plugin, "
          "exiting the plugin with failure!",
          save_dir_.c_str()
        );
        is_enabled_ = false;
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_, "Exception (%s) thrown while attempting to create image capture directory."
      " This task executor is being disabled as it cannot save images.", e.what());
    is_enabled_ = false;
  }

  if (!is_enabled_) {
    RCLCPP_INFO(
      logger_, "Photo at waypoint plugin is disabled.");
  } else {
    RCLCPP_INFO(
      logger_, "Initializing photo at waypoint plugin, subscribing to camera topic named; %s",
      image_topic_.c_str());
    camera_image_subscriber_ = node->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&PhotoAtWaypoint::imageCallback, this, std::placeholders::_1));
  }
}

bool PhotoAtWaypoint::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index)
{
  // 文件名同时包含 waypoint 索引和时间戳秒数，便于把照片与一次导航任务关联起来。
  // 当前帧的读取、ROS Image 到 OpenCV Mat 的转换和写盘处于同一异常处理范围内，
  // 任一环节失败都会返回 false，由 WaypointFollower 决定终止还是继续。
  if (!is_enabled_) {
    RCLCPP_WARN(
      logger_,
      "Photo at waypoint plugin is disabled. Not performing anything"
    );
    return true;
  }
  try {
    // construct the full path to image filename
    std::filesystem::path file_name = std::to_string(
      curr_waypoint_index) + "_" +
      std::to_string(curr_pose.header.stamp.sec) + "." + image_format_;
    std::filesystem::path full_path_image_path = save_dir_ / file_name;

    // save the taken photo at this waypoint to given directory
    std::lock_guard<std::mutex> guard(global_mutex_);
    cv::Mat curr_frame_mat;
    deepCopyMsg2Mat(curr_frame_msg_, curr_frame_mat);
    cv::imwrite(full_path_image_path.c_str(), curr_frame_mat);
    RCLCPP_INFO(
      logger_,
      "Photo has been taken sucessfully at waypoint %i", curr_waypoint_index);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_,
      "Couldn't take photo at waypoint %i! Caught exception: %s \n"
      "Make sure that the image topic named: %s is valid and active!",
      curr_waypoint_index,
      e.what(), image_topic_.c_str());
    return false;
  }
  return true;
}

void PhotoAtWaypoint::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // 相机回调与 processAtWaypoint() 可能运行在不同执行路径，
  // 用同一把互斥锁保护“最新帧指针”的替换和读取，避免拍照时拿到不一致的消息状态。
  std::lock_guard<std::mutex> guard(global_mutex_);
  curr_frame_msg_ = msg;
}

void PhotoAtWaypoint::deepCopyMsg2Mat(
  const sensor_msgs::msg::Image::SharedPtr & msg,
  cv::Mat & mat)
{
  // cv_bridge 先把 ROS Image 映射为 OpenCV 图像；随后复制到独立 Mat，
  // 使写盘过程不依赖相机回调继续持有的消息内存。
  cv_bridge::CvImageConstPtr cv_bridge_ptr = cv_bridge::toCvShare(msg, msg->encoding);
  cv::Mat frame = cv_bridge_ptr->image;
  if (msg->encoding == "rgb8") {
    cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
  }
  frame.copyTo(mat);
}

}      // namespace nav2_waypoint_follower
PLUGINLIB_EXPORT_CLASS(
  nav2_waypoint_follower::PhotoAtWaypoint,
  nav2_core::WaypointTaskExecutor)
