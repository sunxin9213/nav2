// Copyright (c) 2025, Open Navigation LLC
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
// limitations under the License. Reserved.

// ============================================================================
// 文件说明（中文）：
// GraphLoader（路图加载器）的实现。
// 模块定位：nav2_route 中负责路图（Graph）装载的组件：
//   1. 通过 pluginlib 加载 GraphFileLoader 插件（默认 GeoJson 格式），
//      实现对文件格式（GeoJSON / 未来 SQL 等）的解耦；
//   2. loadGraphFromParameter：按 graph_filepath 参数在节点配置时加载；
//      loadGraphFromFile：按指定路径加载（set_route_graph 服务用）；
//   3. transformGraph：把图中各节点坐标从其自带 frame_id 变换到
//      统一的 route_frame（如 map），使用 TF 并按坐标系缓存变换结果。
// ============================================================================

#include <memory>

#include "nav2_route/graph_loader.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace nav2_route
{

// ---------------------------------------------------------------------------
// 构造函数：
//   - 创建 pluginlib 类加载器（基库 nav2_route，接口 GraphFileLoader）；
//   - 读取 graph_filepath 参数（可空，等待运行时设置）；
//   - 读取 graph_file_loader 参数选择文件解析插件（默认 GeoJson）；
//   - 创建并配置该插件实例。
// ---------------------------------------------------------------------------
GraphLoader::GraphLoader(
  nav2_util::LifecycleNode::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame)
: plugin_loader_("nav2_route", "nav2_route::GraphFileLoader"),
  default_plugin_id_("GeoJsonGraphFileLoader")
{
  logger_ = node->get_logger();
  tf_ = tf;
  route_frame_ = frame;

  // 路图文件路径参数（允许为空：稍后通过 set_route_graph 服务设置）
  nav2_util::declare_parameter_if_not_declared(
    node, "graph_filepath", rclcpp::ParameterValue(std::string("")));
  graph_filepath_ = node->get_parameter("graph_filepath").as_string();

  // Default Graph Parser
  // 文件解析插件选择参数（默认 GeoJson）；默认时声明插件类型参数
  nav2_util::declare_parameter_if_not_declared(
    node, "graph_file_loader", rclcpp::ParameterValue(default_plugin_id_));
  auto graph_file_loader_id = node->get_parameter("graph_file_loader").as_string();
  if (graph_file_loader_id == default_plugin_id_) {
    nav2_util::declare_parameter_if_not_declared(
      node, default_plugin_id_ + ".plugin",
      rclcpp::ParameterValue("nav2_route::GeoJsonGraphFileLoader"));
  }

  // Create graph file loader plugin
  // 创建解析插件实例并配置；失败则抛出（导致节点配置失败）
  try {
    plugin_type_ = nav2_util::get_plugin_type_param(node, graph_file_loader_id);
    graph_file_loader_ = plugin_loader_.createSharedInstance((plugin_type_));
    RCLCPP_INFO(
      logger_, "Created GraphFileLoader %s of type %s",
      graph_file_loader_id.c_str(), plugin_type_.c_str());
    graph_file_loader_->configure(node);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      logger_,
      "Failed to create GraphFileLoader. Exception: %s", ex.what());
    throw ex;
  }
}

// ---------------------------------------------------------------------------
// 从指定文件路径加载路图：插件解析 + TF 坐标变换到 route_frame。
// 成功返回 true；路径为空、解析失败或变换失败均返回 false。
// ---------------------------------------------------------------------------
bool GraphLoader::loadGraphFromFile(
  Graph & graph,
  GraphToIDMap & graph_to_id_map,
  const std::string & filepath)
{
  if (filepath.empty()) {
    RCLCPP_ERROR(
      logger_, "The graph filepath was not provided.");
    return false;
  }

  RCLCPP_INFO(
    logger_,
    "Loading graph file from %s, by parser %s", filepath.c_str(), plugin_type_.c_str());

  // 调用解析插件读取文件，填充图与 ID 映射
  if (!graph_file_loader_->loadGraphFromFile(graph, graph_to_id_map, filepath)) {
    return false;
  }

  // 将各节点坐标统一变换到 route_frame
  if (!transformGraph(graph)) {
    RCLCPP_WARN(
      logger_,
      "Failed to transform nodes graph file (%s) to %s!", filepath.c_str(), route_frame_.c_str());
    return false;
  }

  return true;
}

// ---------------------------------------------------------------------------
// 按 graph_filepath 参数加载路图（节点配置阶段调用）。
// 与 loadGraphFromFile 的区别：路径为空视为正常（尚未设置图），
// 返回 true 但不加载任何内容。
// ---------------------------------------------------------------------------
bool GraphLoader::loadGraphFromParameter(
  Graph & graph,
  GraphToIDMap & graph_to_id_map)
{
  if (graph_filepath_.empty()) {
    RCLCPP_INFO(logger_, "No graph file provided to load yet.");
    return true;
  }

  RCLCPP_INFO(
    logger_,
    "Loading graph file from %s, by parser %s", graph_filepath_.c_str(), plugin_type_.c_str());

  if (!graph_file_loader_->loadGraphFromFile(graph, graph_to_id_map, graph_filepath_)) {
    return false;
  }

  if (!transformGraph(graph)) {
    RCLCPP_WARN(
      logger_,
      "Failed to transform nodes graph file (%s) to %s!",
      graph_filepath_.c_str(), route_frame_.c_str());
    return false;
  }

  return true;
}

// ---------------------------------------------------------------------------
// 把图中各节点坐标从其原始坐标系变换到 route_frame：
//   - 坐标系为空或已是 route_frame 的节点跳过；
//   - 每个不同坐标系只查询一次 TF（0.1s 容忍超时）并缓存结果；
//   - 对每个节点坐标（z 取 0）应用缓存的变换并写回。
// 任一坐标系变换失败则整体失败。
// ---------------------------------------------------------------------------
bool GraphLoader::transformGraph(Graph & graph)
{
  std::unordered_map<std::string, tf2::Transform> cached_transforms;
  for (auto & node : graph) {
    std::string node_frame = node.coords.frame_id;
    // 已经是目标坐标系（或未指定）：无需变换
    if (node_frame.empty() || node_frame == route_frame_) {
      continue;
    }

    // 该坐标系首次遇到：查询 TF node_frame -> route_frame 并缓存
    if (cached_transforms.find(node_frame) == cached_transforms.end()) {
      tf2::Transform tf_transform;
      bool got_transform = nav2_util::getTransform(
        node_frame, route_frame_, tf2::durationFromSec(0.1), tf_, tf_transform);

      if (!got_transform) {
        return false;
      }

      cached_transforms.insert({node_frame, tf_transform});
    }

    // 对节点坐标（平面，z=0）应用变换
    tf2::Vector3 graph_coord(
      node.coords.x,
      node.coords.y,
      0.0);

    tf2::Vector3 new_coord = cached_transforms[node_frame] * graph_coord;

    // 写回变换后的坐标，并统一坐标系标记为 route_frame
    node.coords.x = static_cast<float>(new_coord.x());
    node.coords.y = static_cast<float>(new_coord.y());
    node.coords.frame_id = route_frame_;
  }

  return true;
}

}  // namespace nav2_route
