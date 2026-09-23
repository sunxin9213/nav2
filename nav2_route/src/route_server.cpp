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
// RouteServer（路线服务器）的实现 —— nav2_route 模块的核心节点。
// 模块定位：基于预定义路图（graph，如厂区车道/走廊拓扑图）的导航服务器，
// 是 Nav2 中"路线（route）"概念的实现：在节点-边构成的图上搜索路线，
// 再把路线加密为稠密路径供控制器执行。
//
// 对外接口（ROS2 生命周期节点）：
//   动作：
//     - compute_route（nav2_msgs/action/ComputeRoute）：
//       一次性计算路线并返回路径；
//     - compute_and_track_route（ComputeAndTrackRoute）：
//       计算路线后持续跟踪执行（发布反馈、支持重路由/阻塞处理），
//       直到任务完成或被取消。
//   服务：
//     - <节点名>/set_route_graph（SetRouteGraph）：运行时更换路图文件。
//   话题：
//     - route_graph：路图可视化 MarkerArray（transient_local）。
//
// 主要协作组件：
//   GraphLoader         加载 GeoJSON 等格式的路图并做 TF 坐标变换；
//   GoalIntentExtractor 从请求（位姿或节点 ID）确定起点/终点图节点；
//   RoutePlanner        图搜索（Dijkstra/A*）求最短路线；
//   RouteTracker        ComputeAndTrack 模式下的路线跟踪与重路由；
//   PathConverter       路线 -> 稠密 nav_msgs/Path（含拐角圆弧平滑）。
// ============================================================================

#include "nav2_route/route_server.hpp"

using nav2_util::declare_parameter_if_not_declared;
using std::placeholders::_1;
using std::placeholders::_2;

namespace nav2_route
{

// 构造函数：仅初始化生命周期节点框架，实质工作在 on_configure 中
RouteServer::RouteServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("route_server", "", options)
{}

// ---------------------------------------------------------------------------
// 生命周期回调 on_configure：
//   1. 创建 TF 缓冲与监听器；
//   2. 创建路图可视化发布器与两个动作服务器、set_route_graph 服务；
//   3. 声明并读取坐标系与规划时限参数；
//   4. 创建全局代价地图订阅器；
//   5. 构造并配置 GraphLoader / GoalIntentExtractor / RoutePlanner /
//      RouteTracker / PathConverter 五大协作组件。
// ---------------------------------------------------------------------------
nav2_util::CallbackReturn
RouteServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // TF 缓冲 + ROS 定时器接口 + TF 监听器（订阅 /tf、/tf_static）
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  auto node = shared_from_this();
  // 路图可视化发布器（latched：新订阅的 RViz 也能立即看到路图）
  graph_vis_publisher_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "route_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // 两个动作服务器：compute_route（一次性计算）与
  // compute_and_track_route（计算并跟踪），回调分别绑定到
  // computeRoute / computeAndTrackRoute
  compute_route_server_ = std::make_shared<ComputeRouteServer>(
    node, "compute_route",
    std::bind(&RouteServer::computeRoute, this),
    nullptr, std::chrono::milliseconds(500), true);

  compute_and_track_route_server_ = std::make_shared<ComputeAndTrackRouteServer>(
    node, "compute_and_track_route",
    std::bind(&RouteServer::computeAndTrackRoute, this),
    nullptr, std::chrono::milliseconds(500), true);

  // set_route_graph 服务：运行时从文件重新加载路图
  set_graph_service_ = node->create_service<nav2_msgs::srv::SetRouteGraph>(
    std::string(node->get_name()) + "/set_route_graph",
    std::bind(
      &RouteServer::setRouteGraph, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // 坐标系与规划时限参数：
  //   route_frame  路图数据的坐标系（默认 map）
  //   global_frame 全局坐标系（默认 map）
  //   base_frame   机器人本体坐标系（默认 base_link）
  //   max_planning_time 单次规划的最大时长（秒，超时告警）
  declare_parameter_if_not_declared(
    node, "route_frame", rclcpp::ParameterValue(std::string("map")));
  declare_parameter_if_not_declared(
    node, "global_frame", rclcpp::ParameterValue(std::string("map")));
  declare_parameter_if_not_declared(
    node, "base_frame", rclcpp::ParameterValue(std::string("base_link")));
  declare_parameter_if_not_declared(
    node, "max_planning_time", rclcpp::ParameterValue(2.0));

  route_frame_ = node->get_parameter("route_frame").as_string();
  global_frame_ = node->get_parameter("global_frame").as_string();
  base_frame_ = node->get_parameter("base_frame").as_string();
  max_planning_time_ = node->get_parameter("max_planning_time").as_double();

  // Create costmap subscriber
  // 全局代价地图订阅器（供边评分、目标意图提取、跟踪等使用）
  nav2_util::declare_parameter_if_not_declared(
    node, "costmap_topic",
    rclcpp::ParameterValue(std::string("global_costmap/costmap_raw")));
  std::string costmap_topic = node->get_parameter("costmap_topic").as_string();
  costmap_subscriber_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(node, costmap_topic);

  try {
    // 路图加载器：从 graph_filepath 参数加载路图（可为空，等待运行时设置）
    graph_loader_ = std::make_shared<GraphLoader>(node, tf_, route_frame_);
    if (!graph_loader_->loadGraphFromParameter(graph_, id_to_graph_map_)) {
      return nav2_util::CallbackReturn::FAILURE;
    }

    // 目标意图提取器：把请求中的起点/终点（位姿或 ID）映射为图节点
    goal_intent_extractor_ = std::make_shared<GoalIntentExtractor>();
    goal_intent_extractor_->configure(
      node, graph_, &id_to_graph_map_, tf_, costmap_subscriber_, route_frame_,
      global_frame_, base_frame_);

    // 路线规划器：图搜索（Dijkstra）求最短路线
    route_planner_ = std::make_shared<RoutePlanner>();
    route_planner_->configure(node, tf_, costmap_subscriber_);

    // 路线跟踪器：跟踪执行并处理阻塞/重路由
    route_tracker_ = std::make_shared<RouteTracker>();
    route_tracker_->configure(
      node, tf_, costmap_subscriber_, compute_and_track_route_server_, route_frame_, base_frame_);

    // 路径转换器：路线 -> 稠密路径（拐角圆弧平滑）
    path_converter_ = std::make_shared<PathConverter>();
    path_converter_->configure(node);
  } catch (std::exception & e) {
    // 任一组件配置失败都视为致命错误，节点配置失败
    RCLCPP_FATAL(get_logger(), "Failed to configure route server: %s", e.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

// 激活：激活动作服务器与发布器，发布一帧路图可视化，创建 bond
nav2_util::CallbackReturn
RouteServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  compute_route_server_->activate();
  compute_and_track_route_server_->activate();
  graph_vis_publisher_->on_activate();
  graph_vis_publisher_->publish(utils::toMsg(graph_, route_frame_, this->now()));
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

// 停用：与激活顺序相反，最后销毁 bond
nav2_util::CallbackReturn
RouteServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  compute_route_server_->deactivate();
  compute_and_track_route_server_->deactivate();
  graph_vis_publisher_->on_deactivate();
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

// 清理：释放全部组件与资源，清空路图
nav2_util::CallbackReturn
RouteServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  compute_route_server_.reset();
  compute_and_track_route_server_.reset();
  set_graph_service_.reset();
  graph_loader_.reset();
  route_planner_.reset();
  route_tracker_.reset();
  path_converter_.reset();
  goal_intent_extractor_.reset();
  graph_vis_publisher_.reset();
  transform_listener_.reset();
  tf_.reset();
  graph_.clear();
  return nav2_util::CallbackReturn::SUCCESS;
}

// 关闭：仅打印日志
nav2_util::CallbackReturn
RouteServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// 计算本次规划耗时；若超过 max_planning_time 则告警（频率不达标提示）
// ---------------------------------------------------------------------------
rclcpp::Duration
RouteServer::findPlanningDuration(const rclcpp::Time & start_time)
{
  auto cycle_duration = this->now() - start_time;
  if (max_planning_time_ > 0.0 && cycle_duration.seconds() > max_planning_time_) {
    RCLCPP_WARN(
      get_logger(),
      "Route planner missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
      1 / max_planning_time_, 1 / cycle_duration.seconds());
  }

  return cycle_duration;
}

// ---------------------------------------------------------------------------
// 模板工具：检查当前动作请求是否可继续处理。
// 返回 false 的情形（并相应终止动作）：
//   - 动作服务器为空或未激活（节点已停用）；
//   - 收到取消请求（terminate_all 终止所有目标）；
//   - 路图为空（无图可用）。
// ---------------------------------------------------------------------------
template<typename ActionT>
bool
RouteServer::isRequestValid(
  std::shared_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server)
{
  if (!action_server || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return false;
  }

  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling route planning action.");
    action_server->terminate_all();
    return false;
  }

  if (graph_.empty()) {
    RCLCPP_INFO(get_logger(), "No graph set! Aborting request.");
    action_server->terminate_current();
    return false;
  }

  return true;
}

// ComputeRoute 结果填充：路线消息 + 稠密路径 + 规划耗时
void RouteServer::populateActionResult(
  std::shared_ptr<ComputeRoute::Result> result,
  const Route & route,
  const nav_msgs::msg::Path & path,
  const rclcpp::Duration & planning_duration)
{
  result->route = utils::toMsg(route, route_frame_, this->now());
  result->path = path;
  result->planning_time = planning_duration;
}

// ComputeAndTrackRoute 结果填充：仅返回执行总时长
void RouteServer::populateActionResult(
  std::shared_ptr<ComputeAndTrackRoute::Result> result,
  const Route &,
  const nav_msgs::msg::Path &,
  const rclcpp::Duration & execution_duration)
{
  result->execution_duration = execution_duration;
}

// ---------------------------------------------------------------------------
// 模板核心：根据目标请求计算路线（含重路由支持）。
// 流程：
//   1. GoalIntentExtractor 从请求中解析起点/终点图节点；
//   2. 若处于重路由状态（rerouting_start_id 有效），用重路由起点
//      覆盖原起点（机器人当前所在边/位置）；
//   3. 起点 == 终点时直接返回单点路线；
//   4. 否则构造 RouteRequest（起终点 ID 与位姿）调用 RoutePlanner
//      做图搜索，得到节点-边序列；
//   5. 最后 pruneStartandGoal 剪枝：去掉起终点附近多余的边/边中点，
//      使路径从真实起点位姿直达终点位姿。
// ---------------------------------------------------------------------------
template<typename GoalT>
Route RouteServer::findRoute(
  const std::shared_ptr<const GoalT> goal,
  ReroutingState & rerouting_info)
{
  // Find the search boundaries
  // 解析请求，得到起点/终点对应的图节点下标
  auto [start_route, end_route] = goal_intent_extractor_->findStartandGoal(goal);

  // If we're rerouting, use the rerouting start node and pose as the new start
  // 重路由：从被阻塞边的下一节点重新出发，并覆盖起点位姿
  if (rerouting_info.rerouting_start_id != std::numeric_limits<unsigned int>::max()) {
    start_route = id_to_graph_map_.at(rerouting_info.rerouting_start_id);
    goal_intent_extractor_->overrideStart(rerouting_info.rerouting_start_pose);
  }

  Route route;
  if (start_route == end_route) {
    // Succeed with a single-point route
    // 起终点相同：单点路线，代价 0
    route.route_cost = 0.0;
    route.start_node = &graph_.at(start_route);
  } else {
    // Populate request data (start & goal id, start & goal pose, if set) for routing
    // 构造路线搜索请求（ID 与位姿；use_poses 表示是否使用位姿语义）
    RouteRequest route_request;
    route_request.start_nodeid = start_route;
    route_request.goal_nodeid = end_route;
    route_request.start_pose = goal_intent_extractor_->getStart();
    route_request.goal_pose = goal->goal;
    route_request.use_poses = goal->use_poses;

    // Compute the route via graph-search, returns a node-edge sequence
    // 图搜索求最短路线（考虑被阻塞的边/节点列表）
    route = route_planner_->findRoute(
      graph_, start_route, end_route, rerouting_info.blocked_ids, route_request);
  }

  // 剪枝起终点：使路线贴合真实起终点位姿
  return goal_intent_extractor_->pruneStartandGoal(route, goal, rerouting_info);
}

// ---------------------------------------------------------------------------
// 模板主流程：处理 compute_route / compute_and_track_route 两种动作的
// 公共逻辑（通过 ActionT 模板参数区分）。
// 循环直到成功/取消/异常：
//   1. 校验请求有效性（服务器激活、未取消、路图非空）；
//   2. 处理抢占：接受新目标并重置重路由状态；
//   3. findRoute 计算路线；
//   4. PathConverter 把路线加密为稠密路径（含拐角平滑）；
//   5. ComputeAndTrack 模式：RouteTracker::trackRoute 阻塞跟踪
//      （发布反馈；返回 INTERRUPTED 表示需要重路由/取消/抢占，
//      回到循环开头重新规划）；Compute 模式：填充结果并成功返回。
// 各类 nav2_core 路线异常被分别捕获，记录后以当前结果终止动作。
// ---------------------------------------------------------------------------
template<typename ActionT>
void
RouteServer::processRouteRequest(
  std::shared_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server)
{
  auto goal = action_server->get_current_goal();
  auto result = std::make_shared<typename ActionT::Result>();
  ReroutingState rerouting_info;
  auto start_time = this->now();

  try {
    while (rclcpp::ok()) {
      // 请求有效性检查（取消/停用/空图时终止动作并返回）
      if (!isRequestValid<ActionT>(action_server)) {
        return;
      }

      // 抢占处理：接受挂起的新目标，重置重路由状态后重新规划
      if (action_server->is_preempt_requested()) {
        RCLCPP_INFO(get_logger(), "Computing new preempted route to goal.");
        goal = action_server->accept_pending_goal();
        rerouting_info.reset();
      }

      // Find the route
      // 计算路线（含重路由信息）
      Route route = findRoute(goal, rerouting_info);
      RCLCPP_INFO(
        get_logger(), "Route found with %zu nodes and %zu edges",
        route.edges.size() + 1u, route.edges.size());
      // 路线 -> 稠密路径（重路由时起点为边上的最近点）
      auto path = path_converter_->densify(route, rerouting_info, route_frame_, this->now());

      // 编译期分支：两个动作类型的差异处理
      if (std::is_same<ActionT, ComputeAndTrackRoute>::value) {
        // blocks until re-route requested or task completion, publishes feedback
        // 跟踪模式：阻塞直到任务完成或被打断（期间发布反馈）
        switch (route_tracker_->trackRoute(route, path, rerouting_info)) {
          case TrackerResult::COMPLETED:
            // 到达终点：填充执行时长并成功返回
            populateActionResult(result, route, path, this->now() - start_time);
            action_server->succeeded_current(result);
            return;
          case TrackerResult::INTERRUPTED:
            // Reroute, cancel, or preempt requested
            // 被打断（重路由/取消/抢占）：回到循环开头重新规划
            break;
          case TrackerResult::EXITED:
            // rclcpp::ok() is false, so just return
            // rclcpp 已关闭：直接返回
            return;
        }
      } else {
        // Return route if not tracking
        // 非跟踪模式：一次性计算完成，直接返回结果
        populateActionResult(result, route, path, findPlanningDuration(start_time));
        action_server->succeeded_current(result);
        return;
      }
    }
  } catch (nav2_core::NoValidRouteCouldBeFound & ex) {
    // 找不到可行路线（起终点不连通等）
    exceptionWarning(goal, ex);
    action_server->terminate_current(result);
  } catch (nav2_core::TimedOut & ex) {
    // 图搜索超过最大迭代次数
    exceptionWarning(goal, ex);
    action_server->terminate_current(result);
  } catch (nav2_core::RouteTFError & ex) {
    // TF 坐标变换错误
    exceptionWarning(goal, ex);
    action_server->terminate_current(result);
  } catch (nav2_core::NoValidGraph & ex) {
    // 路图无效（为空/边代价非法）
    exceptionWarning(goal, ex);
    action_server->terminate_current(result);
  } catch (nav2_core::IndeterminantNodesOnGraph & ex) {
    // 无法确定最近的图节点（如距所有节点过远）
    exceptionWarning(goal, ex);
    action_server->terminate_current(result);
  } catch (nav2_core::InvalidEdgeScorerUse & ex) {
    // 边评分插件使用不当
    exceptionWarning(goal, ex);
    action_server->terminate_current(result);
  } catch (nav2_core::OperationFailed & ex) {
    // A special case since Operation Failed is only in Compute & Track
    // actions, specifying it to allow otherwise fully shared code
    // 路线操作（route operation）执行失败（仅 Compute&Track 会出现）
    exceptionWarning(goal, ex);
    action_server->terminate_current(result);
  } catch (nav2_core::RouteException & ex) {
    // 其他路线异常基类
    exceptionWarning(goal, ex);
    action_server->terminate_current(result);
  } catch (std::exception & ex) {
    // 兜底：其他标准异常
    exceptionWarning(goal, ex);
    action_server->terminate_current(result);
  }
}

// compute_route 动作回调：一次性路线计算
void
RouteServer::computeRoute()
{
  RCLCPP_INFO(get_logger(), "Computing route to goal.");
  processRouteRequest<ComputeRoute>(compute_route_server_);
}

// compute_and_track_route 动作回调：计算 + 跟踪执行
void
RouteServer::computeAndTrackRoute()
{
  RCLCPP_INFO(get_logger(), "Computing and tracking route to goal.");
  processRouteRequest<ComputeAndTrackRoute>(compute_and_track_route_server_);
}

// ---------------------------------------------------------------------------
// set_route_graph 服务回调：运行时从指定文件重新加载路图。
// 成功后同步更新 GoalIntentExtractor 的图引用，并重新发布可视化。
// ---------------------------------------------------------------------------
void RouteServer::setRouteGraph(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Request> request,
  std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Response> response)
{
  RCLCPP_INFO(get_logger(), "Setting new route graph: %s.", request->graph_filepath.c_str());
  // 清空旧图
  graph_.clear();
  id_to_graph_map_.clear();
  try {
    // 从文件加载新图 + TF 变换到 route_frame
    if (graph_loader_->loadGraphFromFile(graph_, id_to_graph_map_, request->graph_filepath)) {
      // 更新目标意图提取器持有的图引用
      goal_intent_extractor_->setGraph(graph_, &id_to_graph_map_);
      // 重新发布路图可视化
      graph_vis_publisher_->publish(utils::toMsg(graph_, route_frame_, this->now()));
      response->success = true;
      return;
    }
  } catch (std::exception & ex) {
  }

  // 加载失败：告警并返回失败
  RCLCPP_WARN(
    get_logger(),
    "Failed to set new route graph: %s!", request->graph_filepath.c_str());
  response->success = false;
}

// 模板工具：统一格式的异常告警日志（打印起终点位姿/ID 与异常信息）
template<typename GoalT>
void RouteServer::exceptionWarning(
  const std::shared_ptr<const GoalT> goal,
  const std::exception & ex)
{
  RCLCPP_WARN(
    get_logger(),
    "Route server failed on request: Start: [(%0.2f, %0.2f) / %i] Goal: [(%0.2f, %0.2f) / %i]:"
    " \"%s\"", goal->start.pose.position.x, goal->start.pose.position.y, goal->start_id,
    goal->goal.pose.position.x, goal->goal.pose.position.y, goal->goal_id, ex.what());
}

// Explicit template instantiations
// 模板显式实例化：为两种动作类型生成具体代码（分离编译需要）
template bool RouteServer::isRequestValid<RouteServer::ComputeRoute>(
  std::shared_ptr<nav2_util::SimpleActionServer<RouteServer::ComputeRoute>> &);
template bool RouteServer::isRequestValid<RouteServer::ComputeAndTrackRoute>(
  std::shared_ptr<nav2_util::SimpleActionServer<RouteServer::ComputeAndTrackRoute>> &);

template Route RouteServer::findRoute<RouteServer::ComputeRouteGoal>(
  const std::shared_ptr<const RouteServer::ComputeRouteGoal>,
  ReroutingState &);
template Route RouteServer::findRoute<RouteServer::ComputeAndTrackRouteGoal>(
  const std::shared_ptr<const RouteServer::ComputeAndTrackRouteGoal>,
  ReroutingState &);

template void RouteServer::processRouteRequest<RouteServer::ComputeRoute>(
  std::shared_ptr<nav2_util::SimpleActionServer<RouteServer::ComputeRoute>> &);
template void RouteServer::processRouteRequest<RouteServer::ComputeAndTrackRoute>(
  std::shared_ptr<nav2_util::SimpleActionServer<RouteServer::ComputeAndTrackRoute>> &);

template void RouteServer::exceptionWarning<RouteServer::ComputeRouteGoal>(
  const std::shared_ptr<const RouteServer::ComputeRouteGoal>,
  const std::exception &);
template void RouteServer::exceptionWarning<RouteServer::ComputeAndTrackRouteGoal>(
  const std::shared_ptr<const RouteServer::ComputeAndTrackRouteGoal>,
  const std::exception &);

}  // namespace nav2_route

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
// 注册组件节点宏：可作为 rclcpp_components 组件动态加载运行
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_route::RouteServer)
