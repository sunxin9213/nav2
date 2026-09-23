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
// RoutePlanner（路线规划器）的实现 —— 在路图（Graph）上做最短路径搜索。
// 模块定位：nav2_route 的图搜索核心。算法为 Dijkstra 最短路径
// （基于优先队列的统一代价搜索），代价来自：
//   - 边自带的 edge_cost.cost（不可覆盖时直接使用）；
//   - 或 EdgeScorer 管理的边代价评分插件（距离、代价地图、语义等），
//     仅当 edge_cost.overridable 为真且有插件加载时使用。
// 支持被阻塞的边/节点列表（blocked_ids，重路由时排除阻塞路段）。
// 搜索结束后通过每个节点的 search_state.parent_edge 回溯得到
// 起点->终点的边序列，构成 Route。
// ============================================================================

#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <mutex>
#include <algorithm>

#include "nav2_route/route_planner.hpp"

namespace nav2_route
{

// ---------------------------------------------------------------------------
// 配置：读取最大迭代次数参数（<=0 表示不限制，取 int 最大值），
// 并创建边评分器 EdgeScorer（管理各 edge_cost_function 插件）。
// ---------------------------------------------------------------------------
void RoutePlanner::configure(
  nav2_util::LifecycleNode::SharedPtr node,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber)
{
  nav2_util::declare_parameter_if_not_declared(
    node, "max_iterations", rclcpp::ParameterValue(0));
  max_iterations_ = node->get_parameter("max_iterations").as_int();

  if (max_iterations_ <= 0) {
    max_iterations_ = std::numeric_limits<int>::max();
  }

  edge_scorer_ = std::make_unique<EdgeScorer>(node, tf_buffer, costmap_subscriber);
}

// ---------------------------------------------------------------------------
// 路线求解入口：在 graph 上搜索 start_index -> goal_index 的最短路线。
// 流程：图搜索 -> 检查终点是否可达 -> 沿 parent_edge 回溯边序列 ->
// 反转得到正向路线（附带总代价 route_cost）。
// ---------------------------------------------------------------------------
Route RoutePlanner::findRoute(
  Graph & graph, unsigned int start_index, unsigned int goal_index,
  const std::vector<unsigned int> & blocked_ids,
  const RouteRequest & route_request)
{
  // 空图直接抛异常
  if (graph.empty()) {
    throw nav2_core::NoValidGraph("Graph is invalid for routing!");
  }

  // Find the start and goal pointers, it is important in this function
  // that the start node is the underlying pointer, so that the address
  // is valid when this function goes out of scope
  // 取起终点节点指针：必须指向 graph 内部的底层节点，
  // 保证函数返回后指针仍有效
  const NodePtr & start_node = &graph.at(start_index);
  const NodePtr & goal_node = &graph.at(goal_index);
  findShortestGraphTraversal(graph, start_node, goal_node, blocked_ids, route_request);

  // 终点无 parent_edge 说明搜索未能到达终点（不连通/全部被阻塞）
  EdgePtr & parent_edge = goal_node->search_state.parent_edge;
  if (!parent_edge) {
    throw nav2_core::NoValidRouteCouldBeFound("Could not find a route to the requested goal!");
  }

  // Convert graph traversal into a meaningful route
  // 从终点沿 parent_edge 逐级回溯到起点，收集边序列
  Route route;
  while (parent_edge) {
    route.edges.push_back(parent_edge);
    parent_edge = parent_edge->start->search_state.parent_edge;
  }

  // 回溯得到的是终点->起点顺序，反转为起点->终点
  std::reverse(route.edges.begin(), route.edges.end());
  route.start_node = start_node;
  route.route_cost = goal_node->search_state.integrated_cost;  // 总代价
  return route;
}

// ---------------------------------------------------------------------------
// 重置所有节点的搜索状态（代价、父边等）。
// 对于 < 75000 节点的图，直接遍历重置与重新分配内存的开销相当。
// ---------------------------------------------------------------------------
void RoutePlanner::resetSearchStates(Graph & graph)
{
  // For graphs < 75,000 nodes, iterating through one time on initialization to reset the state
  // is neglibably different to allocating & deallocating the complimentary blocks of memory
  for (unsigned int i = 0; i != graph.size(); i++) {
    graph[i].search_state.reset();
  }
}

// ---------------------------------------------------------------------------
// Dijkstra 最短路径搜索主循环：
//   1. 重置搜索状态，起点代价置 0 并入队；
//   2. 循环取出堆顶（当前最低累计代价）节点：
//      - 若其记录代价与堆中代价不一致，说明是过期条目，跳过（懒删除）；
//      - 若是目标节点，搜索成功，清空队列返回；
//      - 否则扩展所有邻接边：
//        * getTraversalCost 计算边代价（阻塞边返回不可通行）；
//        * 若经当前节点到邻居的累计代价更低，则松弛：
//          更新邻居的 parent_edge 与代价并入队。
//   3. 迭代数超过 max_iterations_ 时抛出超时异常。
// ---------------------------------------------------------------------------
void RoutePlanner::findShortestGraphTraversal(
  Graph & graph, const NodePtr start_node, const NodePtr goal_node,
  const std::vector<unsigned int> & blocked_ids,
  const RouteRequest & route_request)
{
  // Setup the Dijkstra's search
  // 初始化：重置状态，记录起终点 ID，起点代价 0 入队
  resetSearchStates(graph);
  start_id_ = start_node->nodeid;
  goal_id_ = goal_node->nodeid;
  start_node->search_state.integrated_cost = 0.0;
  addNode(0.0, start_node);

  NodePtr neighbor{nullptr};
  EdgePtr edge{nullptr};
  float potential_cost = 0.0, traversal_cost = 0.0;
  int iterations = 0;
  while (!queue_.empty() && iterations < max_iterations_) {
    iterations++;

    // Get the next lowest cost node
    // 取出堆顶（最低累计代价）节点
    auto [curr_cost, node] = getNextNode();

    // This has been visited, thus already lowest cost
    // 懒删除：堆中代价与节点当前代价不一致 => 过期条目，跳过
    if (curr_cost != node->search_state.integrated_cost) {
      continue;
    }

    // We have the shortest path
    // 到达目标：Dijkstra 保证此时即最短路径，提前结束
    if (isGoal(node)) {
      // Reset states
      clearQueue();
      return;
    }

    // Expand to connected nodes
    // 扩展当前节点的所有邻接边
    EdgeVector & edges = getEdges(node);
    for (unsigned int edge_num = 0; edge_num != edges.size(); edge_num++) {
      edge = &edges[edge_num];
      neighbor = edge->end;

      // If edge is invalid (lane closed, occupied, etc), don't expand
      // 边不可通行（被阻塞或评分失败）：跳过
      if (!getTraversalCost(edge, traversal_cost, blocked_ids, route_request)) {
        continue;
      }

      // 松弛操作：经当前节点到邻居的候选代价更低则更新并 入队
      potential_cost = curr_cost + traversal_cost;
      if (potential_cost < neighbor->search_state.integrated_cost) {
        neighbor->search_state.parent_edge = edge;   // 记录父边（回溯路线用）
        neighbor->search_state.integrated_cost = potential_cost;
        neighbor->search_state.traversal_cost = traversal_cost;
        addNode(potential_cost, neighbor);
      }
    }
  }

  if (iterations == max_iterations_) {
    // Reset states
    clearQueue();
    // 超过迭代上限：抛出超时异常
    throw nav2_core::TimedOut("Maximum iterations was exceeded!");
  }
}

// ---------------------------------------------------------------------------
// 计算边的通行代价：
//   1. 边或其终点节点在 blocked_ids 中 => 返回 false（禁止通行）；
//   2. 边代价不可覆盖（overridable=false）或无评分插件时，
//      直接使用边自带代价（必须为正，否则抛图非法异常）；
//   3. 否则调用 EdgeScorer 插件体系评分（可综合距离、代价地图、
//      语义偏好等），并传入边的分类（起点边/终点边/普通边）。
// ---------------------------------------------------------------------------
bool RoutePlanner::getTraversalCost(
  const EdgePtr edge, float & score, const std::vector<unsigned int> & blocked_ids,
  const RouteRequest & route_request)
{
  // If edge or node is in the blocked list, don't expand
  // 检查边 ID 或边终点节点 ID 是否在阻塞列表中
  auto is_blocked = std::find_if(
    blocked_ids.begin(), blocked_ids.end(),
    [&](unsigned int id) {return id == edge->edgeid || id == edge->end->nodeid;});
  if (is_blocked != blocked_ids.end()) {
    return false;
  }

  // If an edge's cost is marked as not to be overridden by scoring plugins
  // Or there are no scoring plugins, use the edge's cost, if it is valid (positive)
  // 不可覆盖或无插件：使用边自带代价（非正则视为图非法）
  if (!edge->edge_cost.overridable || edge_scorer_->numPlugins() == 0) {
    if (edge->edge_cost.cost <= 0.0) {
      throw nav2_core::NoValidGraph(
              "Edge " + std::to_string(edge->edgeid) +
              " doesn't contain and cannot compute a valid edge cost!");
    }
    score = edge->edge_cost.cost;
    return true;
  }

  // 插件评分：classifyEdge 标记该边是否为起点/终点特殊边，
  // 供评分插件（如朝向评分）做特殊处理
  return edge_scorer_->score(edge, route_request, classifyEdge(edge), score);
}

// 弹出堆顶元素（最低代价节点）
NodeElement RoutePlanner::getNextNode()
{
  NodeElement data = queue_.top();
  queue_.pop();
  return data;
}

// 以给定累计代价将节点入队
void RoutePlanner::addNode(const float cost, const NodePtr node)
{
  queue_.emplace(cost, node);
}

// 取节点的邻接边集合
EdgeVector & RoutePlanner::getEdges(const NodePtr node)
{
  return node->neighbors;
}

// 清空优先队列（swap 技巧快速释放内存）
void RoutePlanner::clearQueue()
{
  NodeQueue q;
  std::swap(queue_, q);
}

// 判断节点是否为目标节点
bool RoutePlanner::isGoal(const NodePtr node)
{
  return node->nodeid == goal_id_;
}

// 判断节点是否为起点节点
bool RoutePlanner::isStart(const NodePtr node)
{
  return node->nodeid == start_id_;
}

// ---------------------------------------------------------------------------
// 边分类：起点边（边的起点是搜索起点）、终点边（边的终点是搜索目标）、
// 普通边。供边评分插件识别特殊位置的边。
// ---------------------------------------------------------------------------
nav2_route::EdgeType RoutePlanner::classifyEdge(const EdgePtr edge)
{
  if (isStart(edge->start)) {
    return EdgeType::START;
  } else if (isGoal(edge->end)) {
    return EdgeType::END;
  }
  return nav2_route::EdgeType::NONE;
}

}  // namespace nav2_route
