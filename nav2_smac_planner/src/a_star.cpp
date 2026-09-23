// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2020, Applied Electric Vehicles Pty Ltd
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

#include <omp.h>
#include <cmath>
#include <stdexcept>
#include <memory>
#include <algorithm>
#include <limits>
#include <type_traits>
#include <chrono>
#include <thread>
#include <utility>
#include <vector>

#include "nav2_smac_planner/a_star.hpp"
using namespace std::chrono;  // NOLINT

namespace nav2_smac_planner
{

// 【中文注释】本文件实现 SMAC A* 搜索算法的核心流程（模板类，按节点类型实例化）：
//   - Node2D：2D 网格 A*（SmacPlanner2D 使用），4/8 邻域扩展；
//   - NodeHybrid：混合 A* 状态格节点（SmacPlannerHybrid 使用），SE(2) 位姿节点，
//     带最小转弯半径约束，支持 Dubins/Reeds-Shepp 解析扩展；
//   - NodeLattice：预计算控制采样轨迹表的状态格节点（SmacPlannerLattice 使用）。
// 标准 A* 流程：open list（二叉堆优先队列）+ closed set（visited 标记）+
//   f(n) = g(n) + h(n) 弹出最优节点 -> 邻域扩展 -> 回溯父指针得到路径。
// 相比经典 A* 的增强：tryAnalyticExpansion 尝试用解析曲线（直线/Dubins/RS）
//   直接连到目标，一旦成功立即结束搜索，大幅减少扩展节点数。

template<typename NodeT>
AStarAlgorithm<NodeT>::AStarAlgorithm(
  const MotionModel & motion_model,
  const SearchInfo & search_info)
: _traverse_unknown(true),
  _is_initialized(false),
  _max_iterations(0),
  _max_planning_time(0),
  _x_size(0),
  _y_size(0),
  _search_info(search_info),
  _goal_coordinates(Coordinates()),
  _start(nullptr),
  _goal(nullptr),
  _motion_model(motion_model)
{
  _graph.reserve(100000);
}

template<typename NodeT>
AStarAlgorithm<NodeT>::~AStarAlgorithm()
{
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::initialize(
  const bool & allow_unknown,
  int & max_iterations,
  const int & max_on_approach_iterations,
  const double & max_planning_time,
  const float & lookup_table_size,
  const unsigned int & dim_3_size)
{
  // 【中文注释】通用初始化（Hybrid/Lattice 版本）：
  // 记录是否允许穿越未知区、最大迭代数、接近目标的最大迭代数、最大规划时长；
  // 首次调用时预计算距离启发式查找表（Dubins/RS 距离表，详见 node_hybrid/lattice）；
  // 创建解析扩展器 AnalyticExpansion（用于尝试直线/Dubins/RS 曲线直达目标）。
  _traverse_unknown = allow_unknown;
  _max_iterations = max_iterations;
  _max_on_approach_iterations = max_on_approach_iterations;
  _max_planning_time = max_planning_time;
  if (!_is_initialized) {
    NodeT::precomputeDistanceHeuristic(lookup_table_size, _motion_model, dim_3_size, _search_info);
  }
  _is_initialized = true;
  _dim3_size = dim_3_size;
  _expander = std::make_unique<AnalyticExpansion<NodeT>>(
    _motion_model, _search_info, _traverse_unknown, _dim3_size);
}

template<>
void AStarAlgorithm<Node2D>::initialize(
  const bool & allow_unknown,
  int & max_iterations,
  const int & max_on_approach_iterations,
  const double & max_planning_time,
  const float & /*lookup_table_size*/,
  const unsigned int & dim_3_size)
{
  // 【中文注释】Node2D 特化初始化：2D 节点没有第三维（角度），
  // 因此 dim_3_size 必须为 1，也不需要预计算距离启发式表。
  _traverse_unknown = allow_unknown;
  _max_iterations = max_iterations;
  _max_on_approach_iterations = max_on_approach_iterations;
  _max_planning_time = max_planning_time;

  if (dim_3_size != 1) {
    throw std::runtime_error("Node type Node2D cannot be given non-1 dim 3 quantization.");
  }
  _dim3_size = dim_3_size;
  _expander = std::make_unique<AnalyticExpansion<Node2D>>(
    _motion_model, _search_info, _traverse_unknown, _dim3_size);
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::setCollisionChecker(GridCollisionChecker * collision_checker)
{
  // 【中文注释】注入碰撞检查器（含 costmap）：读取地图尺寸，
  // 清空上一次搜索的图，若地图尺寸变化则重新初始化节点的运动模型
  // （邻域偏移表/运动基元），并让解析扩展器也持有碰撞检查器。
  _collision_checker = collision_checker;
  _costmap = collision_checker->getCostmap();
  unsigned int x_size = _costmap->getSizeInCellsX();
  unsigned int y_size = _costmap->getSizeInCellsY();

  clearGraph();

  if (getSizeX() != x_size || getSizeY() != y_size) {
    _x_size = x_size;
    _y_size = y_size;
    NodeT::initMotionModel(_motion_model, _x_size, _y_size, _dim3_size, _search_info);
  }
  _expander->setCollisionChecker(collision_checker);
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr AStarAlgorithm<NodeT>::addToGraph(
  const unsigned int & index)
{
  // 【中文注释】惰性建图：_graph 是"索引 -> 节点对象"的哈希表，
  // 只有被访问到的状态才会真正创建节点（而不是预分配整个状态空间），
  // 这是 SMAC 能在大型 Hybrid/Lattice 状态空间上高效搜索的关键。
  auto iter = _graph.find(index);
  if (iter != _graph.end()) {
    return &(iter->second);
  }

  return &(_graph.emplace(index, NodeT(index)).first->second);
}

template<>
void AStarAlgorithm<Node2D>::setStart(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & dim_3)
{
  if (dim_3 != 0) {
    throw std::runtime_error("Node type Node2D cannot be given non-zero starting dim 3.");
  }
  _start = addToGraph(Node2D::getIndex(mx, my, getSizeX()));
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::setStart(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & dim_3)
{
  // 【中文注释】设置起点（通用版）：(mx,my) 为栅格坐标，dim_3 为第三维索引
  // （Hybrid/Lattice 下是角度量化 bin），由三者的组合索引建节点并记录 SE(2) 位姿。
  _start = addToGraph(NodeT::getIndex(mx, my, dim_3));
  _start->setPose(
    Coordinates(
      static_cast<float>(mx),
      static_cast<float>(my),
      static_cast<float>(dim_3)));
}

template<>
void AStarAlgorithm<Node2D>::setGoal(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & dim_3)
{
  if (dim_3 != 0) {
    throw std::runtime_error("Node type Node2D cannot be given non-zero goal dim 3.");
  }

  _goal = addToGraph(Node2D::getIndex(mx, my, getSizeX()));
  _goal_coordinates = Node2D::Coordinates(mx, my);
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::setGoal(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & dim_3)
{
  // 【中文注释】设置目标点（通用版）：建目标节点并记录位姿。
  // 若未启用障碍启发缓存、或目标坐标发生变化，则重算障碍启发
  // （2D Dijkstra 距离场，见各节点类实现），要求先设置起点。
  _goal = addToGraph(NodeT::getIndex(mx, my, dim_3));

  typename NodeT::Coordinates goal_coords(
    static_cast<float>(mx),
    static_cast<float>(my),
    static_cast<float>(dim_3));

  if (!_search_info.cache_obstacle_heuristic || goal_coords != _goal_coordinates) {
    if (!_start) {
      throw std::runtime_error("Start must be set before goal.");
    }

    NodeT::resetObstacleHeuristic(_costmap, _start->pose.x, _start->pose.y, mx, my);
  }

  _goal_coordinates = goal_coords;
  _goal->setPose(_goal_coordinates);
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::areInputsValid()
{
  // 【中文注释】输入合法性检查：图非空、起终点已设置、
  // 终点在无容差时必须可通行（被障碍占据且 tolerance<0.001 则报错）、
  // 起点必须不在致命障碍内。
  // Check if graph was filled in
  if (_graph.empty()) {
    throw std::runtime_error("Failed to compute path, no costmap given.");
  }

  // Check if points were filled in
  if (!_start || !_goal) {
    throw std::runtime_error("Failed to compute path, no valid start or goal given.");
  }

  // Check if ending point is valid
  if (getToleranceHeuristic() < 0.001 &&
    !_goal->isNodeValid(_traverse_unknown, _collision_checker))
  {
    throw std::runtime_error("Failed to compute path, goal is occupied with no tolerance.");
  }

  // Check if starting point is valid
  if (!_start->isNodeValid(_traverse_unknown, _collision_checker)) {
    throw std::runtime_error("Starting point in lethal space! Cannot create feasible plan.");
  }

  return true;
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::createPath(
  CoordinateVector & path, int & iterations,
  const float & tolerance)
{
  // 【中文注释】A* 主搜索循环（SMAC 核心）。
  // path 为输出路径，iterations 返回实际扩展的节点数，tolerance 为目标容差。
  steady_clock::time_point start_time = steady_clock::now();
  _tolerance = tolerance;
  // 【中文注释】_best_heuristic_node 记录历史中"启发值最小(离目标最近)"的
  // {启发值, 节点索引}，用于容差内提前结束或搜索失败时返回最近路径。
  _best_heuristic_node = {std::numeric_limits<float>::max(), 0};
  clearQueue();

  if (!areInputsValid()) {
    return false;
  }

  // 0) Add starting point to the open set
  // 【中文注释】第 0 步：起点入 open set（代价 0），g 值清零
  addNode(0.0, getStart());
  getStart()->setAccumulatedCost(0.0);

  // Optimization: preallocate all variables
  // 【中文注释】循环外预分配所有变量，避免热路径上的临时对象构造开销
  NodePtr current_node = nullptr;
  NodePtr neighbor = nullptr;
  NodePtr expansion_result = nullptr;
  float g_cost = 0.0;
  NodeVector neighbors;
  int approach_iterations = 0;
  NeighborIterator neighbor_iterator;
  int analytic_iterations = 0;
  int closest_distance = std::numeric_limits<int>::max();

  // Given an index, return a node ptr reference if its collision-free and valid
  // 【中文注释】邻居获取器闭包：给定状态索引，返回图中对应节点指针；
  // 越界索引返回 false。供节点类的 getNeighbors 与解析扩展共用。
  const unsigned int max_index = getSizeX() * getSizeY() * getSizeDim3();
  NodeGetter neighborGetter =
    [&, this](const unsigned int & index, NodePtr & neighbor_rtn) -> bool
    {
      if (index >= max_index) {
        return false;
      }

      neighbor_rtn = addToGraph(index);
      return true;
    };

  // 【中文注释】主循环条件：未超最大迭代数 且 open list 非空
  while (iterations < getMaxIterations() && !_queue.empty()) {
    // Check for planning timeout only on every Nth iteration
    // 【中文注释】每 _timing_interval 次迭代检查一次超时，避免频繁取时钟
    if (iterations % _timing_interval == 0) {
      std::chrono::duration<double> planning_duration =
        std::chrono::duration_cast<std::chrono::duration<double>>(steady_clock::now() - start_time);
      if (static_cast<double>(planning_duration.count()) >= _max_planning_time) {
        return false;
      }
    }

    // 1) Pick Nbest from O s.t. min(f(Nbest)), remove from queue
    // 【中文注释】第 1 步：从 open list（最小堆）弹出 f 值最小的节点
    current_node = getNextNode();

    // We allow for nodes to be queued multiple times in case
    // shorter paths result in it, but we can visit only once
    // 【中文注释】惰性删除策略：同一节点可能以不同 f 值多次入队，
    // 弹出时若已被访问过（相当于已在 closed set）直接跳过
    if (current_node->wasVisited()) {
      continue;
    }

    iterations++;

    // 2) Mark Nbest as visited
    // 【中文注释】第 2 步：加入 closed set（标记 visited）
    current_node->visited();

    // 2.1) Use an analytic expansion (if available) to generate a path
    // 【中文注释】第 2.1 步：解析扩展（SMAC 特色）——尝试用解析曲线
    // （2D:直线；Hybrid:Dubins/Reeds-Shepp；Lattice:直线）从当前节点
    // 直接连到目标。若成功返回目标节点，搜索立即收敛。
    expansion_result = nullptr;
    expansion_result = _expander->tryAnalyticExpansion(
      current_node, getGoal(), neighborGetter, analytic_iterations, closest_distance);
    if (expansion_result != nullptr) {
      current_node = expansion_result;
    }

    // 3) Check if we're at the goal, backtrace if required
    // 【中文注释】第 3 步：到达目标 -> 沿 parent 指针回溯出完整路径返回
    if (isGoal(current_node)) {
      return current_node->backtracePath(path);
    } else if (_best_heuristic_node.first < getToleranceHeuristic()) {
      // Optimization: Let us find when in tolerance and refine within reason
      // 【中文注释】容差优化：当前最优节点已进入目标容差范围时，
      // 允许再迭代 on_approach_max_iterations 次以细化路径，
      // 超过后直接从"历史离目标最近"的节点回溯返回
      approach_iterations++;
      if (approach_iterations >= getOnApproachMaxIterations()) {
        return _graph.at(_best_heuristic_node.second).backtracePath(path);
      }
    }

    // 4) Expand neighbors of Nbest not visited
    // 【中文注释】第 4 步：邻域扩展。Node2D 为 4/8 邻域；
    // NodeHybrid/NodeLattice 为运动基元（满足最小转弯半径的圆弧+直线段）。
    neighbors.clear();
    current_node->getNeighbors(neighborGetter, _collision_checker, _traverse_unknown, neighbors);

    for (neighbor_iterator = neighbors.begin();
      neighbor_iterator != neighbors.end(); ++neighbor_iterator)
    {
      neighbor = *neighbor_iterator;

      // 4.1) Compute the cost to go to this node
      // 【中文注释】候选 g 值 = 父节点累积代价 g(current) + 边的穿越代价
      // （穿越代价包含运动长度、转向惩罚、代价地图代价值等，见各节点类实现）
      g_cost = current_node->getAccumulatedCost() + current_node->getTraversalCost(neighbor);

      // 4.2) If this is a lower cost than prior, we set this as the new cost and new approach
      // 【中文注释】松弛操作：若新 g 值更小则更新 g 值并重设父节点
      if (g_cost < neighbor->getAccumulatedCost()) {
        neighbor->setAccumulatedCost(g_cost);
        neighbor->parent = current_node;

        // 4.3) Add to queue with heuristic cost
        // 【中文注释】以 f = g + h 入队（h 为启发函数，含 tie-breaker 小量）
        addNode(g_cost + getHeuristicCost(neighbor), neighbor);
      }
    }
  }

  if (_best_heuristic_node.first < getToleranceHeuristic()) {
    // If we run out of serach options, return the path that is closest, if within tolerance.
    // 【中文注释】搜索耗尽但历史最优节点已在容差内：返回到该"最接近节点"的路径
    return _graph.at(_best_heuristic_node.second).backtracePath(path);
  }

  // 【中文注释】彻底失败：迭代耗尽/队列为空且未接近目标
  return false;
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::isGoal(NodePtr & node)
{
  return node == getGoal();
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr & AStarAlgorithm<NodeT>::getStart()
{
  return _start;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr & AStarAlgorithm<NodeT>::getGoal()
{
  return _goal;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr AStarAlgorithm<NodeT>::getNextNode()
{
  // 【中文注释】弹出堆顶（f 最小）节点：堆里存的是轻量 NodeBasic 句柄，
  // processSearchNode() 将其还原为图中真实节点的指针再返回。
  NodeBasic<NodeT> node = _queue.top().second;
  _queue.pop();
  node.processSearchNode();
  return node.graph_node_ptr;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::addNode(const float & cost, NodePtr & node)
{
  // 【中文注释】以给定 f 值(cost)把节点句柄压入 open list 最小堆
  NodeBasic<NodeT> queued_node(node->getIndex());
  queued_node.populateSearchNode(node);
  _queue.emplace(cost, queued_node);
}

template<typename NodeT>
float AStarAlgorithm<NodeT>::getHeuristicCost(const NodePtr & node)
{
  // 【中文注释】计算节点到目标的启发值 h(n)（各节点类型各自实现：
  // Node2D 用欧氏距离；NodeHybrid 用"障碍启发+距离启发"的较大者），
  // 同时顺带更新"历史离目标最近"的节点记录（用于容差提前结束）。
  const Coordinates node_coords =
    NodeT::getCoords(node->getIndex(), getSizeX(), getSizeDim3());
  float heuristic = NodeT::getHeuristicCost(
    node_coords, _goal_coordinates, _costmap);

  if (heuristic < _best_heuristic_node.first) {
    _best_heuristic_node = {heuristic, node->getIndex()};
  }

  return heuristic;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::clearQueue()
{
  // 【中文注释】swap 技巧快速清空优先队列（无 clear 接口）
  NodeQueue q;
  std::swap(_queue, q);
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::clearGraph()
{
  // 【中文注释】清空搜索图并重新预留容量，为下一次规划做准备
  Graph g;
  std::swap(_graph, g);
  _graph.reserve(100000);
}

template<typename NodeT>
int & AStarAlgorithm<NodeT>::getMaxIterations()
{
  return _max_iterations;
}

template<typename NodeT>
int & AStarAlgorithm<NodeT>::getOnApproachMaxIterations()
{
  return _max_on_approach_iterations;
}

template<typename NodeT>
float & AStarAlgorithm<NodeT>::getToleranceHeuristic()
{
  return _tolerance;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeX()
{
  return _x_size;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeY()
{
  return _y_size;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeDim3()
{
  return _dim3_size;
}

// Instantiate algorithm for the supported template types
// 【中文注释】显式实例化：A* 模板对三种节点类型（2D/混合/Lattice）各生成一份代码
template class AStarAlgorithm<Node2D>;
template class AStarAlgorithm<NodeHybrid>;
template class AStarAlgorithm<NodeLattice>;

}  // namespace nav2_smac_planner
