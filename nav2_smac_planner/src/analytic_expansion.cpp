// Copyright (c) 2021, Samsung Research America
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

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <algorithm>
#include <vector>
#include <memory>

#include "nav2_smac_planner/analytic_expansion.hpp"

namespace nav2_smac_planner
{

// 【中文注释】解析扩展（Analytic Expansion）实现：
// SMAC 在标准 A* 图搜索之外的"抄近道"机制——定期尝试用解析可算的曲线
// 从当前扩展节点直接连到目标：
//   - Dubins 模型：前进-only 的最短曲线（圆弧-直线-圆弧，CSC 等类型），经 OMPL 计算；
//   - Reeds-Shepp 模型：允许倒车的最短曲线（含倒车段），同样经 OMPL 计算；
//   - State Lattice：同上（曲线需与预计算轨迹表的角度 bin 对齐）；
//   - Node2D：不支持（特化为空实现，2D 搜索靠邻域扩展即可）。
// 连接成功的判定：把曲线按 sqrt(2) 栅格间距离散采样，逐点做碰撞检查，
// 全部通过才接受。成功后直接把曲线节点串成链并接到目标，立即结束搜索。
// 触发频率由 analytic_expansion_ratio 控制（离目标越近尝试越频繁）。

template<typename NodeT>
AnalyticExpansion<NodeT>::AnalyticExpansion(
  const MotionModel & motion_model,
  const SearchInfo & search_info,
  const bool & traverse_unknown,
  const unsigned int & dim_3_size)
: _motion_model(motion_model),
  _search_info(search_info),
  _traverse_unknown(traverse_unknown),
  _dim_3_size(dim_3_size),
  _collision_checker(nullptr)
{
}

template<typename NodeT>
void AnalyticExpansion<NodeT>::setCollisionChecker(
  GridCollisionChecker * collision_checker)
{
  _collision_checker = collision_checker;
}

template<typename NodeT>
typename AnalyticExpansion<NodeT>::NodePtr AnalyticExpansion<NodeT>::tryAnalyticExpansion(
  const NodePtr & current_node, const NodePtr & goal_node,
  const NodeGetter & getter, int & analytic_iterations,
  int & closest_distance)
{
  // This must be a valid motion model for analytic expansion to be attempted
  // 【中文注释】只有曲线运动模型（Dubin / Reeds-Shepp / 状态格）才支持解析扩展，
  // 2D 网格模型走特化的空实现。
  if (_motion_model == MotionModel::DUBIN || _motion_model == MotionModel::REEDS_SHEPP ||
    _motion_model == MotionModel::STATE_LATTICE)
  {
    // See if we are closer and should be expanding more often
    // 【中文注释】动态调整尝试频率：更新历史"离目标最近距离"closest_distance
    // （用启发值近似），离目标越近，两次尝试之间的迭代间隔越短。
    auto costmap = _collision_checker->getCostmap();
    const Coordinates node_coords =
      NodeT::getCoords(current_node->getIndex(), costmap->getSizeInCellsX(), _dim_3_size);
    closest_distance = std::min(
      closest_distance,
      static_cast<int>(NodeT::getHeuristicCost(node_coords, goal_node->pose, costmap)));

    // We want to expand at a rate of d/expansion_ratio,
    // but check to see if we are so close that we would be expanding every iteration
    // If so, limit it to the expansion ratio (rounded up)
    // 【中文注释】目标尝试间隔 = 最近距离 / analytic_expansion_ratio，
    // 并限制最小不小于 ratio 本身（避免贴着目标时每次迭代都做昂贵的曲线检查）。
    int desired_iterations = std::max(
      static_cast<int>(closest_distance / _search_info.analytic_expansion_ratio),
      static_cast<int>(std::ceil(_search_info.analytic_expansion_ratio)));

    // If we are closer now, we should update the target number of iterations to go
    // 【中文注释】若本次更接近目标，缩短剩余等待的迭代数
    analytic_iterations =
      std::min(analytic_iterations, desired_iterations);

    // Always run the expansion on the first run in case there is a
    // trivial path to be found
    // 【中文注释】计数归零时执行一次解析扩展尝试（首轮也会触发，
    // 以便起终点之间存在"平凡直达路径"时直接秒出结果）。
    if (analytic_iterations <= 0) {
      // Reset the counter and try the analytic path expansion
      analytic_iterations = desired_iterations;
      AnalyticExpansionNodes analytic_nodes = getAnalyticPath(current_node, goal_node, getter);
      if (!analytic_nodes.empty()) {
        // If we have a valid path, attempt to refine it
        // 【中文注释】成功后做"回溯精炼"：从当前节点沿父链每次回退 5 个节点，
        // 重新尝试解析曲线；若更早的祖先也能直连目标，就用更早的连接点
        // （曲线更长、路径整体更优），最多回退 8 次（约 40 个节点）。
        NodePtr node = current_node;
        NodePtr test_node = current_node;
        AnalyticExpansionNodes refined_analytic_nodes;
        for (int i = 0; i < 8; i++) {
          // Attempt to create better paths in 5 node increments, need to make sure
          // they exist for each in order to do so (maximum of 40 points back).
          if (test_node->parent && test_node->parent->parent && test_node->parent->parent->parent &&
            test_node->parent->parent->parent->parent &&
            test_node->parent->parent->parent->parent->parent)
          {
            test_node = test_node->parent->parent->parent->parent->parent;
            refined_analytic_nodes = getAnalyticPath(test_node, goal_node, getter);
            if (refined_analytic_nodes.empty()) {
              break;
            }
            analytic_nodes = refined_analytic_nodes;
            node = test_node;
          } else {
            break;
          }
        }

        // 【中文注释】把最终选定的曲线写入搜索图（设置父链）并返回目标节点
        return setAnalyticPath(node, goal_node, analytic_nodes);
      }
    }

    analytic_iterations--;
  }

  // No valid motion model - return nullptr
  return NodePtr(nullptr);
}

template<typename NodeT>
typename AnalyticExpansion<NodeT>::AnalyticExpansionNodes AnalyticExpansion<NodeT>::getAnalyticPath(
  const NodePtr & node,
  const NodePtr & goal,
  const NodeGetter & node_getter)
{
  // 【中文注释】生成从 node 到 goal 的解析曲线并离散校验：
  // 1) 把起终点 SE(2) 位姿填入 OMPL 状态空间（Dubins 或 RS 空间）；
  // 2) state_space->distance() 给出最短曲线长度 d；
  // 3) 距离过长则放弃（防止离目标很远时"抄近道"穿过高代价区，
  //    该阈值 analytic_expansion_max_length 不应小于 4~5 倍最小转弯半径，
  //    否则规划时间会暴涨）；
  // 4) 按每段 sqrt(2) 栅格的距离对曲线插值采样，逐点转换为状态格节点
  //    并做碰撞检查（isNodeValid），任何一点失败则整体失败。
  static ompl::base::ScopedState<> from(node->motion_table.state_space), to(
    node->motion_table.state_space), s(node->motion_table.state_space);
  from[0] = node->pose.x;
  from[1] = node->pose.y;
  from[2] = node->motion_table.getAngleFromBin(node->pose.theta);
  to[0] = goal->pose.x;
  to[1] = goal->pose.y;
  to[2] = node->motion_table.getAngleFromBin(goal->pose.theta);

  float d = node->motion_table.state_space->distance(from(), to());

  // If the length is too far, exit. This prevents unsafe shortcutting of paths
  // into higher cost areas far out from the goal itself, let search to the work of getting
  // close before the analytic expansion brings it home. This should never be smaller than
  // 4-5x the minimum turning radius being used, or planning times will begin to spike.
  if (d > _search_info.analytic_expansion_max_length) {
    return AnalyticExpansionNodes();
  }

  // A move of sqrt(2) is guaranteed to be in a new cell
  // 【中文注释】sqrt(2) 是保证每个采样点落入不同栅格的最大步长（对角线长度）
  static const float sqrt_2 = std::sqrt(2.);
  unsigned int num_intervals = std::floor(d / sqrt_2);

  AnalyticExpansionNodes possible_nodes;
  // When "from" and "to" are zero or one cell away,
  // num_intervals == 0
  possible_nodes.reserve(num_intervals);  // We won't store this node or the goal
  std::vector<double> reals;
  double theta;

  // Pre-allocate
  // 【中文注释】prev 记录曲线上的上一个节点，用于避免重复节点
  NodePtr prev(node);
  unsigned int index = 0;
  NodePtr next(nullptr);
  float angle = 0.0;
  Coordinates proposed_coordinates;
  bool failure = false;

  // Check intermediary poses (non-goal, non-start)
  // 【中文注释】沿曲线逐段插值：interpolate() 得到第 i 段的 (x,y,theta)，
  // 角度归一到 [0,2PI) 后量化到最近的角度 bin，再构造节点索引；
  // 节点必须通过碰撞检查且不与上一点重复，否则整体失败。
  for (float i = 1; i <= num_intervals; i++) {
    node->motion_table.state_space->interpolate(from(), to(), i / num_intervals, s());
    reals = s.reals();
    // Make sure in range [0, 2PI)
    theta = (reals[2] < 0.0) ? (reals[2] + 2.0 * M_PI) : reals[2];
    theta = (theta > 2.0 * M_PI) ? (theta - 2.0 * M_PI) : theta;
    angle = node->motion_table.getClosestAngularBin(theta);

    // Turn the pose into a node, and check if it is valid
    index = NodeT::getIndex(
      static_cast<unsigned int>(reals[0]),
      static_cast<unsigned int>(reals[1]),
      static_cast<unsigned int>(angle));
    // Get the node from the graph
    if (node_getter(index, next)) {
      // 【中文注释】临时把曲线位姿写进节点做碰撞检查，
      // 并保存原始位姿以便失败时回滚（不影响后续搜索）
      Coordinates initial_node_coords = next->pose;
      proposed_coordinates = {static_cast<float>(reals[0]), static_cast<float>(reals[1]), angle};
      next->setPose(proposed_coordinates);
      if (next->isNodeValid(_traverse_unknown, _collision_checker) && next != prev) {
        // Save the node, and its previous coordinates in case we need to abort
        possible_nodes.emplace_back(next, initial_node_coords, proposed_coordinates);
        prev = next;
      } else {
        // Abort
        next->setPose(initial_node_coords);
        failure = true;
        break;
      }
    } else {
      // Abort
      failure = true;
      break;
    }
  }

  // Reset to initial poses to not impact future searches
  // 【中文注释】无论成败，把所有被临时改写位姿的节点恢复原值
  for (const auto & node_pose : possible_nodes) {
    const auto & n = node_pose.node;
    n->setPose(node_pose.initial_coords);
  }

  if (failure) {
    return AnalyticExpansionNodes();
  }

  return possible_nodes;
}

template<typename NodeT>
typename AnalyticExpansion<NodeT>::NodePtr AnalyticExpansion<NodeT>::setAnalyticPath(
  const NodePtr & node,
  const NodePtr & goal_node,
  const AnalyticExpansionNodes & expanded_nodes)
{
  // 【中文注释】把校验通过的曲线节点正式写入搜索图：
  // 依次设置父指针形成 node -> ... -> goal 的链，写入曲线位姿并标记 visited。
  // 若某个中间节点已被搜索访问过（wasVisited），则复制出一个"脱节点"
  // （detached node，索引 -1，不进图）来承载曲线状态，避免破坏既有搜索状态。
  _detached_nodes.clear();
  // Legitimate final path - set the parent relationships, states, and poses
  NodePtr prev = node;
  for (const auto & node_pose : expanded_nodes) {
    auto n = node_pose.node;
    cleanNode(n);
    if (n->getIndex() != goal_node->getIndex()) {
      if (n->wasVisited()) {
        _detached_nodes.push_back(std::make_unique<NodeT>(-1));
        n = _detached_nodes.back().get();
      }
      n->parent = prev;
      n->pose = node_pose.proposed_coords;
      n->visited();
      prev = n;
    }
  }
  if (goal_node != prev) {
    // 【中文注释】最后把目标节点接到曲线末端，返回目标节点供 A* 主循环立即回溯
    goal_node->parent = prev;
    cleanNode(goal_node);
    goal_node->visited();
  }
  return goal_node;
}

template<>
void AnalyticExpansion<NodeLattice>::cleanNode(const NodePtr & node)
{
  // 【中文注释】Lattice 特化：曲线节点不来自预计算轨迹表，清除其运动基元指针，
  // 防止回溯路径时误用基元信息做轨迹重构
  node->setMotionPrimitive(nullptr);
}

template<typename NodeT>
void AnalyticExpansion<NodeT>::cleanNode(const NodePtr & /*expanded_nodes*/)
{
}

// 【中文注释】以下三个 Node2D 特化均为空实现：
// 2D 网格搜索没有连续角度/转弯半径概念，解析扩展不适用。
template<>
typename AnalyticExpansion<Node2D>::AnalyticExpansionNodes AnalyticExpansion<Node2D>::
getAnalyticPath(
  const NodePtr & node,
  const NodePtr & goal,
  const NodeGetter & node_getter)
{
  return AnalyticExpansionNodes();
}

template<>
typename AnalyticExpansion<Node2D>::NodePtr AnalyticExpansion<Node2D>::setAnalyticPath(
  const NodePtr & node,
  const NodePtr & goal_node,
  const AnalyticExpansionNodes & expanded_nodes)
{
  return NodePtr(nullptr);
}

template<>
typename AnalyticExpansion<Node2D>::NodePtr AnalyticExpansion<Node2D>::tryAnalyticExpansion(
  const NodePtr & current_node, const NodePtr & goal_node,
  const NodeGetter & getter, int & analytic_iterations,
  int & closest_distance)
{
  return NodePtr(nullptr);
}

template class AnalyticExpansion<Node2D>;
template class AnalyticExpansion<NodeHybrid>;
template class AnalyticExpansion<NodeLattice>;

}  // namespace nav2_smac_planner
