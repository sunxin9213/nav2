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

#include "nav2_smac_planner/node_2d.hpp"

#include <vector>
#include <limits>

namespace nav2_smac_planner
{

// defining static member for all instance to share
// 【中文注释】Node2D 静态成员：_neighbors_grid_offsets 为 8 邻域的一维索引偏移
// （左右/上下/四个对角，基于地图宽度 x_size）；cost_travel_multiplier 为
// 地图代价的权重乘子（cost_penalty），使路径偏向低代价区域。
std::vector<int> Node2D::_neighbors_grid_offsets;
float Node2D::cost_travel_multiplier = 2.0;

// 【中文注释】Node2D：2D 网格 A* 节点（SmacPlanner2D 用）。
// 状态只有栅格坐标 (x,y)，索引 index = x + y*size_x。
// g 值(_accumulated_cost)初始化为无穷大，visited 标记即 closed set。
Node2D::Node2D(const unsigned int index)
: parent(nullptr),
  _cell_cost(std::numeric_limits<float>::quiet_NaN()),
  _accumulated_cost(std::numeric_limits<float>::max()),
  _index(index),
  _was_visited(false),
  _is_queued(false)
{
}

Node2D::~Node2D()
{
  parent = nullptr;
}

void Node2D::reset()
{
  parent = nullptr;
  _cell_cost = std::numeric_limits<float>::quiet_NaN();
  _accumulated_cost = std::numeric_limits<float>::max();
  _was_visited = false;
  _is_queued = false;
}

bool Node2D::isNodeValid(
  const bool & traverse_unknown,
  GridCollisionChecker * collision_checker)
{
  // 【中文注释】按一维索引做碰撞检查，通过后缓存该格代价值
  if (collision_checker->inCollision(this->getIndex(), traverse_unknown)) {
    return false;
  }

  _cell_cost = collision_checker->getCost();
  return true;
}

float Node2D::getTraversalCost(const NodePtr & child)
{
  // 【中文注释】边代价 = 移动长度 * (1 + cost_travel_multiplier * 归一化地图代价)：
  // 对角移动长度 sqrt(2)（用距离平方 >1.05 判断，容许浮点误差），
  // 直线移动长度 1.0。归一化代价 = 格代价/252，权重 cost_travel_multiplier
  // 决定路径"绕开高代价区"的强度。
  float normalized_cost = child->getCost() / 252.0;
  const Coordinates A = getCoords(child->getIndex());
  const Coordinates B = getCoords(this->getIndex());
  const float & dx = A.x - B.x;
  const float & dy = A.y - B.y;
  static float sqrt_2 = sqrt(2);

  // If a diagonal move, travel cost is sqrt(2) not 1.0.
  if ((dx * dx + dy * dy) > 1.05) {
    return sqrt_2 * (1.0 + cost_travel_multiplier * normalized_cost);
  }

  // Length = 1.0
  return 1.0 + cost_travel_multiplier * normalized_cost;
}

float Node2D::getHeuristicCost(
  const Coordinates & node_coords,
  const Coordinates & goal_coordinates,
  const nav2_costmap_2d::Costmap2D * /*costmap*/)
{
  // Using Moore distance as it more accurately represents the distances
  // even a Van Neumann neighborhood robot can navigate.
  // 【中文注释】启发函数：到目标的欧氏距离（Moore 邻域下的可采纳下界，
  // 不会高估 8 邻域移动的真实代价）
  auto dx = goal_coordinates.x - node_coords.x;
  auto dy = goal_coordinates.y - node_coords.y;
  return std::sqrt(dx * dx + dy * dy);
}

void Node2D::initMotionModel(
  const MotionModel & motion_model,
  unsigned int & x_size_uint,
  unsigned int & /*size_y*/,
  unsigned int & /*num_angle_quantization*/,
  SearchInfo & search_info)
{
  // 【中文注释】初始化 2D 运动模型：只允许 TWOD 模型，
  // 构建 8 邻域索引偏移表（-1/+1 为左右，±x_size 为上下，组合为对角）。
  if (motion_model != MotionModel::TWOD) {
    throw std::runtime_error("Invalid motion model for 2D node.");
  }

  int x_size = static_cast<int>(x_size_uint);
  cost_travel_multiplier = search_info.cost_penalty;
  _neighbors_grid_offsets = {-1, +1, -x_size, +x_size, -x_size - 1,
    -x_size + 1, +x_size - 1, +x_size + 1};
}

void Node2D::getNeighbors(
  std::function<bool(const unsigned int &, nav2_smac_planner::Node2D * &)> & NeighborGetter,
  GridCollisionChecker * collision_checker,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{
  // NOTE(stevemacenski): Irritatingly, the order here matters. If you start in free
  // space and then expand 8-connected, the first set of neighbors will be all cost
  // 1.0. Then its expansion will all be 2 * 1.0 but now multiple
  // nodes are touching that node so the last cell to update the back pointer wins.
  // Thusly, the ordering ends with the cardinal directions for both sets such that
  // behavior is consistent in large free spaces between them.
  // 100  50   0
  // 100  50  50
  // 100 100 100   where lower-middle '100' is visited with same cost by both bottom '50' nodes
  // Therefore, it is valuable to have some low-potential across the entire map
  // rather than a small inflation around the obstacles
  // 【中文注释】邻域遍历：对 8 个偏移逐一取候选邻居索引；
  // 坐标差 >1 说明因行环绕产生了非法邻居（如行尾+1 到了行首），跳过；
  // 通过碰撞检查且未访问的邻居加入列表。
  // 注意：偏移顺序会影响等代价时的父指针选择（见上方英文注释），
  // 靠后者在同代价时胜出，因此顺序不能随意改动。
  int index;
  NodePtr neighbor;
  int node_i = this->getIndex();
  const Coordinates parent = getCoords(this->getIndex());
  Coordinates child;

  for (unsigned int i = 0; i != _neighbors_grid_offsets.size(); ++i) {
    index = node_i + _neighbors_grid_offsets[i];

    // Check for wrap around conditions
    child = getCoords(index);
    if (fabs(parent.x - child.x) > 1 || fabs(parent.y - child.y) > 1) {
      continue;
    }

    if (NeighborGetter(index, neighbor)) {
      if (neighbor->isNodeValid(traverse_unknown, collision_checker) && !neighbor->wasVisited()) {
        neighbors.push_back(neighbor);
      }
    }
  }
}

bool Node2D::backtracePath(CoordinateVector & path)
{
  // 【中文注释】从目标沿 parent 链回溯到起点，输出逆序坐标路径
  if (!this->parent) {
    return false;
  }

  NodePtr current_node = this;

  while (current_node->parent) {
    path.push_back(
      Node2D::getCoords(current_node->getIndex()));
    current_node = current_node->parent;
  }

  // add the start pose
  path.push_back(Node2D::getCoords(current_node->getIndex()));

  return true;
}

}  // namespace nav2_smac_planner
