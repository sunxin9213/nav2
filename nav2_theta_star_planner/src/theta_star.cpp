//  Copyright 2020 Anshumaan Singh
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include <vector>
#include "nav2_theta_star_planner/theta_star.hpp"

namespace theta_star
{

// Theta* 的关键思想是：
// 在标准 A* 中，路径总是沿着网格边逐步移动；
// 但在很多环境中，机器人其实可以“直接看见并跨过中间一段空旷区域”，
// 因此它可以绕过中间节点，直接把更远的可见点连接起来，减少不必要的拐角。
// 这就是 Theta* 里 LOS (Line of Sight) 判断的意义：
// 只有当连接这两个状态之间不存在障碍时，才允许直接压缩路径，形成更自然的全局轨迹。
ThetaStar::ThetaStar()
: w_traversal_cost_(1.0),
  w_euc_cost_(2.0),
  w_heuristic_cost_(1.0),
  how_many_corners_(8),
  allow_unknown_(true),
  size_x_(0),
  size_y_(0),
  index_generated_(0)
{
  exp_node = new tree_node;
}

void ThetaStar::setStartAndGoal(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  // 把世界坐标中的起点/终点转换到栅格坐标，
  // 这样后续搜索和 LOS 判断都能在离散地图上进行，便于与 costmap 的索引逻辑保持一致。
  unsigned int s[2], d[2];
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, s[0], s[1]);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, d[0], d[1]);

  src_ = {static_cast<int>(s[0]), static_cast<int>(s[1])};
  dst_ = {static_cast<int>(d[0]), static_cast<int>(d[1])};
}

bool ThetaStar::generatePath(std::vector<coordsW> & raw_path)
{
  // 这是整个 Theta* 搜索的入口：
  // 1) 清空并重置内部节点缓存；
  // 2) 把起点放进搜索集合；
  // 3) 在优先队列中不断扩展最优候选节点；
  // 4) 通过 LOS 判断决定是否可以跳过中间栅格，降低路径弯折。
  resetContainers();
  addToNodesData(index_generated_);
  double src_g_cost = getTraversalCost(src_.x, src_.y), src_h_cost = getHCost(src_.x, src_.y);
  nodes_data_[index_generated_] =
  {src_.x, src_.y, src_g_cost, src_h_cost, &nodes_data_[index_generated_], true,
    src_g_cost + src_h_cost};
  queue_.push({&nodes_data_[index_generated_]});
  addIndex(src_.x, src_.y, &nodes_data_[index_generated_]);
  tree_node * curr_data = &nodes_data_[index_generated_];
  index_generated_++;
  nodes_opened = 0;

  while (!queue_.empty()) {
    nodes_opened++;

    if (isGoal(*curr_data)) {
      break;
    }

    // resetParent() 会尝试用当前节点的父节点的父节点作为更优父节点，
    // 如果 line-of-sight 允许，则直接跳过中间节点，减少冗余路径段。
    resetParent(curr_data);
    setNeighbors(curr_data);

    curr_data = queue_.top();
    queue_.pop();
  }

  if (queue_.empty()) {
    raw_path.clear();
    return false;
  }

  backtrace(raw_path, curr_data);
  clearQueue();

  return true;
}

void ThetaStar::resetParent(tree_node * curr_data)
{
  // 这一段是 Theta* 相比 A* 的关键优化：
  // 不是只允许父节点直接连接，而是尝试评估“当前节点的祖先是否更适合做父节点”，
  // 只要从当前节点到祖先节点之间满足 LOS，且总代价更小，就更新父节点。
  double g_cost, los_cost = 0;
  curr_data->is_in_queue = false;
  const tree_node * curr_par = curr_data->parent_id;
  const tree_node * maybe_par = curr_par->parent_id;

  if (losCheck(curr_data->x, curr_data->y, maybe_par->x, maybe_par->y, los_cost)) {
    g_cost = maybe_par->g +
      getEuclideanCost(curr_data->x, curr_data->y, maybe_par->x, maybe_par->y) + los_cost;

    if (g_cost < curr_data->g) {
      curr_data->parent_id = maybe_par;
      curr_data->g = g_cost;
      curr_data->f = g_cost + curr_data->h;
    }
  }
}

void ThetaStar::setNeighbors(const tree_node * curr_data)
{
  // 对当前状态进行局部扩展：遍历其邻居节点，
  // 对每个相邻栅格计算 g/h/f 值，并更新优先队列中的候选节点。
  int mx, my;
  tree_node * m_id = nullptr;
  double g_cost, h_cost, cal_cost;

  for (int i = 0; i < how_many_corners_; i++) {
    mx = curr_data->x + moves[i].x;
    my = curr_data->y + moves[i].y;

    if (withinLimits(mx, my)) {
      if (!isSafe(mx, my)) {
        continue;
      }
    } else {
      continue;
    }

    g_cost = curr_data->g + getEuclideanCost(curr_data->x, curr_data->y, mx, my) +
      getTraversalCost(mx, my);

    m_id = getIndex(mx, my);

    if (m_id == nullptr) {
      addToNodesData(index_generated_);
      m_id = &nodes_data_[index_generated_];
      addIndex(mx, my, m_id);
      index_generated_++;
    }

    exp_node = m_id;

    h_cost = getHCost(mx, my);
    cal_cost = g_cost + h_cost;
    if (exp_node->f > cal_cost) {
      exp_node->g = g_cost;
      exp_node->h = h_cost;
      exp_node->f = cal_cost;
      exp_node->parent_id = curr_data;
      if (!exp_node->is_in_queue) {
        exp_node->x = mx;
        exp_node->y = my;
        exp_node->is_in_queue = true;
        queue_.push({m_id});
      }
    }
  }
}

void ThetaStar::backtrace(std::vector<coordsW> & raw_points, const tree_node * curr_n) const
{
  // 从终点往回追踪父指针，直到回到起点，
  // 这样就能得到一条从起点到目标的路径，并随即转换到世界坐标系。
  std::vector<coordsW> path_rev;
  coordsW world{};
  do {
    costmap_->mapToWorld(curr_n->x, curr_n->y, world.x, world.y);
    path_rev.push_back(world);
    if (path_rev.size() > 1) {
      curr_n = curr_n->parent_id;
    }
  } while (curr_n->parent_id != curr_n);
  costmap_->mapToWorld(curr_n->x, curr_n->y, world.x, world.y);
  path_rev.push_back(world);

  raw_points.reserve(path_rev.size());
  for (int i = static_cast<int>(path_rev.size()) - 1; i >= 0; i--) {
    raw_points.push_back(path_rev[i]);
  }
}

bool ThetaStar::losCheck(
  const int & x0, const int & y0, const int & x1, const int & y1,
  double & sl_cost) const
{
  // LOS (Line of Sight) 判断的核心逻辑：
  // 检查从 (x0, y0) 到 (x1, y1) 的直线段是否穿过障碍，
  // 若不穿过障碍，则说明两点在“可见”的状态下可以直接连接，
  // 这正是 Theta* 与传统 A* 的关键区别。
  sl_cost = 0;

  int cx, cy;
  int dy = abs(y1 - y0), dx = abs(x1 - x0), f = 0;
  int sx, sy;
  sx = x1 > x0 ? 1 : -1;
  sy = y1 > y0 ? 1 : -1;

  int u_x = (sx - 1) / 2;
  int u_y = (sy - 1) / 2;
  cx = x0;
  cy = y0;

  if (dx >= dy) {
    while (cx != x1) {
      f += dy;
      if (f >= dx) {
        if (!isSafe(cx + u_x, cy + u_y, sl_cost)) {
          return false;
        }
        cy += sy;
        f -= dx;
      }
      if (f != 0 && !isSafe(cx + u_x, cy + u_y, sl_cost)) {
        return false;
      }
      if (dy == 0 && !isSafe(cx + u_x, cy, sl_cost) && !isSafe(cx + u_x, cy - 1, sl_cost)) {
        return false;
      }
      cx += sx;
    }
  } else {
    while (cy != y1) {
      f = f + dx;
      if (f >= dy) {
        if (!isSafe(cx + u_x, cy + u_y, sl_cost)) {
          return false;
        }
        cx += sx;
        f -= dy;
      }
      if (f != 0 && !isSafe(cx + u_x, cy + u_y, sl_cost)) {
        return false;
      }
      if (dx == 0 && !isSafe(cx, cy + u_y, sl_cost) && !isSafe(cx - 1, cy + u_y, sl_cost)) {
        return false;
      }
      cy += sy;
    }
  }
  return true;
}

void ThetaStar::resetContainers()
{
  // 每次搜索前都要重置节点索引和缓存状态，
  // 这是因为节点之间的父子关系、open set、visited 集合都在当前规划周期中有效，
  // 不能跨次循环污染旧状态。
  index_generated_ = 0;
  int last_size_x = size_x_;
  int last_size_y = size_y_;
  int curr_size_x = static_cast<int>(costmap_->getSizeInCellsX());
  int curr_size_y = static_cast<int>(costmap_->getSizeInCellsY());
  if (((last_size_x != curr_size_x) || (last_size_y != curr_size_y)) &&
    static_cast<int>(node_position_.size()) < (curr_size_x * curr_size_y))
  {
    initializePosn(curr_size_y * curr_size_x - last_size_y * last_size_x);
    nodes_data_.reserve(curr_size_x * curr_size_y);
  } else {
    initializePosn();
  }
  size_x_ = curr_size_x;
  size_y_ = curr_size_y;
}

void ThetaStar::initializePosn(int size_inc)
{
  if (!node_position_.empty()) {
    for (int i = 0; i < size_x_ * size_y_; i++) {
      node_position_[i] = nullptr;
    }
  }

  for (int i = 0; i < size_inc; i++) {
    node_position_.push_back(nullptr);
  }
}
}  //  namespace theta_star
