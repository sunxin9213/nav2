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

#include <math.h>
#include <chrono>
#include <vector>
#include <memory>
#include <algorithm>
#include <queue>
#include <limits>
#include <utility>

#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"

#include "nav2_smac_planner/node_hybrid.hpp"

using namespace std::chrono;  // NOLINT

namespace nav2_smac_planner
{

// defining static member for all instance to share
// 【中文注释】NodeHybrid 的静态成员（所有节点实例共享，属"全局搜索上下文"）：
// obstacle_heuristic_lookup_table：2D 障碍启发距离场（目标反向 Dijkstra 扩展结果）；
// travel_distance_cost：单步运动基元的基准长度（由最小转弯半径导出）；
// motion_table：运动基元表（HybridMotionTable，含所有投影/预计算三角值）；
// size_lookup：距离启发查找表的窗口尺寸（栅格数）；
// dist_heuristic_lookup_table：Dubins/RS 距离启发查找表（目标附近局部窗口）；
// sampled_costmap：用于障碍启发计算的下采样代价地图（2 倍下采样）；
// downsampler：下采样器实例；
// obstacle_heuristic_queue：障碍启发扩展用的优先队列（惰性 Dijkstra）。
LookupTable NodeHybrid::obstacle_heuristic_lookup_table;
double NodeHybrid::travel_distance_cost = sqrt(2);
HybridMotionTable NodeHybrid::motion_table;
float NodeHybrid::size_lookup = 25;
LookupTable NodeHybrid::dist_heuristic_lookup_table;
nav2_costmap_2d::Costmap2D * NodeHybrid::sampled_costmap = nullptr;
CostmapDownsampler NodeHybrid::downsampler;
ObstacleHeuristicQueue NodeHybrid::obstacle_heuristic_queue;

// Each of these tables are the projected motion models through
// time and space applied to the search on the current node in
// continuous map-coordinates (e.g. not meters but partial map cells)
// Currently, these are set to project *at minimum* into a neighboring
// cell. Though this could be later modified to project a certain
// amount of time or particular distance forward.

// http://planning.cs.uiuc.edu/node821.html
// Model for ackermann style vehicle with minimum radius restriction
void HybridMotionTable::initDubin(
  unsigned int & size_x_in,
  unsigned int & /*size_y_in*/,
  unsigned int & num_angle_quantization_in,
  SearchInfo & search_info)
{
  // 【中文注释】初始化 Dubins 运动模型（阿克曼结构、只能前进）。
  // 运动基元(motion primitives)的构造原理：
  //   在最小转弯半径 R 的圆上取一段圆弧，使弦长 >= sqrt(2)（保证一步跨出当前栅格），
  //   且转角是角度量化 bin 的整数倍。由 chord = 2*R*sin(angle/2) 反解出
  //   angle = 2*asin(sqrt(2)/(2R))，再向上取整到 bin 的整数倍（increments）。
  //   这样得到 3 个基元：直行、左转、右转（每个都保证进入新栅格）。
  // 代价惩罚参数从 search_info 复制：change_penalty(换向)、non_straight_penalty(非直线)、
  //   cost_penalty(地图代价)、reverse_penalty(倒车)、retrospective_penalty(回溯奖励)。
  size_x = size_x_in;
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;
  travel_distance_reward = 1.0f - search_info.retrospective_penalty;

  // if nothing changed, no need to re-compute primitives
  // 【中文注释】参数未变化时跳过昂贵的重新预计算
  if (num_angle_quantization_in == num_angle_quantization &&
    min_turning_radius == search_info.minimum_turning_radius &&
    motion_model == MotionModel::DUBIN)
  {
    return;
  }

  num_angle_quantization = num_angle_quantization_in;
  num_angle_quantization_float = static_cast<float>(num_angle_quantization);
  min_turning_radius = search_info.minimum_turning_radius;
  motion_model = MotionModel::DUBIN;

  // angle must meet 3 requirements:
  // 1) be increment of quantized bin size
  // 2) chord length must be greater than sqrt(2) to leave current cell
  // 3) maximum curvature must be respected, represented by minimum turning angle
  // Thusly:
  // On circle of radius minimum turning angle, we need select motion primatives
  // with chord length > sqrt(2) and be an increment of our bin size
  //
  // chord >= sqrt(2) >= 2 * R * sin (angle / 2); where angle / N = quantized bin size
  // Thusly: angle <= 2.0 * asin(sqrt(2) / (2 * R))
  // 【中文注释】由最小转弯半径导出的最大单步转角（不超过 bin 时取 1 个 bin）
  float angle = 2.0 * asin(sqrt(2.0) / (2 * min_turning_radius));
  // Now make sure angle is an increment of the quantized bin size
  // And since its based on the minimum chord, we need to make sure its always larger
  bin_size =
    2.0f * static_cast<float>(M_PI) / static_cast<float>(num_angle_quantization);
  float increments;
  if (angle < bin_size) {
    increments = 1.0f;
  } else {
    // Search dimensions are clean multiples of quantization - this prevents
    // paths with loops in them
    // 【中文注释】转角向上取整为 bin 整数倍（防止路径出现绕圈）
    increments = ceil(angle / bin_size);
  }
  angle = increments * bin_size;

  // find deflections
  // If we make a right triangle out of the chord in circle of radius
  // min turning angle, we can see that delta X = R * sin (angle)
  // 【中文注释】用直角三角形分解圆弧终点的偏移量：
  // dx = R*sin(angle)，dy = R - R*cos(angle)（圆心在正前方 R 处）
  float delta_x = min_turning_radius * sin(angle);
  // Using that same right triangle, we can see that the complement
  // to delta Y is R * cos (angle). If we subtract R, we get the actual value
  float delta_y = min_turning_radius - (min_turning_radius * cos(angle));

  // 【中文注释】三个运动基元（局部坐标系下的位移+角度变化，单位是栅格/bin）：
  // 直行(前向弦长)、左转、右转
  projections.clear();
  projections.reserve(3);
  projections.emplace_back(hypotf(delta_x, delta_y), 0.0, 0.0);  // Forward
  projections.emplace_back(delta_x, delta_y, increments);  // Left
  projections.emplace_back(delta_x, -delta_y, -increments);  // Right

  // Create the correct OMPL state space
  // 【中文注释】创建 OMPL Dubins 状态空间，供解析扩展和启发计算使用
  state_space = std::make_unique<ompl::base::DubinsStateSpace>(min_turning_radius);

  // Precompute projection deltas
  // 【中文注释】预计算旋转查表：delta_xs/delta_ys[基元 i][朝向 bin j]
  // = 基元 i 的局部位移旋转到朝向 j 的世界位移。搜索时直接查表，
  // 避免每次扩展做三角函数运算。trig_values 缓存每个 bin 的 cos/sin。
  delta_xs.resize(projections.size());
  delta_ys.resize(projections.size());
  trig_values.resize(num_angle_quantization);

  for (unsigned int i = 0; i != projections.size(); i++) {
    delta_xs[i].resize(num_angle_quantization);
    delta_ys[i].resize(num_angle_quantization);

    for (unsigned int j = 0; j != num_angle_quantization; j++) {
      double cos_theta = cos(bin_size * j);
      double sin_theta = sin(bin_size * j);
      if (i == 0) {
        // if first iteration, cache the trig values for later
        trig_values[j] = {cos_theta, sin_theta};
      }
      delta_xs[i][j] = projections[i]._x * cos_theta - projections[i]._y * sin_theta;
      delta_ys[i][j] = projections[i]._x * sin_theta + projections[i]._y * cos_theta;
    }
  }
}

// http://planning.cs.uiuc.edu/node822.html
// Same as Dubin model but now reverse is valid
// See notes in Dubin for explanation
// 【中文注释】Reeds-Shepp 运动模型初始化：与 Dubin 相同的基元构造方法，
// 但允许倒车，因此基元从 3 个扩展到 6 个：
// 前进(直/左/右) + 倒车(直/左/右)。注意倒车时左右转的角度符号相反。
// OMPL 状态空间换为 ReedsSheppStateSpace。
void HybridMotionTable::initReedsShepp(
  unsigned int & size_x_in,
  unsigned int & /*size_y_in*/,
  unsigned int & num_angle_quantization_in,
  SearchInfo & search_info)
{
  size_x = size_x_in;
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;
  travel_distance_reward = 1.0f - search_info.retrospective_penalty;

  // if nothing changed, no need to re-compute primitives
  if (num_angle_quantization_in == num_angle_quantization &&
    min_turning_radius == search_info.minimum_turning_radius &&
    motion_model == MotionModel::REEDS_SHEPP)
  {
    return;
  }

  num_angle_quantization = num_angle_quantization_in;
  num_angle_quantization_float = static_cast<float>(num_angle_quantization);
  min_turning_radius = search_info.minimum_turning_radius;
  motion_model = MotionModel::REEDS_SHEPP;

  float angle = 2.0 * asin(sqrt(2.0) / (2 * min_turning_radius));
  bin_size =
    2.0f * static_cast<float>(M_PI) / static_cast<float>(num_angle_quantization);
  float increments;
  if (angle < bin_size) {
    increments = 1.0f;
  } else {
    increments = ceil(angle / bin_size);
  }
  angle = increments * bin_size;

  float delta_x = min_turning_radius * sin(angle);
  float delta_y = min_turning_radius - (min_turning_radius * cos(angle));

  projections.clear();
  projections.reserve(6);
  projections.emplace_back(hypotf(delta_x, delta_y), 0.0, 0.0);  // Forward
  projections.emplace_back(delta_x, delta_y, increments);  // Forward + Left
  projections.emplace_back(delta_x, -delta_y, -increments);  // Forward + Right
  projections.emplace_back(-hypotf(delta_x, delta_y), 0.0, 0.0);  // Backward
  projections.emplace_back(-delta_x, delta_y, -increments);  // Backward + Left
  projections.emplace_back(-delta_x, -delta_y, increments);  // Backward + Right

  // Create the correct OMPL state space
  state_space = std::make_unique<ompl::base::ReedsSheppStateSpace>(min_turning_radius);

  // Precompute projection deltas
  delta_xs.resize(projections.size());
  delta_ys.resize(projections.size());
  trig_values.resize(num_angle_quantization);

  for (unsigned int i = 0; i != projections.size(); i++) {
    delta_xs[i].resize(num_angle_quantization);
    delta_ys[i].resize(num_angle_quantization);

    for (unsigned int j = 0; j != num_angle_quantization; j++) {
      double cos_theta = cos(bin_size * j);
      double sin_theta = sin(bin_size * j);
      if (i == 0) {
        // if first iteration, cache the trig values for later
        trig_values[j] = {cos_theta, sin_theta};
      }
      delta_xs[i][j] = projections[i]._x * cos_theta - projections[i]._y * sin_theta;
      delta_ys[i][j] = projections[i]._x * sin_theta + projections[i]._y * cos_theta;
    }
  }
}

MotionPoses HybridMotionTable::getProjections(const NodeHybrid * node)
{
  // 【中文注释】返回当前节点的所有运动基元投影：
  // 对每个基元，用节点当前朝向 bin 查预计算表得到世界坐标位移，
  // 叠加到节点坐标上得到新位姿；角度 = 当前 bin + 基元转角 bin 数（归一到 [0,N)）。
  MotionPoses projection_list;
  projection_list.reserve(projections.size());

  for (unsigned int i = 0; i != projections.size(); i++) {
    const MotionPose & motion_model = projections[i];

    // normalize theta, I know its overkill, but I've been burned before...
    const float & node_heading = node->pose.theta;
    float new_heading = node_heading + motion_model._theta;

    if (new_heading < 0.0) {
      new_heading += num_angle_quantization_float;
    }

    if (new_heading >= num_angle_quantization_float) {
      new_heading -= num_angle_quantization_float;
    }

    projection_list.emplace_back(
      delta_xs[i][node_heading] + node->pose.x,
      delta_ys[i][node_heading] + node->pose.y,
      new_heading);
  }

  return projection_list;
}

unsigned int HybridMotionTable::getClosestAngularBin(const double & theta)
{
  // 【中文注释】把连续弧度角量化到最近的 bin 索引（环绕处理：超出 N 归 0）
  auto bin = static_cast<unsigned int>(round(static_cast<float>(theta) / bin_size));
  return bin < num_angle_quantization ? bin : 0u;
}

float HybridMotionTable::getAngleFromBin(const unsigned int & bin_idx)
{
  // 【中文注释】bin 索引转回实际弧度角
  return bin_idx * bin_size;
}

// 【中文注释】NodeHybrid：混合 A* 的 SE(2) 状态节点。
// 状态 = (x, y, theta bin)，索引 = x + y*size_x + theta*size_x*size_y。
// 注意 pose 中的 theta 是角度 bin 序号而非弧度。
NodeHybrid::NodeHybrid(const unsigned int index)
: parent(nullptr),
  pose(0.0f, 0.0f, 0.0f),
  _cell_cost(std::numeric_limits<float>::quiet_NaN()),
  _accumulated_cost(std::numeric_limits<float>::max()),
  _index(index),
  _was_visited(false),
  _motion_primitive_index(std::numeric_limits<unsigned int>::max())
{
}

NodeHybrid::~NodeHybrid()
{
  parent = nullptr;
}

void NodeHybrid::reset()
{
  parent = nullptr;
  _cell_cost = std::numeric_limits<float>::quiet_NaN();
  _accumulated_cost = std::numeric_limits<float>::max();
  _was_visited = false;
  _motion_primitive_index = std::numeric_limits<unsigned int>::max();
  pose.x = 0.0f;
  pose.y = 0.0f;
  pose.theta = 0.0f;
}

bool NodeHybrid::isNodeValid(
  const bool & traverse_unknown,
  GridCollisionChecker * collision_checker)
{
  // 【中文注释】节点合法性 = 无碰撞：用 SE(2) 位姿做碰撞检查
  // （theta 传 bin 号，检查器内部用它取预旋转足迹），
  // 通过后缓存该位姿的代价值供 getTraversalCost 使用。
  if (collision_checker->inCollision(
      this->pose.x, this->pose.y, this->pose.theta /*bin number*/, traverse_unknown))
  {
    return false;
  }

  _cell_cost = collision_checker->getCost();
  return true;
}

float NodeHybrid::getTraversalCost(const NodePtr & child)
{
  // 【中文注释】计算从本节点到子节点的边代价（混合 A* 代价设计的核心）：
  //   基础项 = 基元长度 * (行进奖励 + cost_penalty * 归一化地图代价)
  //   归一化代价 = 该格代价 / 252，体现"偏向低代价区域"；
  //   直行(基元 0/3)：无附加惩罚；
  //   转弯且与父节点基元相同：乘 non_straight_penalty（鼓励坚持一个转向）；
  //   转弯且换向：乘 non_straight_penalty + change_penalty（惩罚左右摆动）；
  //   倒车(基元 > 2)：再乘 reverse_penalty。
  const float normalized_cost = child->getCost() / 252.0;
  if (std::isnan(normalized_cost)) {
    throw std::runtime_error(
            "Node attempted to get traversal "
            "cost without a known SE2 collision cost!");
  }

  // this is the first node
  // 【中文注释】起点没有基元索引：只收基准行进代价
  if (getMotionPrimitiveIndex() == std::numeric_limits<unsigned int>::max()) {
    return NodeHybrid::travel_distance_cost;
  }

  float travel_cost = 0.0;
  float travel_cost_raw =
    NodeHybrid::travel_distance_cost *
    (motion_table.travel_distance_reward + motion_table.cost_penalty * normalized_cost);

  if (child->getMotionPrimitiveIndex() == 0 || child->getMotionPrimitiveIndex() == 3) {
    // New motion is a straight motion, no additional costs to be applied
    travel_cost = travel_cost_raw;
  } else {
    if (getMotionPrimitiveIndex() == child->getMotionPrimitiveIndex()) {
      // Turning motion but keeps in same direction: encourages to commit to turning if starting it
      travel_cost = travel_cost_raw * motion_table.non_straight_penalty;
    } else {
      // Turning motion and changing direction: penalizes wiggling
      travel_cost = travel_cost_raw *
        (motion_table.non_straight_penalty + motion_table.change_penalty);
    }
  }

  if (child->getMotionPrimitiveIndex() > 2) {
    // reverse direction
    travel_cost *= motion_table.reverse_penalty;
  }

  return travel_cost;
}

float NodeHybrid::getHeuristicCost(
  const Coordinates & node_coords,
  const Coordinates & goal_coords,
  const nav2_costmap_2d::Costmap2D * /*costmap*/)
{
  // 【中文注释】混合 A* 的启发函数 = max(障碍启发, 距离启发)：
  //   障碍启发：下采样地图上从目标反向 Dijkstra 的 2D 代价距离（考虑障碍）；
  //   距离启发：目标附近局部窗口内预计算的 Dubins/RS 距离（考虑转弯半径）；
  // 两者取大保证启发值的引导力，同时都不高估真实代价（可采纳）。
  const float obstacle_heuristic =
    getObstacleHeuristic(node_coords, goal_coords, motion_table.cost_penalty);
  const float dist_heuristic = getDistanceHeuristic(node_coords, goal_coords, obstacle_heuristic);
  return std::max(obstacle_heuristic, dist_heuristic);
}

void NodeHybrid::initMotionModel(
  const MotionModel & motion_model,
  unsigned int & size_x,
  unsigned int & size_y,
  unsigned int & num_angle_quantization,
  SearchInfo & search_info)
{
  // find the motion model selected
  switch (motion_model) {
    case MotionModel::DUBIN:
      motion_table.initDubin(size_x, size_y, num_angle_quantization, search_info);
      break;
    case MotionModel::REEDS_SHEPP:
      motion_table.initReedsShepp(size_x, size_y, num_angle_quantization, search_info);
      break;
    default:
      throw std::runtime_error(
              "Invalid motion model for Hybrid A*. Please select between"
              " Dubin (Ackermann forward only),"
              " Reeds-Shepp (Ackermann forward and back).");
  }

  travel_distance_cost = motion_table.projections[0]._x;
}

inline float distanceHeuristic2D(
  const unsigned int idx, const unsigned int size_x,
  const unsigned int target_x, const unsigned int target_y)
{
  int dx = static_cast<int>(idx % size_x) - static_cast<int>(target_x);
  int dy = static_cast<int>(idx / size_x) - static_cast<int>(target_y);
  return std::sqrt(dx * dx + dy * dy);
}

void NodeHybrid::resetObstacleHeuristic(
  nav2_costmap_2d::Costmap2D * costmap,
  const unsigned int & start_x, const unsigned int & start_y,
  const unsigned int & goal_x, const unsigned int & goal_y)
{
  // Downsample costmap 2x to compute a sparse obstacle heuristic. This speeds up
  // the planner considerably to search through 75% less cells with no detectable
  // erosion of path quality after even modest smoothing. The error would be no more
  // than 0.05 * normalized cost. Since this is just a search prior, there's no loss in generality
  // 【中文注释】重置障碍启发（每次规划周期调用）：
  // 1) 把 costmap 2 倍下采样（搜索量减少 75%，误差 <0.05 归一化代价，可接受）；
  // 2) 清零距离场查找表和优先队列；
  // 3) 把目标点（坐标除以 2 对应下采样图）作为波源入队。
  // 查找表约定：0.0=未初始化，负值=open set 中的临时 g 值，正值=closed 的最终 g 值。
  std::weak_ptr<nav2_util::LifecycleNode> ptr;
  downsampler.on_configure(ptr, "fake_frame", "fake_topic", costmap, 2.0, true);
  downsampler.on_activate();
  sampled_costmap = downsampler.downsample(2.0);

  // Clear lookup table
  // 【中文注释】尺寸一致时整体清零；尺寸变化时扩容并只清旧范围
  unsigned int size = sampled_costmap->getSizeInCellsX() * sampled_costmap->getSizeInCellsY();
  if (obstacle_heuristic_lookup_table.size() == size) {
    // must reset all values
    std::fill(
      obstacle_heuristic_lookup_table.begin(),
      obstacle_heuristic_lookup_table.end(), 0.0);
  } else {
    unsigned int obstacle_size = obstacle_heuristic_lookup_table.size();
    obstacle_heuristic_lookup_table.resize(size, 0.0);
    // must reset values for non-constructed indices
    std::fill_n(
      obstacle_heuristic_lookup_table.begin(), obstacle_size, 0.0);
  }

  obstacle_heuristic_queue.clear();
  obstacle_heuristic_queue.reserve(
    sampled_costmap->getSizeInCellsX() * sampled_costmap->getSizeInCellsY());

  // Set initial goal point to queue from. Divided by 2 due to downsampled costmap.
  // 【中文注释】目标格入队：优先级 = g + 到起点的欧氏距离（A* 式排序），
  // 初始 g 值取 -0.00001（负号表示在 open set，非零以区别"未初始化"）
  const unsigned int size_x = sampled_costmap->getSizeInCellsX();
  const unsigned int goal_index = floor(goal_y / 2.0) * size_x + floor(goal_x / 2.0);
  obstacle_heuristic_queue.emplace_back(
    distanceHeuristic2D(goal_index, size_x, start_x, start_y), goal_index);

  // initialize goal cell with a very small value to differentiate it from 0.0 (~uninitialized)
  // the negative value means the cell is in the open set
  obstacle_heuristic_lookup_table[goal_index] = -0.00001f;
}

float NodeHybrid::getObstacleHeuristic(
  const Coordinates & node_coords,
  const Coordinates & goal_coords,
  const double & cost_penalty)
{
  // If already expanded, return the cost
  // 【中文注释】查询障碍启发：把节点坐标映射到下采样图（除以 2），
  // 若该格已完成 Dijkstra 扩展（查找表值为正），直接返回（*2 折算回原分辨率）。
  const unsigned int size_x = sampled_costmap->getSizeInCellsX();
  // Divided by 2 due to downsampled costmap.
  const unsigned int start_y = floor(node_coords.y / 2.0);
  const unsigned int start_x = floor(node_coords.x / 2.0);
  const unsigned int start_index = start_y * size_x + start_x;
  const float & requested_node_cost = obstacle_heuristic_lookup_table[start_index];
  if (requested_node_cost > 0.0f) {
    // costs are doubled due to downsampling
    return 2.0 * requested_node_cost;
  }

  // If not, expand until it is included. This dynamic programming ensures that
  // we only expand the MINIMUM spanning set of the costmap per planning request.
  // Rather than naively expanding the entire (potentially massive) map for a limited
  // path, we only expand to the extent required for the furthest expansion in the
  // search-planning request that dynamically updates during search as needed.
  // 【中文注释】惰性 Dijkstra（按需扩展）：目标格尚未波及到查询点时，
  // 从现有 open set 继续扩展直到覆盖查询点。这样一次规划只需扩展
  // "搜索实际到达的最远范围"，而不是整张大地图，查询点动态驱动扩展量。

  // start_x and start_y have changed since last call
  // we need to recompute 2D distance heuristic and reprioritize queue
  // 【中文注释】由于本次查询点不同（相当于 A* 的"目标"变了），
  // 重算队列中所有元素的优先级（g + 到新查询点的欧氏距离）并重建堆
  for (auto & n : obstacle_heuristic_queue) {
    n.first = -obstacle_heuristic_lookup_table[n.second] +
      distanceHeuristic2D(n.second, size_x, start_x, start_y);
  }
  std::make_heap(
    obstacle_heuristic_queue.begin(), obstacle_heuristic_queue.end(),
    ObstacleHeuristicComparator{});

  const int size_x_int = static_cast<int>(size_x);
  const unsigned int size_y = sampled_costmap->getSizeInCellsY();
  const float sqrt_2 = sqrt(2);
  float c_cost, cost, travel_cost, new_cost, existing_cost;
  unsigned int idx, mx, my, mx_idx, my_idx;
  unsigned int new_idx = 0;

  // 【中文注释】8 邻域索引偏移（左右/上下/四个对角）
  const std::vector<int> neighborhood = {1, -1,  // left right
    size_x_int, -size_x_int,  // up down
    size_x_int + 1, size_x_int - 1,  // upper diagonals
    -size_x_int + 1, -size_x_int - 1};  // lower diagonals

  // 【中文注释】A* 式主循环：弹出 f=g+h 最小的格子，关闭（正值化），
  // 松弛 8 邻域；边代价 = 步长(1 或 sqrt2) * (1 + cost_penalty*代价/252)，
  // 代价 >= INSCRIBED(内切) 的格子直接跳过。水平/垂直环绕检查防止跨行误判。
  while (!obstacle_heuristic_queue.empty()) {
    idx = obstacle_heuristic_queue.front().second;
    std::pop_heap(
      obstacle_heuristic_queue.begin(), obstacle_heuristic_queue.end(),
      ObstacleHeuristicComparator{});
    obstacle_heuristic_queue.pop_back();
    c_cost = obstacle_heuristic_lookup_table[idx];
    if (c_cost > 0.0f) {
      // cell has been processed and closed, no further cost improvements
      // are mathematically possible thanks to euclidean distance heuristic consistency
      continue;
    }
    c_cost = -c_cost;
    obstacle_heuristic_lookup_table[idx] = c_cost;  // set a positive value to close the cell

    my_idx = idx / size_x;
    mx_idx = idx - (my_idx * size_x);

    // find neighbors
    for (unsigned int i = 0; i != neighborhood.size(); i++) {
      new_idx = static_cast<unsigned int>(static_cast<int>(idx) + neighborhood[i]);

      // if neighbor path is better and non-lethal, set new cost and add to queue
      if (new_idx < size_x * size_y) {
        cost = static_cast<float>(sampled_costmap->getCost(new_idx));
        if (cost >= INSCRIBED) {
          continue;
        }

        my = new_idx / size_x;
        mx = new_idx - (my * size_x);

        if (mx == 0 && mx_idx >= size_x - 1 || mx >= size_x - 1 && mx_idx == 0) {
          continue;
        }
        if (my == 0 && my_idx >= size_y - 1 || my >= size_y - 1 && my_idx == 0) {
          continue;
        }

        existing_cost = obstacle_heuristic_lookup_table[new_idx];
        if (existing_cost <= 0.0f) {
          travel_cost =
            ((i <= 3) ? 1.0f : sqrt_2) * (1.0f + (cost_penalty * cost / 252.0f));
          new_cost = c_cost + travel_cost;
          if (existing_cost == 0.0f || -existing_cost > new_cost) {
            // the negative value means the cell is in the open set
            obstacle_heuristic_lookup_table[new_idx] = -new_cost;
            obstacle_heuristic_queue.emplace_back(
              new_cost + distanceHeuristic2D(new_idx, size_x, start_x, start_y), new_idx);
            std::push_heap(
              obstacle_heuristic_queue.begin(), obstacle_heuristic_queue.end(),
              ObstacleHeuristicComparator{});
          }
        }
      }
    }

    // 【中文注释】查询点已被关闭（波前到达），立即停止扩展
    if (idx == start_index) {
      break;
    }
  }

  // return requested_node_cost which has been updated by the search
  // costs are doubled due to downsampling
  return 2.0 * requested_node_cost;
}

float NodeHybrid::getDistanceHeuristic(
  const Coordinates & node_coords,
  const Coordinates & goal_coords,
  const float & obstacle_heuristic)
{
  // rotate and translate node_coords such that goal_coords relative is (0,0,0)
  // Due to the rounding involved in exact cell increments for caching,
  // this is not an exact replica of a live heuristic, but has bounded error.
  // (Usually less than 1 cell)
  // 【中文注释】距离启发查询：查"目标附近局部窗口"的 Dubins/RS 距离表。
  // 先把节点坐标变换到目标坐标系（目标处于原点、朝向 0）：
  // 用目标朝向的逆旋转（cos(-th)=cos th, sin(-th)=-sin th）旋转位移向量，
  // 角度差也归一到 [0, N)。

  // This angle is negative since we are de-rotating the current node
  // by the goal angle; cos(-th) = cos(th) & sin(-th) = -sin(th)
  const TrigValues & trig_vals = motion_table.trig_values[goal_coords.theta];
  const float cos_th = trig_vals.first;
  const float sin_th = -trig_vals.second;
  const float dx = node_coords.x - goal_coords.x;
  const float dy = node_coords.y - goal_coords.y;

  double dtheta_bin = node_coords.theta - goal_coords.theta;
  if (dtheta_bin < 0) {
    dtheta_bin += motion_table.num_angle_quantization;
  }
  if (dtheta_bin > motion_table.num_angle_quantization) {
    dtheta_bin -= motion_table.num_angle_quantization;
  }

  Coordinates node_coords_relative(
    round(dx * cos_th - dy * sin_th),
    round(dx * sin_th + dy * cos_th),
    round(dtheta_bin));

  // Check if the relative node coordinate is within the localized window around the goal
  // to apply the distance heuristic. Since the lookup table is contains only the positive
  // X axis, we mirror the Y and theta values across the X axis to find the heuristic values.
  // 【中文注释】局部窗口命中：相对坐标在 size_lookup 窗口内则查表。
  // 由于启发空间关于 X 轴对称，表只存了 y>=0 的半边：
  // y<0 时把 y 和角度 bin 做镜像（theta' = N - theta）再查，节省一半内存。
  float motion_heuristic = 0.0;
  const int floored_size = floor(size_lookup / 2.0);
  const int ceiling_size = ceil(size_lookup / 2.0);
  const float mirrored_relative_y = abs(node_coords_relative.y);
  if (abs(node_coords_relative.x) < floored_size && mirrored_relative_y < floored_size) {
    // Need to mirror angle if Y coordinate was mirrored
    int theta_pos;
    if (node_coords_relative.y < 0.0) {
      theta_pos = motion_table.num_angle_quantization - node_coords_relative.theta;
    } else {
      theta_pos = node_coords_relative.theta;
    }
    const int x_pos = node_coords_relative.x + floored_size;
    const int y_pos = static_cast<int>(mirrored_relative_y);
    const int index =
      x_pos * ceiling_size * motion_table.num_angle_quantization +
      y_pos * motion_table.num_angle_quantization +
      theta_pos;
    motion_heuristic = dist_heuristic_lookup_table[index];
  } else if (obstacle_heuristic <= 0.0) {
    // If no obstacle heuristic value, must have some H to use
    // In nominal situations, this should never be called.
    // 【中文注释】窗口外且障碍启发不可用（正常不会发生）：
    // 退化为现场调用 OMPL 计算一次 Dubins/RS 距离
    static ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
    to[0] = goal_coords.x;
    to[1] = goal_coords.y;
    to[2] = goal_coords.theta * motion_table.num_angle_quantization;
    from[0] = node_coords.x;
    from[1] = node_coords.y;
    from[2] = node_coords.theta * motion_table.num_angle_quantization;
    motion_heuristic = motion_table.state_space->distance(from(), to());
  }

  return motion_heuristic;
}

void NodeHybrid::precomputeDistanceHeuristic(
  const float & lookup_table_dim,
  const MotionModel & motion_model,
  const unsigned int & dim_3_size,
  const SearchInfo & search_info)
{
  // Dubin or Reeds-Shepp shortest distances
  // 【中文注释】预计算距离启发查找表（一次性，configure 阶段执行）：
  // 在目标点周围的 size_lookup x size_lookup 局部窗口内，
  // 对每个相对位置 (x, y) 和每个朝向 bin，离线计算到原点的
  // Dubins/Reeds-Shepp 最短距离并缓存。
  // 利用对称性只存 y>=0 的半边（查询时镜像），内存减半、窗口可更大。
  if (motion_model == MotionModel::DUBIN) {
    motion_table.state_space = std::make_unique<ompl::base::DubinsStateSpace>(
      search_info.minimum_turning_radius);
  } else if (motion_model == MotionModel::REEDS_SHEPP) {
    motion_table.state_space = std::make_unique<ompl::base::ReedsSheppStateSpace>(
      search_info.minimum_turning_radius);
  } else {
    throw std::runtime_error(
            "Node attempted to precompute distance heuristics "
            "with invalid motion model!");
  }

  ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
  to[0] = 0.0;
  to[1] = 0.0;
  to[2] = 0.0;
  size_lookup = lookup_table_dim;
  float motion_heuristic = 0.0;
  unsigned int index = 0;
  int dim_3_size_int = static_cast<int>(dim_3_size);
  float angular_bin_size = 2 * M_PI / static_cast<float>(dim_3_size);

  // Create a lookup table of Dubin/Reeds-Shepp distances in a window around the goal
  // to help drive the search towards admissible approaches. Deu to symmetries in the
  // Heuristic space, we need to only store 2 of the 4 quadrants and simply mirror
  // around the X axis any relative node lookup. This reduces memory overhead and increases
  // the size of a window a platform can store in memory.
  // 【中文注释】三重循环填充表：x 遍历窗口全宽，y 只取上半，
  // heading 遍历所有角度 bin；OMPL 的 distance() 给出最短曲线长度
  dist_heuristic_lookup_table.resize(size_lookup * ceil(size_lookup / 2.0) * dim_3_size_int);
  for (float x = ceil(-size_lookup / 2.0); x <= floor(size_lookup / 2.0); x += 1.0) {
    for (float y = 0.0; y <= floor(size_lookup / 2.0); y += 1.0) {
      for (int heading = 0; heading != dim_3_size_int; heading++) {
        from[0] = x;
        from[1] = y;
        from[2] = heading * angular_bin_size;
        motion_heuristic = motion_table.state_space->distance(from(), to());
        dist_heuristic_lookup_table[index] = motion_heuristic;
        index++;
      }
    }
  }
}

void NodeHybrid::getNeighbors(
  std::function<bool(const unsigned int &, nav2_smac_planner::NodeHybrid * &)> & NeighborGetter,
  GridCollisionChecker * collision_checker,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{
  // 【中文注释】混合 A* 的邻域扩展：不是固定 8 邻域，而是沿运动基元扩展。
  // 对每个基元投影出的新 SE(2) 位姿：
  //   1) 由 (x, y, theta bin) 计算状态索引并从图中取节点；
  //   2) 节点未被访问过时，临时写入连续坐标位姿（保持亚栅格精度），
  //      做碰撞检查；通过则记录所用基元索引并加入邻居列表，
  //      失败则回滚位姿，不污染该节点后续使用。
  unsigned int index = 0;
  NodePtr neighbor = nullptr;
  Coordinates initial_node_coords;
  const MotionPoses motion_projections = motion_table.getProjections(this);

  for (unsigned int i = 0; i != motion_projections.size(); i++) {
    index = NodeHybrid::getIndex(
      static_cast<unsigned int>(motion_projections[i]._x),
      static_cast<unsigned int>(motion_projections[i]._y),
      static_cast<unsigned int>(motion_projections[i]._theta),
      motion_table.size_x, motion_table.num_angle_quantization);

    if (NeighborGetter(index, neighbor) && !neighbor->wasVisited()) {
      // Cache the initial pose in case it was visited but valid
      // don't want to disrupt continuous coordinate expansion
      initial_node_coords = neighbor->pose;
      neighbor->setPose(
        Coordinates(
          motion_projections[i]._x,
          motion_projections[i]._y,
          motion_projections[i]._theta));
      if (neighbor->isNodeValid(traverse_unknown, collision_checker)) {
        neighbor->setMotionPrimitiveIndex(i);
        neighbors.push_back(neighbor);
      } else {
        neighbor->setPose(initial_node_coords);
      }
    }
  }
}

bool NodeHybrid::backtracePath(CoordinateVector & path)
{
  // 【中文注释】从目标节点沿 parent 指针回溯到起点，
  // 生成逆序路径（目标在前），并把每个点的 theta 从 bin 序号转回弧度。
  if (!this->parent) {
    return false;
  }

  NodePtr current_node = this;

  while (current_node->parent) {
    path.push_back(current_node->pose);
    // Convert angle to radians
    path.back().theta = NodeHybrid::motion_table.getAngleFromBin(path.back().theta);
    current_node = current_node->parent;
  }

  // add the start pose
  path.push_back(current_node->pose);
  // Convert angle to radians
  path.back().theta = NodeHybrid::motion_table.getAngleFromBin(path.back().theta);

  return true;
}

}  // namespace nav2_smac_planner
