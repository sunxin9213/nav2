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

#include <math.h>
#include <chrono>
#include <vector>
#include <memory>
#include <algorithm>
#include <queue>
#include <limits>
#include <string>
#include <fstream>
#include <cmath>

#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"

#include "nav2_smac_planner/node_lattice.hpp"

using namespace std::chrono;  // NOLINT

namespace nav2_smac_planner
{

// defining static member for all instance to share
// 【中文注释】NodeLattice 静态成员：motion_table 为状态格运动表
// （从 JSON 轨迹文件加载的预计算控制采样基元）；
// dist_heuristic_lookup_table 为 Dubins/RS 距离启发查找表（与 Hybrid 相同机制）。
LatticeMotionTable NodeLattice::motion_table;
float NodeLattice::size_lookup = 25;
LookupTable NodeLattice::dist_heuristic_lookup_table;

// Each of these tables are the projected motion models through
// time and space applied to the search on the current node in
// continuous map-coordinates (e.g. not meters but partial map cells)
// Currently, these are set to project *at minimum* into a neighboring
// cell. Though this could be later modified to project a certain
// amount of time or particular distance forward.
void LatticeMotionTable::initMotionModel(
  unsigned int & size_x_in,
  SearchInfo & search_info)
{
  // 【中文注释】初始化状态格运动模型：从 lattice_filepath 指定的 JSON 文件
  // 加载"最小控制集"(minimum control set)——该文件由外部工具离线生成，
  // 包含对机器人做控制空间采样得到的一组运动基元（每个基元是一条
  // 带长度的轨迹，含中间采样位姿），已保证满足曲率/速度/加速度等运动学约束。
  // 这里把基元按起始朝向分桶存入 motion_primitives[heading]。
  size_x = size_x_in;

  // 【中文注释】文件路径未变则跳过重复解析
  if (current_lattice_filepath == search_info.lattice_filepath) {
    return;
  }

  size_x = size_x_in;
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;
  travel_distance_reward = 1.0f - search_info.retrospective_penalty;
  current_lattice_filepath = search_info.lattice_filepath;
  allow_reverse_expansion = search_info.allow_reverse_expansion;
  rotation_penalty = search_info.rotation_penalty;

  // Get the metadata about this minimum control set
  // 【中文注释】读取文件元数据（朝向数、最小转弯半径、栅格分辨率等）并解析 JSON
  lattice_metadata = getLatticeMetadata(current_lattice_filepath);
  std::ifstream latticeFile(current_lattice_filepath);
  if (!latticeFile.is_open()) {
    throw std::runtime_error("Could not open lattice file");
  }
  nlohmann::json json;
  latticeFile >> json;
  num_angle_quantization = lattice_metadata.number_of_headings;

  // 【中文注释】按是否允许倒车选择 OMPL 状态空间（Dubins 或 Reeds-Shepp），
  // 供解析扩展和距离启发使用
  if (!state_space) {
    if (!allow_reverse_expansion) {
      state_space = std::make_unique<ompl::base::DubinsStateSpace>(
        lattice_metadata.min_turning_radius);
    } else {
      state_space = std::make_unique<ompl::base::ReedsSheppStateSpace>(
        lattice_metadata.min_turning_radius);
    }
  }

  // Populate the motion primitives at each heading angle
  // 【中文注释】按起始朝向角把基元分桶：JSON 中基元按 start_angle 排序，
  // 遇到新起始角就开一个新桶，motion_primitives[i] 即朝向 bin i 可用的基元集
  float prev_start_angle = 0.0;
  std::vector<MotionPrimitive> primitives;
  nlohmann::json json_primitives = json["primitives"];
  for (unsigned int i = 0; i < json_primitives.size(); ++i) {
    MotionPrimitive new_primitive;
    fromJsonToMotionPrimitive(json_primitives[i], new_primitive);

    if (prev_start_angle != new_primitive.start_angle) {
      motion_primitives.push_back(primitives);
      primitives.clear();
      prev_start_angle = new_primitive.start_angle;
    }
    primitives.push_back(new_primitive);
  }
  motion_primitives.push_back(primitives);

  // Populate useful precomputed values to be leveraged
  // 【中文注释】预计算各朝向角的 cos/sin 供后用
  trig_values.reserve(lattice_metadata.number_of_headings);
  for (unsigned int i = 0; i < lattice_metadata.heading_angles.size(); ++i) {
    trig_values.emplace_back(
      cos(lattice_metadata.heading_angles[i]),
      sin(lattice_metadata.heading_angles[i]));
  }
}

MotionPrimitivePtrs LatticeMotionTable::getMotionPrimitives(const NodeLattice * node)
{
  // 【中文注释】返回当前节点可用的基元集合：
  // 先取当前朝向 bin 的基元；若允许倒车扩展，再取"相反朝向"(bin - N/2)
  // 的基元集合（倒着开时基元的起始朝向是车头反方向），两者拼接返回。
  MotionPrimitives & prims_at_heading = motion_primitives[node->pose.theta];
  MotionPrimitivePtrs primitive_projection_list;
  for (unsigned int i = 0; i != prims_at_heading.size(); i++) {
    primitive_projection_list.push_back(&prims_at_heading[i]);
  }

  if (allow_reverse_expansion) {
    // Find normalized heading bin of the reverse expansion
    // 【中文注释】计算相反朝向 bin 并归一化到 [0, N)
    double reserve_heading = node->pose.theta - (num_angle_quantization / 2);
    if (reserve_heading < 0) {
      reserve_heading += num_angle_quantization;
    }
    if (reserve_heading > num_angle_quantization) {
      reserve_heading -= num_angle_quantization;
    }

    MotionPrimitives & prims_at_reverse_heading = motion_primitives[reserve_heading];
    for (unsigned int i = 0; i != prims_at_reverse_heading.size(); i++) {
      primitive_projection_list.push_back(&prims_at_reverse_heading[i]);
    }
  }

  return primitive_projection_list;
}

LatticeMetadata LatticeMotionTable::getLatticeMetadata(const std::string & lattice_filepath)
{
  // 【中文注释】从 JSON 文件读 lattice_metadata 节并反序列化为结构体
  std::ifstream lattice_file(lattice_filepath);
  if (!lattice_file.is_open()) {
    throw std::runtime_error("Could not open lattice file!");
  }

  nlohmann::json j;
  lattice_file >> j;
  LatticeMetadata metadata;
  fromJsonToMetaData(j["lattice_metadata"], metadata);
  return metadata;
}

unsigned int LatticeMotionTable::getClosestAngularBin(const double & theta)
{
  // 【中文注释】把连续角度量化到最接近的朝向 bin：
  // 注意 Lattice 的朝向角不一定是均匀分布，因此线性扫描取最短角距者
  float min_dist = std::numeric_limits<float>::max();
  unsigned int closest_idx = 0;
  float dist = 0.0;
  for (unsigned int i = 0; i != lattice_metadata.heading_angles.size(); i++) {
    dist = fabs(angles::shortest_angular_distance(theta, lattice_metadata.heading_angles[i]));
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  return closest_idx;
}

float & LatticeMotionTable::getAngleFromBin(const unsigned int & bin_idx)
{
  // 【中文注释】bin 索引转实际朝向角（弧度，来自元数据表）
  return lattice_metadata.heading_angles[bin_idx];
}

// 【中文注释】NodeLattice：状态格(state lattice)节点，结构类似 NodeHybrid
// （SE(2) 位姿 + 静态运动表），区别在于运动基元不是内建的 Dubins/RS 生成，
// 而是从外部 JSON 文件加载的预计算控制采样轨迹，能表达更复杂的运动模型。
NodeLattice::NodeLattice(const unsigned int index)
: parent(nullptr),
  pose(0.0f, 0.0f, 0.0f),
  _cell_cost(std::numeric_limits<float>::quiet_NaN()),
  _accumulated_cost(std::numeric_limits<float>::max()),
  _index(index),
  _was_visited(false),
  _motion_primitive(nullptr),
  _backwards(false)
{
}

NodeLattice::~NodeLattice()
{
  parent = nullptr;
}

void NodeLattice::reset()
{
  parent = nullptr;
  _cell_cost = std::numeric_limits<float>::quiet_NaN();
  _accumulated_cost = std::numeric_limits<float>::max();
  _was_visited = false;
  pose.x = 0.0f;
  pose.y = 0.0f;
  pose.theta = 0.0f;
  _motion_primitive = nullptr;
  _backwards = false;
}

bool NodeLattice::isNodeValid(
  const bool & traverse_unknown,
  GridCollisionChecker * collision_checker,
  MotionPrimitive * motion_primitive,
  bool is_backwards)
{
  // Check primitive end pose
  // Convert grid quantization of primitives to radians, then collision checker quantization
  // 【中文注释】Lattice 的碰撞检查比 Hybrid 更严格：不仅检查基元终点位姿，
  // 还沿基元轨迹每隔至少一个栅格对角线距离(2*res^2 的平方)检查中间位姿。
  // 节点代价取整条基元覆盖的所有采样点中的最大代价值。
  // 注意角度需从 lattice 的量化换算到碰撞检查器的角度 bin。
  static const double bin_size = 2.0 * M_PI / collision_checker->getPrecomputedAngles().size();
  const double & angle = motion_table.getAngleFromBin(this->pose.theta) / bin_size;
  if (collision_checker->inCollision(
      this->pose.x, this->pose.y, angle /*bin in collision checker*/, traverse_unknown))
  {
    return false;
  }

  // Set the cost of a node to the highest cost across the primitive
  // 【中文注释】节点代价 = 基元全程最大代价值（保守策略，保证轨迹整体安全）
  float max_cell_cost = collision_checker->getCost();

  // If valid motion primitives are set, check intermediary poses > 1 cell apart
  if (motion_primitive) {
    const float & grid_resolution = motion_table.lattice_metadata.grid_resolution;
    const float & resolution_diag_sq = 2.0 * grid_resolution * grid_resolution;
    MotionPose last_pose(1e9, 1e9, 1e9), pose_dist(0.0, 0.0, 0.0);

    // Back out the initial node starting point to move motion primitive relative to
    // 【中文注释】基元位姿以米为单位、相对 (0,0) 存储；
    // 用终点位姿反推基元起点，把基元整体平移到当前节点位置
    MotionPose initial_pose, prim_pose;
    initial_pose._x = this->pose.x - (motion_primitive->poses.back()._x / grid_resolution);
    initial_pose._y = this->pose.y - (motion_primitive->poses.back()._y / grid_resolution);
    initial_pose._theta = motion_table.getAngleFromBin(motion_primitive->start_angle);

    for (auto it = motion_primitive->poses.begin(); it != motion_primitive->poses.end(); ++it) {
      // poses are in metric coordinates from (0, 0), not grid space yet
      // 【中文注释】只在离上个检查点超过一个栅格时才检查（用平方比较避免开方）
      pose_dist = *it - last_pose;
      // Avoid square roots by (hypot(x, y) > res) == (x*x+y*y > diag*diag)
      if (pose_dist._x * pose_dist._x + pose_dist._y * pose_dist._y > resolution_diag_sq) {
        last_pose = *it;
        // Convert primitive pose into grid space if it should be checked
        // 【中文注释】把基元中间位姿换算到栅格空间做碰撞检查
        prim_pose._x = initial_pose._x + (it->_x / grid_resolution);
        prim_pose._y = initial_pose._y + (it->_y / grid_resolution);
        // If reversing, invert the angle because the robot is backing into the primitive
        // not driving forward with it
        // 【中文注释】倒车时机器人朝向与基元方向相反：角度加 PI 翻转
        if (is_backwards) {
          prim_pose._theta = std::fmod(it->_theta + M_PI, 2.0 * M_PI);
        } else {
          prim_pose._theta = it->_theta;
        }
        if (collision_checker->inCollision(
            prim_pose._x,
            prim_pose._y,
            prim_pose._theta / bin_size /*bin in collision checker*/,
            traverse_unknown))
        {
          return false;
        }
        max_cell_cost = std::max(max_cell_cost, collision_checker->getCost());
      }
    }
  }

  _cell_cost = max_cell_cost;
  return true;
}

float NodeLattice::getTraversalCost(const NodePtr & child)
{
  // 【中文注释】Lattice 边代价（与 Hybrid 类似但基于基元长度）：
  //   基础项 = 基元轨迹长度(换算成栅格) * (行进奖励 + cost_penalty * 归一化代价)
  //   原地旋转基元(轨迹长度~0)：只收 rotation_penalty；
  //   直线基元(弧长~0)：无附加惩罚；
  //   转弯且左右方向不变：乘 non_straight_penalty；
  //   转弯且换向：乘 non_straight_penalty + change_penalty；
  //   倒车基元：再乘 reverse_penalty。
  const float normalized_cost = child->getCost() / 252.0;
  if (std::isnan(normalized_cost)) {
    throw std::runtime_error(
            "Node attempted to get traversal "
            "cost without a known collision cost!");
  }

  // this is the first node
  // 【中文注释】起点无入边基元：只收基元长度
  MotionPrimitive * prim = this->getMotionPrimitive();
  MotionPrimitive * transition_prim = child->getMotionPrimitive();
  const float prim_length =
    transition_prim->trajectory_length / motion_table.lattice_metadata.grid_resolution;
  if (prim == nullptr) {
    return prim_length;
  }

  // Pure rotation in place 1 angular bin in either direction
  // 【中文注释】原地旋转基元（轨迹长度接近 0）
  if (transition_prim->trajectory_length < 1e-4) {
    return motion_table.rotation_penalty * (1.0 + motion_table.cost_penalty * normalized_cost);
  }

  float travel_cost = 0.0;
  float travel_cost_raw = prim_length *
    (motion_table.travel_distance_reward + motion_table.cost_penalty * normalized_cost);

  if (transition_prim->arc_length < 0.001) {
    // New motion is a straight motion, no additional costs to be applied
    travel_cost = travel_cost_raw;
  } else {
    if (prim->left_turn == transition_prim->left_turn) {
      // Turning motion but keeps in same general direction: encourages to commit to actions
      travel_cost = travel_cost_raw * motion_table.non_straight_penalty;
    } else {
      // Turning motion and velocity directions: penalizes wiggling.
      travel_cost = travel_cost_raw *
        (motion_table.non_straight_penalty + motion_table.change_penalty);
    }
  }

  // If backwards flag is set, this primitive is moving in reverse
  if (child->isBackward()) {
    // reverse direction
    travel_cost *= motion_table.reverse_penalty;
  }

  return travel_cost;
}

float NodeLattice::getHeuristicCost(
  const Coordinates & node_coords,
  const Coordinates & goal_coords,
  const nav2_costmap_2d::Costmap2D * costmap)
{
  // get obstacle heuristic value
  // 【中文注释】启发函数与 Hybrid 相同：max(障碍启发, 距离启发)
  //（复用 NodeHybrid 的静态距离场与距离查找表机制）
  const float obstacle_heuristic = getObstacleHeuristic(
    node_coords, goal_coords, motion_table.cost_penalty);
  const float distance_heuristic =
    getDistanceHeuristic(node_coords, goal_coords, obstacle_heuristic);
  return std::max(obstacle_heuristic, distance_heuristic);
}

void NodeLattice::initMotionModel(
  const MotionModel & motion_model,
  unsigned int & size_x,
  unsigned int & /*size_y*/,
  unsigned int & /*num_angle_quantization*/,
  SearchInfo & search_info)
{
  // 【中文注释】入口：只接受 STATE_LATTICE 模型，委托给运动表初始化（加载 JSON 基元）
  if (motion_model != MotionModel::STATE_LATTICE) {
    throw std::runtime_error(
            "Invalid motion model for Lattice node. Please select"
            " STATE_LATTICE and provide a valid lattice file.");
  }

  motion_table.initMotionModel(size_x, search_info);
}

float NodeLattice::getDistanceHeuristic(
  const Coordinates & node_coords,
  const Coordinates & goal_coords,
  const float & obstacle_heuristic)
{
  // rotate and translate node_coords such that goal_coords relative is (0,0,0)
  // Due to the rounding involved in exact cell increments for caching,
  // this is not an exact replica of a live heuristic, but has bounded error.
  // (Usually less than 1 cell length)

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
  } else if (obstacle_heuristic == 0.0) {
    static ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
    to[0] = goal_coords.x;
    to[1] = goal_coords.y;
    to[2] = motion_table.getAngleFromBin(goal_coords.theta);
    from[0] = node_coords.x;
    from[1] = node_coords.y;
    from[2] = motion_table.getAngleFromBin(node_coords.theta);
    motion_heuristic = motion_table.state_space->distance(from(), to());
  }

  return motion_heuristic;
}

void NodeLattice::precomputeDistanceHeuristic(
  const float & lookup_table_dim,
  const MotionModel & motion_model,
  const unsigned int & dim_3_size,
  const SearchInfo & search_info)
{
  // Dubin or Reeds-Shepp shortest distances
  // 【中文注释】预计算距离启发查找表：与 NodeHybrid::precomputeDistanceHeuristic
  // 机制相同（目标局部窗口内的 Dubins/RS 距离表，利用对称性只存上半平面），
  // 区别是朝向角来自 lattice 元数据（可能非均匀分布）。
  if (!search_info.allow_reverse_expansion) {
    motion_table.state_space = std::make_unique<ompl::base::DubinsStateSpace>(
      search_info.minimum_turning_radius);
  } else {
    motion_table.state_space = std::make_unique<ompl::base::ReedsSheppStateSpace>(
      search_info.minimum_turning_radius);
  }
  motion_table.lattice_metadata =
    LatticeMotionTable::getLatticeMetadata(search_info.lattice_filepath);

  ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
  to[0] = 0.0;
  to[1] = 0.0;
  to[2] = 0.0;
  size_lookup = lookup_table_dim;
  float motion_heuristic = 0.0;
  unsigned int index = 0;
  int dim_3_size_int = static_cast<int>(dim_3_size);

  // Create a lookup table of Dubin/Reeds-Shepp distances in a window around the goal
  // to help drive the search towards admissible approaches. Deu to symmetries in the
  // Heuristic space, we need to only store 2 of the 4 quadrants and simply mirror
  // around the X axis any relative node lookup. This reduces memory overhead and increases
  // the size of a window a platform can store in memory.
  dist_heuristic_lookup_table.resize(size_lookup * ceil(size_lookup / 2.0) * dim_3_size_int);
  for (float x = ceil(-size_lookup / 2.0); x <= floor(size_lookup / 2.0); x += 1.0) {
    for (float y = 0.0; y <= floor(size_lookup / 2.0); y += 1.0) {
      for (int heading = 0; heading != dim_3_size_int; heading++) {
        from[0] = x;
        from[1] = y;
        from[2] = motion_table.getAngleFromBin(heading);
        motion_heuristic = motion_table.state_space->distance(from(), to());
        dist_heuristic_lookup_table[index] = motion_heuristic;
        index++;
      }
    }
  }
}

void NodeLattice::getNeighbors(
  std::function<bool(const unsigned int &, nav2_smac_planner::NodeLattice * &)> & NeighborGetter,
  GridCollisionChecker * collision_checker,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{
  // 【中文注释】Lattice 邻域扩展：遍历当前朝向可用的所有运动基元（含倒车基元）。
  // 与 Hybrid 的关键区别：碰撞检查传入基元本身，会沿基元轨迹逐点检查。
  unsigned int index = 0;
  bool backwards = false;
  NodePtr neighbor = nullptr;
  Coordinates initial_node_coords, motion_projection;
  MotionPrimitivePtrs motion_primitives = motion_table.getMotionPrimitives(this);
  const float & grid_resolution = motion_table.lattice_metadata.grid_resolution;

  // 【中文注释】找到正反基元的分界索引：getMotionPrimitives 先返回正向基元
  // （与当前朝向相同），再追加反向基元（起始朝向相反），分界点在此确定
  unsigned int direction_change_idx = 1e9;
  for (unsigned int i = 0; i != motion_primitives.size(); i++) {
    if (motion_primitives[0]->start_angle != motion_primitives[i]->start_angle) {
      direction_change_idx = i;
      break;
    }
  }

  for (unsigned int i = 0; i != motion_primitives.size(); i++) {
    // 【中文注释】基元终点位姿（米制转栅格）叠加当前节点坐标得到投影
    const MotionPose & end_pose = motion_primitives[i]->poses.back();
    motion_projection.x = this->pose.x + (end_pose._x / grid_resolution);
    motion_projection.y = this->pose.y + (end_pose._y / grid_resolution);
    motion_projection.theta = motion_primitives[i]->end_angle /*this is the ending angular bin*/;

    // if i >= idx, then we're in a reversing primitive. In that situation,
    // the orientation of the robot is mirrored from what it would otherwise
    // appear to be from the motion primitives file. We want to take this into
    // account in case the robot base footprint is asymmetric.
    // 【中文注释】反向基元：机器人车头朝向与基元运动方向相反（差 N/2 个 bin），
    // 需要把朝向镜像 180 度（对非对称足迹很重要）
    backwards = false;
    if (i >= direction_change_idx) {
      backwards = true;
      float opposite_heading_theta =
        motion_projection.theta - (motion_table.num_angle_quantization / 2);
      if (opposite_heading_theta < 0) {
        opposite_heading_theta += motion_table.num_angle_quantization;
      }
      if (opposite_heading_theta > motion_table.num_angle_quantization) {
        opposite_heading_theta -= motion_table.num_angle_quantization;
      }
      motion_projection.theta = opposite_heading_theta;
    }

    index = NodeLattice::getIndex(
      static_cast<unsigned int>(motion_projection.x),
      static_cast<unsigned int>(motion_projection.y),
      static_cast<unsigned int>(motion_projection.theta));

    if (NeighborGetter(index, neighbor) && !neighbor->wasVisited()) {
      // Cache the initial pose in case it was visited but valid
      // don't want to disrupt continuous coordinate expansion
      // 【中文注释】同 Hybrid：临时写连续位姿 -> 带基元做碰撞检查 ->
      // 通过则记录基元和倒车标记，失败则回滚位姿
      initial_node_coords = neighbor->pose;
      neighbor->setPose(
        Coordinates(
          motion_projection.x,
          motion_projection.y,
          motion_projection.theta));

      // Using a special isNodeValid API here, giving the motion primitive to use to
      // validity check the transition of the current node to the new node over
      if (neighbor->isNodeValid(
          traverse_unknown, collision_checker, motion_primitives[i], backwards))
      {
        neighbor->setMotionPrimitive(motion_primitives[i]);
        // Marking if this search was obtained in the reverse direction
        neighbor->backwards(backwards);
        neighbors.push_back(neighbor);
      } else {
        neighbor->setPose(initial_node_coords);
      }
    }
  }
}

bool NodeLattice::backtracePath(CoordinateVector & path)
{
  // 【中文注释】回溯路径：与 2D/Hybrid 不同的是，每个节点不只输出一个点，
  // 而是把该节点所经运动基元的完整密集轨迹点（栅格分辨率级）加入路径，
  // 使输出路径天然连续、无需额外插值。
  if (!this->parent) {
    return false;
  }

  NodePtr current_node = this;

  while (current_node->parent) {
    addNodeToPath(current_node, path);
    current_node = current_node->parent;
  }

  // add start to path
  addNodeToPath(current_node, path);

  return true;
}

void NodeLattice::addNodeToPath(
  NodeLattice::NodePtr current_node,
  NodeLattice::CoordinateVector & path)
{
  // 【中文注释】把单个节点展开为路径点序列：
  // 反推基元起点，逆序（crbegin 从终点向起点）遍历基元中间位姿，
  // 换算到栅格空间并加入路径；倒车时角度加 PI 翻转。
  // 解析扩展节点没有基元，直接输出节点位姿（bin 转弧度）。
  Coordinates initial_pose, prim_pose;
  MotionPrimitive * prim = nullptr;
  const float & grid_resolution = NodeLattice::motion_table.lattice_metadata.grid_resolution;
  prim = current_node->getMotionPrimitive();
  // if motion primitive is valid, then was searched (rather than analytically expanded),
  // include dense path of subpoints making up the primitive at grid resolution
  if (prim) {
    initial_pose.x = current_node->pose.x - (prim->poses.back()._x / grid_resolution);
    initial_pose.y = current_node->pose.y - (prim->poses.back()._y / grid_resolution);
    initial_pose.theta = NodeLattice::motion_table.getAngleFromBin(prim->start_angle);

    for (auto it = prim->poses.crbegin(); it != prim->poses.crend(); ++it) {
      // Convert primitive pose into grid space if it should be checked
      prim_pose.x = initial_pose.x + (it->_x / grid_resolution);
      prim_pose.y = initial_pose.y + (it->_y / grid_resolution);
      // If reversing, invert the angle because the robot is backing into the primitive
      // not driving forward with it
      if (current_node->isBackward()) {
        prim_pose.theta = std::fmod(it->_theta + M_PI, 2.0 * M_PI);
      } else {
        prim_pose.theta = it->_theta;
      }
      path.push_back(prim_pose);
    }
  } else {
    // For analytic expansion nodes where there is no valid motion primitive
    path.push_back(current_node->pose);
    path.back().theta = NodeLattice::motion_table.getAngleFromBin(path.back().theta);
  }
}

}  // namespace nav2_smac_planner
