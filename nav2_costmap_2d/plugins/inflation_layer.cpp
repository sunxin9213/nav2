/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
// ============================================================================
// 中文注释：膨胀层（InflationLayer）实现文件
//
// 【文件功能】
// 本文件实现 Nav2 代价地图插件体系中的"膨胀层"。它不产生新的障碍物，
// 而是读取主代价地图（master costmap）中已存在的障碍物（LETHAL_OBSTACLE），
// 并以障碍物为中心向外"膨胀"出一圈按距离衰减的代价值，使规划器/控制器
// 在选择路径时天然倾向于远离障碍物。
//
// 【在 Nav2 中的位置】
// nav2_costmap_2d 是 Nav2 的代价地图核心包，采用 LayeredCostmap 分层架构：
//   StaticLayer(静态地图) / ObstacleLayer(2D障碍) / VoxelLayer(3D体素障碍)
//   等层先在 updateBounds/updateCosts 阶段写入障碍物，最后由 InflationLayer
//   统一膨胀。本层通常作为"最后加工层"放在 layers 列表末尾。
//
// 【核心原理】
// 1. 代价模型（见 cost_values.hpp）：
//    - 距障碍 < 内切半径(inscribed_radius)：INSCRIBED_INFLATED_OBSTACLE(253)，
//      表示机器人中心到此一定碰撞（机器人内切圆压到障碍）。
//    - 距障碍 > 膨胀半径(inflation_radius)：不膨胀，保持原值。
//    - 两者之间：按指数衰减 exp(-cost_scaling_factor * (distance - inscribed_radius))
//      计算，从 253 平滑衰减到 0。
// 2. 算法采用类似 BFS 的多级桶队列（inflation_cells_，map<距离等级, 待处理链表>）
//    从每个障碍格出发向四邻域扩散，用 seen_ 数组防止重复访问，
//    并用预计算的距离/代价查找表（cached_distances_/cached_costs_）加速。
// 3. 通过 pluginlib 宏 PLUGINLIB_EXPORT_CLASS 注册为可动态加载的 Layer 插件，
//    在 YAML 配置中通过 plugin: "nav2_costmap_2d::InflationLayer" 引用。
// ============================================================================
#include "nav2_costmap_2d/inflation_layer.hpp"

#include <limits>
#include <map>
#include <vector>
#include <algorithm>
#include <utility>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_events_filter.hpp"

// pluginlib 导出宏：将 InflationLayer 注册为 Layer 基类插件，
// 运行时由 Costmap2DROS 通过 pluginlib 动态加载（costmap_plugins.xml 中声明）
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::InflationLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

// ============================================================================
// 构造函数：仅做成员初始化，真正的参数读取在 onInitialize() 中完成
// ============================================================================
InflationLayer::InflationLayer()
: inflation_radius_(0),                        // 膨胀半径（米），默认在 onInitialize 中取 0.55
  inscribed_radius_(0),                         // 机器人足迹内切半径（米），由 onFootprintChanged 更新
  cost_scaling_factor_(0),                      // 代价指数衰减因子，越大衰减越快
  inflate_unknown_(false),                      // 是否允许把 NO_INFORMATION(255) 格也写上膨胀代价
  inflate_around_unknown_(false),               // 是否把 NO_INFORMATION 格当作障碍物进行膨胀
  cell_inflation_radius_(0),                    // 以"格"为单位的膨胀半径 = inflation_radius_/resolution
  cached_cell_inflation_radius_(0),             // 上次构建缓存时的格半径，用于判断缓存是否需要重建
  resolution_(0),                               // 地图分辨率（米/格）
  cache_length_(0),                             // 查找表边长（格数）= cell_inflation_radius_ + 2
  last_min_x_(std::numeric_limits<double>::lowest()),
  last_min_y_(std::numeric_limits<double>::lowest()),
  last_max_x_(std::numeric_limits<double>::max()),
  last_max_y_(std::numeric_limits<double>::max())
{
  // 创建互斥锁，保护本层内部数据（缓存、膨胀队列等）的线程安全访问
  access_ = new mutex_t();
}

// 析构函数：释放动态参数回调句柄与互斥锁
InflationLayer::~InflationLayer()
{
  dyn_params_handler_.reset();
  delete access_;
}

// ============================================================================
// onInitialize()：插件初始化（由 Costmap2DROS::on_configure 阶段调用）
// 完成参数声明/读取、注册动态参数回调、构建查找表
// ============================================================================
void
InflationLayer::onInitialize()
{
  // ---- 参数声明（declare_parameter）----
  // ROS2 要求参数必须先声明后使用；带命名空间前缀时实际参数名为
  // "<层名>.<参数名>"，例如 "inflation_layer.inflation_radius"
  declareParameter("enabled", rclcpp::ParameterValue(true));                       // 是否启用本层
  declareParameter("inflation_radius", rclcpp::ParameterValue(0.55));              // 膨胀半径（米）
  declareParameter("cost_scaling_factor", rclcpp::ParameterValue(10.0));           // 指数衰减系数
  declareParameter("inflate_unknown", rclcpp::ParameterValue(false));              // 是否对未知区域赋膨胀代价
  declareParameter("inflate_around_unknown", rclcpp::ParameterValue(false));       // 是否围绕未知区域膨胀

  {
    // node_ 是 weak_ptr，这里 lock() 提升为 shared_ptr 以访问生命周期节点
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }
    // ---- 参数读取（get_parameter）：注意参数名带层名前缀 ----
    node->get_parameter(name_ + "." + "enabled", enabled_);
    node->get_parameter(name_ + "." + "inflation_radius", inflation_radius_);
    node->get_parameter(name_ + "." + "cost_scaling_factor", cost_scaling_factor_);
    node->get_parameter(name_ + "." + "inflate_unknown", inflate_unknown_);
    node->get_parameter(name_ + "." + "inflate_around_unknown", inflate_around_unknown_);

    // 注册动态参数回调：运行时通过 ros2 param set 修改参数会触发
    // dynamicParametersCallback()，无需重启即可调整膨胀行为
    dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(
        &InflationLayer::dynamicParametersCallback,
        this, std::placeholders::_1));
  }

  current_ = true;                 // 标记本层数据为最新（膨胀层总是可用）
  seen_.clear();                   // 清空访问标记表
  cached_distances_.clear();       // 清空距离查找表
  cached_costs_.clear();           // 清空代价查找表
  need_reinflation_ = false;       // 重置"需要全图重新膨胀"标志
  cell_inflation_radius_ = cellDistance(inflation_radius_);  // 米 → 格换算
  matchSize();                     // 按主地图尺寸构建缓存
}

// ============================================================================
// matchSize()：与主代价地图尺寸对齐，重建距离/代价查找表
// 当主地图尺寸/分辨率变化或膨胀参数变化时被调用
// ============================================================================
void
InflationLayer::matchSize()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  // 获取主代价地图指针（LayeredCostmap 持有的 Costmap2D）
  nav2_costmap_2d::Costmap2D * costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();                        // 分辨率（米/格）
  cell_inflation_radius_ = cellDistance(inflation_radius_);      // 膨胀半径换算为格数
  computeCaches();                                               // 构建/更新查找表
  // seen_ 与主地图等大（size_x*size_y 个 bool），用于膨胀过程中的访问去重
  seen_ = std::vector<bool>(costmap->getSizeInCellsX() * costmap->getSizeInCellsY(), false);
}

// ============================================================================
// updateBounds()：分层更新第一阶段——计算本层需要更新的世界坐标边界
// 膨胀层的特殊之处：它不产生新障碍，但会把边界向外扩 inflation_radius_，
// 因为界外膨胀源会影响界内格子的代价值
// ============================================================================
void
InflationLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (need_reinflation_) {
    // 情况一：参数变化后需要全图重新膨胀——直接把边界设为无穷大，
    // 使 updateCosts 覆盖整张地图
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = std::numeric_limits<double>::lowest();
    *min_y = std::numeric_limits<double>::lowest();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();
    need_reinflation_ = false;
  } else {
    // 情况二：常规周期——取上一轮与本轮边界的并集，再向外扩一个膨胀半径。
    // 这样保证上一轮新写入的障碍（可能在旧边界外）也能被膨胀到
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
  }
}

// ============================================================================
// onFootprintChanged()：机器人足迹变化时更新内切半径并重建缓存
// 足迹变化 → 内切半径变化 → 代价衰减曲线起点变化，因此必须重算
// ============================================================================
void
InflationLayer::onFootprintChanged()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  inscribed_radius_ = layered_costmap_->getInscribedRadius();   // 新的内切半径
  cell_inflation_radius_ = cellDistance(inflation_radius_);      // 重算格半径
  computeCaches();                                               // 重建代价查找表
  need_reinflation_ = true;                                      // 标记需要全图重膨胀

  // 安全检查：膨胀半径小于内切半径时，机器人内切圆区域可能没有被
  // 正确标记为 INSCRIBED_INFLATED_OBSTACLE，存在碰撞风险
  if (inflation_radius_ < inscribed_radius_) {
    RCLCPP_ERROR(
      logger_,
      "The configured inflation radius (%.3f) is smaller than "
      "the computed inscribed radius (%.3f) of your footprint, "
      "it is highly recommended to set inflation radius to be at "
      "least as big as the inscribed radius to avoid collisions",
      inflation_radius_, inscribed_radius_);
  }

  RCLCPP_DEBUG(
    logger_, "InflationLayer::onFootprintChanged(): num footprint points: %zu,"
    " inscribed_radius_ = %.3f, inflation_radius_ = %.3f",
    layered_costmap_->getFootprint().size(), inscribed_radius_, inflation_radius_);
}

// ============================================================================
// updateCosts()：分层更新第二阶段——在给定栅格范围内执行膨胀，写入主地图
// 算法：多级桶 BFS（Cascaded Digital Bellman-Ford 风格）
//   1) 扫描范围内的所有致命障碍格（距离=0）作为种子；
//   2) 按"整数距离等级"从小到大逐桶处理，每个格子查表得到代价并写入主地图；
//   3) 处理时把四邻域格子按其到障碍的距离放入对应桶，实现按距离有序扩散
// ============================================================================
void
InflationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  // 层被禁用或膨胀半径为 0 时直接跳过
  if (!enabled_ || (cell_inflation_radius_ == 0)) {
    return;
  }

  // make sure the inflation list is empty at the beginning of the cycle (should always be true)
  // 健全性检查：上一轮膨胀队列理应已清空，否则说明状态被破坏（致命错误）
  for (auto & dist : inflation_cells_) {
    RCLCPP_FATAL_EXPRESSION(
      logger_,
      !dist.empty(), "The inflation list must be empty at the beginning of inflation");
  }

  // 获取主地图底层 unsigned char 数组及尺寸——
  // costmap 内存布局：一维数组，index = mx + my * size_x
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // seen_ 尺寸与主地图不一致（地图尺寸刚变化）则重建
  if (seen_.size() != size_x * size_y) {
    RCLCPP_WARN(
      logger_, "InflationLayer::updateCosts(): seen_ vector size is wrong");
    seen_ = std::vector<bool>(size_x * size_y, false);
  }

  // 每轮膨胀前清零访问标记表
  std::fill(begin(seen_), end(seen_), false);

  // We need to include in the inflation cells outside the bounding
  // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
  // up to that distance outside the box can still influence the costs
  // stored in cells inside the box.
  // 保存原始边界（base_*）：界外的障碍也要作为膨胀种子，
  // 但只有界内格子的代价才会被真正写回主地图
  const int base_min_i = min_i;
  const int base_min_j = min_j;
  const int base_max_i = max_i;
  const int base_max_j = max_j;
  // 扫描窗口向外扩 cell_inflation_radius_ 格
  min_i -= static_cast<int>(cell_inflation_radius_);
  min_j -= static_cast<int>(cell_inflation_radius_);
  max_i += static_cast<int>(cell_inflation_radius_);
  max_j += static_cast<int>(cell_inflation_radius_);

  // 裁剪到地图范围内，防止数组越界
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // Inflation list; we append cells to visit in a list associated with
  // its distance to the nearest obstacle
  // We use a map<distance, list> to emulate the priority queue used before,
  // with a notable performance boost
  // 膨胀队列：inflation_cells_ 是 map<整数距离等级, CellData链表>，
  // 用"分级桶"模拟优先队列，比真正的 priority_queue 快得多

  // Start with lethal obstacles: by definition distance is 0.0
  // 第一桶（距离 0）：扫描窗口内所有致命障碍格（及可选的未知格）作为种子
  auto & obs_bin = inflation_cells_[0];
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = static_cast<int>(master_grid.getIndex(i, j));   // i,j → 一维下标
      unsigned char cost = master_array[index];
      // LETHAL_OBSTACLE(254) 是种子；若开启 inflate_around_unknown_，
      // NO_INFORMATION(255) 也当作障碍种子
      if (cost == LETHAL_OBSTACLE || (inflate_around_unknown_ && cost == NO_INFORMATION)) {
        obs_bin.emplace_back(index, i, j, i, j);   // 种子格：源即自身
      }
    }
  }

  // Process cells by increasing distance; new cells are appended to the
  // corresponding distance bin, so they
  // can overtake previously inserted but farther away cells
  // 按距离等级从小到大逐桶处理——保证先处理离障碍近的格子
  for (const auto & dist_bin : inflation_cells_) {
    for (std::size_t i = 0; i < dist_bin.size(); ++i) {
      // Do not use iterator or for-range based loops to
      // iterate though dist_bin, since it's size might
      // change when a new cell is enqueued, invalidating all iterators
      // 注意：必须用下标循环，处理过程中会向桶内追加元素，会使迭代器失效
      unsigned int index = dist_bin[i].index_;

      // ignore if already visited
      // 已访问过（已被更近的障碍覆盖）则跳过——保证每个格子只取最近障碍的距离
      if (seen_[index]) {
        continue;
      }

      seen_[index] = true;    // 标记为已访问

      // 取出该格坐标 (mx,my) 与其膨胀源障碍坐标 (sx,sy)
      unsigned int mx = dist_bin[i].x_;
      unsigned int my = dist_bin[i].y_;
      unsigned int sx = dist_bin[i].src_x_;
      unsigned int sy = dist_bin[i].src_y_;

      // assign the cost associated with the distance from an obstacle to the cell
      // 查代价表得到该格相对源障碍的膨胀代价（考虑 (mx-sx, my-sy) 的偏移）
      unsigned char cost = costLookup(mx, my, sx, sy);
      unsigned char old_cost = master_array[index];
      // In order to avoid artifacts appeared out of boundary areas
      // when some layer is going after inflation_layer,
      // we need to apply inflation_layer only to inside of given bounds
      // 只写回原始边界（base_*）内的格子，避免把膨胀结果写到
      // 本轮更新区域之外而产生边界伪影
      if (static_cast<int>(mx) >= base_min_i &&
        static_cast<int>(my) >= base_min_j &&
        static_cast<int>(mx) < base_max_i &&
        static_cast<int>(my) < base_max_j)
      {
        // 未知格的处理策略：
        //  - inflate_unknown_=true：只要膨胀代价 > FREE_SPACE(0) 就覆盖未知；
        //  - inflate_unknown_=false：只有代价 >= INSCRIBED_INFLATED_OBSTACLE(253)
        //    （即必碰区域）才覆盖未知
        if (old_cost == NO_INFORMATION &&
          (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
        {
          master_array[index] = cost;
        } else {
          // 普通格取新旧代价的较大值——保留更危险的信息（只增不减）
          master_array[index] = std::max(old_cost, cost);
        }
      }

      // attempt to put the neighbors of the current cell onto the inflation list
      // 把当前格的四个邻域格压入膨胀队列（注意一维数组中上下邻居差 size_x）
      if (mx > 0) {
        enqueue(index - 1, mx - 1, my, sx, sy);                  // 左邻：index-1
      }
      if (my > 0) {
        enqueue(index - size_x, mx, my - 1, sx, sy);             // 下邻：index-size_x
      }
      if (mx < size_x - 1) {
        enqueue(index + 1, mx + 1, my, sx, sy);                  // 右邻：index+1
      }
      if (my < size_y - 1) {
        enqueue(index + size_x, mx, my + 1, sx, sy);             // 上邻：index+size_x
      }
    }
  }

  // 本轮结束：清空所有桶并预留空间，供下一周期复用（避免反复分配内存）
  for (auto & dist : inflation_cells_) {
    dist.clear();
    dist.reserve(200);
  }

  current_ = true;   // 标记本层输出为最新
}

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point inflation started at
 * @param  src_y The y index of the obstacle point inflation started at
 */
// 中文注释：把邻域格压入膨胀队列。
// 先查距离表得到该格到源障碍的距离；超出膨胀半径则丢弃；
// 否则按整数距离等级放入对应的桶中等待处理。
void
InflationLayer::enqueue(
  unsigned int index, unsigned int mx, unsigned int my,
  unsigned int src_x, unsigned int src_y)
{
  if (!seen_[index]) {
    // we compute our distance table one cell further than the
    // inflation radius dictates so we can make the check below
    // 查预计算距离表（表比膨胀半径多算 2 格，便于下面的越界判断）
    double distance = distanceLookup(mx, my, src_x, src_y);

    // we only want to put the cell in the list if it is within
    // the inflation radius of the obstacle point
    // 超出膨胀半径的格子不再入队——膨胀的截止条件
    if (distance > cell_inflation_radius_) {
      return;
    }

    const unsigned int r = cell_inflation_radius_ + 2;

    // push the cell data onto the inflation list and mark
    // 通过整数距离矩阵 distance_matrix_ 找到该偏移对应的距离等级（桶号），
    // 把 CellData(index,坐标,源坐标) 追加到该桶
    inflation_cells_[distance_matrix_[mx - src_x + r][my - src_y + r]].emplace_back(
      index, mx, my, src_x, src_y);
  }
}

// ============================================================================
// computeCaches()：构建距离/代价两张查找表
// cached_distances_[i][j] = 格偏移 (i,j) 的欧氏距离
// cached_costs_[i][j]     = 该距离对应的膨胀代价值
// 直接查表避免在膨胀主循环中反复调用 exp()，大幅提升性能
// ============================================================================
void
InflationLayer::computeCaches()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (cell_inflation_radius_ == 0) {
    return;
  }

  cache_length_ = cell_inflation_radius_ + 2;   // 表边长：比膨胀半径大 2 格

  // based on the inflation radius... compute distance and cost caches
  // 仅当格半径变化时才重建距离表（欧氏距离与衰减因子无关）
  if (cell_inflation_radius_ != cached_cell_inflation_radius_) {
    cached_costs_.resize(cache_length_ * cache_length_);
    cached_distances_.resize(cache_length_ * cache_length_);

    // 距离表：cached_distances_[i*cache_length_+j] = hypot(i,j)
    // 即以表中心为原点、偏移 (i,j) 的欧氏距离
    for (unsigned int i = 0; i < cache_length_; ++i) {
      for (unsigned int j = 0; j < cache_length_; ++j) {
        cached_distances_[i * cache_length_ + j] = hypot(i, j);
      }
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  // 代价表：每次都要重算，因为它依赖 cost_scaling_factor_（可动态修改）
  for (unsigned int i = 0; i < cache_length_; ++i) {
    for (unsigned int j = 0; j < cache_length_; ++j) {
      cached_costs_[i * cache_length_ + j] = computeCost(cached_distances_[i * cache_length_ + j]);
    }
  }

  // 生成整数距离矩阵并按最大等级数重建膨胀桶
  int max_dist = generateIntegerDistances();
  inflation_cells_.clear();
  inflation_cells_.resize(max_dist + 1);
  for (auto & dist : inflation_cells_) {
    dist.reserve(200);   // 预留空间减少后续 push 的再分配
  }
}

// ============================================================================
// generateIntegerDistances()：生成整数距离矩阵 distance_matrix_
// 把圆形范围内每个偏移 (x,y) 映射到一个"整数距离等级"（按到原点的
// 欧氏距离平方排序后分层编号），供 enqueue() 快速确定桶号。
// 返回最大等级数。
// ============================================================================
int
InflationLayer::generateIntegerDistances()
{
  const int r = cell_inflation_radius_ + 2;   // 半径（含缓冲）
  const int size = r * 2 + 1;                 // 矩阵边长

  std::vector<std::pair<int, int>> points;

  // 收集圆形范围内（含边界）的所有整数偏移点
  for (int y = -r; y <= r; y++) {
    for (int x = -r; x <= r; x++) {
      if (x * x + y * y <= r * r) {
        points.emplace_back(x, y);
      }
    }
  }

  // 按到原点距离平方升序排序——距离相同的点将获得相同等级
  std::sort(
    points.begin(), points.end(),
    [](const std::pair<int, int> & a, const std::pair<int, int> & b) -> bool {
      return a.first * a.first + a.second * a.second < b.first * b.first + b.second * b.second;
    }
  );

  // 距离平方每增大一次，等级 level 加一；写入矩阵（平移 +r 使下标非负）
  std::vector<std::vector<int>> distance_matrix(size, std::vector<int>(size, 0));
  std::pair<int, int> last = {0, 0};
  int level = 0;
  for (auto const & p : points) {
    if (p.first * p.first + p.second * p.second !=
      last.first * last.first + last.second * last.second)
    {
      level++;
    }
    distance_matrix[p.first + r][p.second + r] = level;
    last = p;
  }

  distance_matrix_ = distance_matrix;
  return level;   // 返回最大等级（即桶的总数-1）
}

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
// 中文注释：动态参数回调。运行中修改 inflation_radius / cost_scaling_factor /
// enabled / inflate_unknown / inflate_around_unknown 时触发，
// 更新成员变量并标记需要重新膨胀/重建缓存。
rcl_interfaces::msg::SetParametersResult
InflationLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  bool need_cache_recompute = false;

  // 遍历本次事件中所有被修改的参数，按类型和名称分发处理
  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      // 膨胀半径变化：需要重新膨胀 + 重建缓存（格半径变了）
      if (param_name == name_ + "." + "inflation_radius" &&
        inflation_radius_ != parameter.as_double())
      {
        inflation_radius_ = parameter.as_double();
        need_reinflation_ = true;
        need_cache_recompute = true;
      } else if (param_name == name_ + "." + "cost_scaling_factor" && // NOLINT
        cost_scaling_factor_ != parameter.as_double())
      {
        // 衰减因子变化：需要重新膨胀 + 重算代价表（表内容变了）
        cost_scaling_factor_ = parameter.as_double();
        need_reinflation_ = true;
        need_cache_recompute = true;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      // 启用开关变化：标记不最新，等待重新激活流程
      if (param_name == name_ + "." + "enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        need_reinflation_ = true;
        current_ = false;
      } else if (param_name == name_ + "." + "inflate_unknown" && // NOLINT
        inflate_unknown_ != parameter.as_bool())
      {
        inflate_unknown_ = parameter.as_bool();
        need_reinflation_ = true;
      } else if (param_name == name_ + "." + "inflate_around_unknown" && // NOLINT
        inflate_around_unknown_ != parameter.as_bool())
      {
        inflate_around_unknown_ = parameter.as_bool();
        need_reinflation_ = true;
      }
    }
  }

  // 半径或衰减因子变化时重建查找表
  if (need_cache_recompute) {
    matchSize();
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_costmap_2d
