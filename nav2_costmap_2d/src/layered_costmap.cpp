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
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
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
#include "nav2_costmap_2d/layered_costmap.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "nav2_costmap_2d/footprint.hpp"


using std::vector;

namespace nav2_costmap_2d
{

// 构造函数负责初始化两张关键地图：
// - primary_costmap_：插件更新的主地图缓存；
// - combined_costmap_：最终对规划器可见的合并地图；
// 这两者在尺寸、原点和默认值上保持一致，便于插件先写入 primary，再按需复制到 final。
// 如果启用了 track_unknown，则把无信息区域标记为 255（常见的未知栅格值），
// 否则使用 0 作为默认空闲成本，表示可通行区域。
LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown)
: primary_costmap_(), combined_costmap_(),
  global_frame_(global_frame),
  rolling_window_(rolling_window),
  current_(false),
  minx_(0.0),
  miny_(0.0),
  maxx_(0.0),
  maxy_(0.0),
  bx0_(0),
  bxn_(0),
  by0_(0),
  byn_(0),
  initialized_(false),
  size_locked_(false),
  circumscribed_radius_(1.0),
  inscribed_radius_(0.1)
{
  if (track_unknown) {
    primary_costmap_.setDefaultValue(255);
    combined_costmap_.setDefaultValue(255);
  } else {
    primary_costmap_.setDefaultValue(0);
    combined_costmap_.setDefaultValue(0);
  }
}

LayeredCostmap::~LayeredCostmap()
{
  while (plugins_.size() > 0) {
    plugins_.pop_back();
  }
  while (filters_.size() > 0) {
    filters_.pop_back();
  }
}

void LayeredCostmap::addPlugin(std::shared_ptr<Layer> plugin)
{
  // 所有层按顺序追加到 plugins_ 容器中，后续 updateMap() 会按照这个顺序处理。
  // 例如：StaticLayer -> ObstacleLayer -> InflationLayer
  // 这样可以保证底层地图先写入，再由上层继续叠加障碍和代价膨胀。
  std::unique_lock<Costmap2D::mutex_t> lock(*(combined_costmap_.getMutex()));
  plugins_.push_back(plugin);
}

void LayeredCostmap::addFilter(std::shared_ptr<Layer> filter)
{
  // Filter 与 Plugin 的区别在于它们被认为是“后处理”步骤：
  // 插件负责产生基础代价，过滤器则在最终成果上做修正，
  // 让代价地图更符合任务约束、地图裁剪或语义条件。
  std::unique_lock<Costmap2D::mutex_t> lock(*(combined_costmap_.getMutex()));
  filters_.push_back(filter);
}

void LayeredCostmap::resizeMap(
  unsigned int size_x, unsigned int size_y, double resolution,
  double origin_x,
  double origin_y,
  bool size_locked)
{
  // 重设地图尺寸时，必须同时更新两张底层图和所有已经注册的层。
  // 否则插件内部缓存的地图大小、原点和栅格索引将和最终可见地图不一致，
  // 导致规划器在坐标转换时出现错误的越界访问或误判。
  std::unique_lock<Costmap2D::mutex_t> lock(*(combined_costmap_.getMutex()));
  size_locked_ = size_locked;
  primary_costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  combined_costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end(); ++plugin)
  {
    (*plugin)->matchSize();
  }
  for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin();
    filter != filters_.end(); ++filter)
  {
    (*filter)->matchSize();
  }
}

bool LayeredCostmap::isOutofBounds(double robot_x, double robot_y)
{
  unsigned int mx, my;
  return !combined_costmap_.worldToMap(robot_x, robot_y, mx, my);
}

void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  // 这一段是代价地图“更新周期”的核心：
  // 1) 锁住全局代价地图互斥量，避免多个线程同时写入同一张地图；
  // 2) 如果是 rolling window，随着机器人移动改变地图原点，使地图中心始终跟随机器人；
  // 3) 让每个 layer 计算它们感兴趣的最小/最大边界；
  // 4) 对该区域窗口执行插件更新与过滤器更新；
  // 5) 最后把处理结果写回最终的 combined_costmap_。
  std::unique_lock<Costmap2D::mutex_t> lock(*(combined_costmap_.getMutex()));

  // if we're using a rolling buffer costmap...
  // we need to update the origin using the robot's position
  if (rolling_window_) {
    // 在滚动窗口模式下，原点并不固定，而是跟随机器人位置移动，
    // 这样地图“窗口”总是围绕当前机器人，保证局部规划看到的是“以机器人为中心”的感知。
    double new_origin_x = robot_x - combined_costmap_.getSizeInMetersX() / 2;
    double new_origin_y = robot_y - combined_costmap_.getSizeInMetersY() / 2;
    primary_costmap_.updateOrigin(new_origin_x, new_origin_y);
    combined_costmap_.updateOrigin(new_origin_x, new_origin_y);
  }

  if (isOutofBounds(robot_x, robot_y)) {
    RCLCPP_WARN(
      rclcpp::get_logger("nav2_costmap_2d"),
      "Robot is out of bounds of the costmap!");
  }

  if (plugins_.size() == 0 && filters_.size() == 0) {
    return;
  }

  minx_ = miny_ = std::numeric_limits<double>::max();
  maxx_ = maxy_ = std::numeric_limits<double>::lowest();

  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end(); ++plugin)
  {
    // 每个 layer 都可以用 updateBounds() 报告自己实际更新了哪一块区域。
    // 例如静态地图可能只需要整张图；障碍层只关心机器人周围一块感知范围；
    // 估算出的 min/max 界限就是后续局部刷新区域的边界，体现“只更新变化区域”的优化思想。
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy) {
      RCLCPP_WARN(
        rclcpp::get_logger(
          "nav2_costmap_2d"), "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
        prev_minx, prev_miny, prev_maxx, prev_maxy,
        minx_, miny_, maxx_, maxy_,
        (*plugin)->getName().c_str());
    }
  }
  for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin();
    filter != filters_.end(); ++filter)
  {
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    (*filter)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy) {
      RCLCPP_WARN(
        rclcpp::get_logger(
          "nav2_costmap_2d"), "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
        "is now [tl: (%f, %f), br: (%f, %f)]. The offending filter is %s",
        prev_minx, prev_miny, prev_maxx, prev_maxy,
        minx_, miny_, maxx_, maxy_,
        (*filter)->getName().c_str());
    }
  }

  int x0, xn, y0, yn;
  // 把世界坐标的边界转换为栅格索引，并强制保证在地图范围内，
  // 避免 layer 更新时访问到数组外部区域。
  combined_costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
  combined_costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

  x0 = std::max(0, x0);
  xn = std::min(static_cast<int>(combined_costmap_.getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(static_cast<int>(combined_costmap_.getSizeInCellsY()), yn + 1);

  RCLCPP_DEBUG(
    rclcpp::get_logger(
      "nav2_costmap_2d"), "Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);

  if (xn < x0 || yn < y0) {
    return;
  }

  if (filters_.size() == 0) {
    // 如果没有 filter，那么最简单直接的策略是：
    // 先清空 combined_costmap_ 的目标窗口，再按 plugins 顺序依次写入成本。
    combined_costmap_.resetMap(x0, y0, xn, yn);
    for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
      plugin != plugins_.end(); ++plugin)
    {
      (*plugin)->updateCosts(combined_costmap_, x0, y0, xn, yn);
    }
  } else {
    // 如果启用了 filter，说明地图处理路径变成了两阶段流水线：
    // 阶段 1：先让 plugin 在 primary_costmap_ 里生成基础成本；
    // 阶段 2：复制到 combined_costmap_，再让 filter 在最终地图上做后处理。
    // 这样设计的原因是 filter 不能污染 plugin 的基础数据，否则下一轮 updateMap() 会把 filter 的修正再次当作输入，导致叠加误差。
    primary_costmap_.resetMap(x0, y0, xn, yn);
    for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
      plugin != plugins_.end(); ++plugin)
    {
      (*plugin)->updateCosts(primary_costmap_, x0, y0, xn, yn);
    }

    // 2. Copy processed costmap window to a final costmap.
    // primary_costmap_ remain to be untouched for further usage by plugins.
    if (!combined_costmap_.copyWindow(primary_costmap_, x0, y0, xn, yn, x0, y0)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("nav2_costmap_2d"),
        "Can not copy costmap (%i,%i)..(%i,%i) window",
        x0, y0, xn, yn);
      throw std::runtime_error{"Can not copy costmap"};
    }

    // 3. Apply filters over the plugins in order to make filters' work
    // not being considered by plugins on next updateMap() calls
    for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin();
      filter != filters_.end(); ++filter)
    {
      (*filter)->updateCosts(combined_costmap_, x0, y0, xn, yn);
    }
  }

  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;

  initialized_ = true;
}

bool LayeredCostmap::isCurrent()
{
  // 这个函数用于判断所有 layer/filter 是否都处于“有效且最新”的状态。
  // 若某个 layer 被禁用，则跳过它，不让它拖累整个地图状态判断；
  // 若所有启用中的层都认为自己是新鲜数据，则返回 true，说明当前 map 对 planner 来说足够可靠。
  current_ = true;
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end(); ++plugin)
  {
    current_ = current_ && ((*plugin)->isCurrent() || !(*plugin)->isEnabled());
  }
  for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin();
    filter != filters_.end(); ++filter)
  {
    current_ = current_ && ((*filter)->isCurrent() || !(*filter)->isEnabled());
  }
  return current_;
}

void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::msg::Point> & footprint_spec)
{
  footprint_ = footprint_spec;
  nav2_costmap_2d::calculateMinAndMaxDistances(
    footprint_spec,
    inscribed_radius_, circumscribed_radius_);

  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->onFootprintChanged();
  }
  for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin();
    filter != filters_.end();
    ++filter)
  {
    (*filter)->onFootprintChanged();
  }
}

}  // namespace nav2_costmap_2d
