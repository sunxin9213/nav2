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
#ifndef NAV2_COSTMAP_2D__LAYERED_COSTMAP_HPP_
#define NAV2_COSTMAP_2D__LAYERED_COSTMAP_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_costmap_2d
{
class Layer;

/**
 * @class LayeredCostmap
 * @brief Instantiates different layer plugins and aggregates them into one score
 */
// LayeredCostmap 是整个导航成本地图的核心汇总结构：
// 它不是单一一张二维栅格，而是“多层插件叠加后的最终成本图”。
// 每个 layer 负责一个特定职责，例如静态地图、膨胀层、障碍层、过滤层等；
// 最终 combined_costmap_ 会把这些成本按规则合并，供全局规划和局部规划使用。
class LayeredCostmap
{
public:
  /**
   * @brief  Constructor for a costmap
   */
  // 构造: 指定全局坐标系、是否滚动窗口、是否追踪未知空间（决定默认代价值）
  LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown);

  /**
   * @brief  Destructor
   */
  // 析构: 无需手动释放（全部使用智能指针与值语义成员）
  ~LayeredCostmap();

  /**
   * @brief  Update the underlying costmap with new data.
   * If you want to update the map outside of the update loop that runs, you can call this.
   *
   * 该过程会让每个 layer 根据当前机器人位姿更新它们自己覆盖区域的代价，
   * 之后再将各层的结果合并到 combined_costmap_，保证规划器看到的是最新障碍环境。
   */
  void updateMap(double robot_x, double robot_y, double robot_yaw);

  // 全局坐标系名（如 "map"/"odom"）
  std::string getGlobalFrameID() const
  {
    return global_frame_;
  }

  /**
   * @brief Resize the map to a new size, resolution, or origin
   *
   * 负责重设主代价地图的尺寸和分辨率；
   * 这通常在启动、地图切换或滚动地图模式中发生，确保全局/局部规划器使用同一坐标系。
   */
  void resizeMap(
    unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
    double origin_y,
    bool size_locked = false);

  /**
   * @brief Get the size of the bounds for update
   */
  // 输出最近一次 updateMap 累计的更新包围盒（世界坐标，米）
  void getUpdatedBounds(double & minx, double & miny, double & maxx, double & maxy)
  {
    minx = minx_;
    miny = miny_;
    maxx = maxx_;
    maxy = maxy_;
  }

  /**
   * @brief If the costmap is current, e.g. are all the layers processing recent data
   * and not stale information for a good state.
   */
  // 所有 layer 的数据是否都是"新鲜"的（各层 isCurrent() 均为 true）；
  // obstacle layer 等按观测时间戳判断，过旧返回 false 提示上游等待
  bool isCurrent();

  /**
   * @brief Get the costmap pointer to the master costmap
   */
  // 取合并后的主代价地图（plugins+filters 全部叠加的最终结果）
  Costmap2D * getCostmap()
  {
    return &combined_costmap_;
  }

  /**
   * @brief If this costmap is rolling or not
   */
  // 是否为滚动窗口模式（局部地图随机器人平移原点）
  bool isRolling()
  {
    return rolling_window_;
  }

  /**
   * @brief If this costmap is tracking unknown space or not
   */
  // 是否追踪未知空间：默认代价值为 NO_INFORMATION(255) 即视为开启
  bool isTrackingUnknown()
  {
    return combined_costmap_.getDefaultValue() == nav2_costmap_2d::NO_INFORMATION;
  }

  /**
   * @brief Get the vector of pointers to the costmap plugins
   */
  // 取图层插件列表（static/obstacle/inflation 等，按声明顺序更新）
  std::vector<std::shared_ptr<Layer>> * getPlugins()
  {
    return &plugins_;
  }

  /**
   * @brief Get the vector of pointers to the costmap filters
   */
  // 取过滤器插件列表（keepout/speed 等 costmap filter，在 plugins 之后叠加）
  std::vector<std::shared_ptr<Layer>> * getFilters()
  {
    return &filters_;
  }

  /**
   * @brief Add a new plugin to the plugins vector to process
   */
  // 追加一个图层插件（Costmap2DROS 加载插件后调用）
  void addPlugin(std::shared_ptr<Layer> plugin);


  /**
   * @brief Add a new costmap filter plugin to the filters vector to process
   */
  // 追加一个过滤器插件（Costmap2DROS 加载 filter 后调用）
  void addFilter(std::shared_ptr<Layer> filter);


  /**
   * @brief Get if the size of the costmap is locked
   */
  // 尺寸是否被锁定（static layer 加载地图后锁定，禁止外部 resize 改变尺寸）
  bool isSizeLocked()
  {
    return size_locked_;
  }

  /**
   * @brief Get the bounds of the costmap
   */
  // 输出地图的整体尺寸边界（格数下限/上限，含 bx0_..byn_ 四个成员）
  void getBounds(unsigned int * x0, unsigned int * xn, unsigned int * y0, unsigned int * yn)
  {
    *x0 = bx0_;
    *xn = bxn_;
    *y0 = by0_;
    *yn = byn_;
  }

  /**
   * @brief if the costmap is initialized
   */
  // 地图是否已完成首次更新（第一次 updateMap 后置 true）
  bool isInitialized()
  {
    return initialized_;
  }

  /** @brief Updates the stored footprint, updates the circumscribed
   * and inscribed radii, and calls onFootprintChanged() in all
   * layers. */
  // 更新机器人足迹：重算外接/内切半径，并通知所有 layer
  // （inflation layer 依赖这两个半径重算膨胀参数）
  void setFootprint(const std::vector<geometry_msgs::msg::Point> & footprint_spec);

  /** @brief Returns the latest footprint stored with setFootprint(). */
  // 返回最近一次 setFootprint() 存储的足迹多边形
  const std::vector<geometry_msgs::msg::Point> & getFootprint() {return footprint_;}

  /** @brief The radius of a circle centered at the origin of the
   * robot which just surrounds all points on the robot's
   * footprint.
   *
   * This is updated by setFootprint(). */
  // 外接圆半径: 以机器人为圆心、恰好包住整个足迹的最小圆
  // （膨胀层用它确定"绝对不可进入"的硬边界尺度）
  double getCircumscribedRadius() {return circumscribed_radius_;}

  /** @brief The radius of a circle centered at the origin of the
   * robot which is just within all points and edges of the robot's
   * footprint.
   *
   * This is updated by setFootprint(). */
  // 内切圆半径: 以机器人为圆心、完全落在足迹内部的最大圆
  // （机器人中心距障碍小于该值则必然碰撞，即内切必碰 INSCRIBED）
  double getInscribedRadius() {return inscribed_radius_;}

  /** @brief Checks if the robot is outside the bounds of its costmap in the case
  * of poorly configured setups. */
  // 检查机器人是否在代价地图边界之外（配置错误的兜底检测，
  // 如局部地图过小或 TF 异常导致位姿越界）
  bool isOutofBounds(double robot_x, double robot_y);

private:
  // primary_costmap_ is a bottom costmap used by plugins when costmap filters were enabled.
  // combined_costmap_ is a final costmap where all results produced by plugins and filters (if any)
  // to be merged.
  // The separation is aimed to avoid interferences of work between plugins and filters.
  // primay_costmap_ and combined_costmap_ have the same sizes, origins and default values.
  // 双地图设计（中文补充）:
  //   primary_costmap_  —— plugins（static/obstacle/inflation 等）的合并结果，
  //                        作为 filter 工作的"底图"；
  //   combined_costmap_ —— 最终地图 = primary_costmap_ 再叠加 filters 的结果，
  //                        规划/控制实际查询的就是它（getCostmap 返回值）。
  //   两张图尺寸/原点/默认值完全一致，分离的目的是避免 filter 与
  //   普通 plugin 在写图时互相干扰。
  Costmap2D primary_costmap_, combined_costmap_;
  // 全局坐标系名（"map"/"odom" 等）
  std::string global_frame_;

  // 是否滚动窗口（局部地图）
  bool rolling_window_;  /// < @brief Whether or not the costmap should roll with the robot

  // ---- 状态与边界缓存 ----
  // current_:         所有层数据是否新鲜（isCurrent 的缓存值）
  // minx_/miny_/maxx_/maxy_: 最近一次更新涉及的世界坐标包围盒
  // bx0_..byn_:       地图整体尺寸边界（格数，供 getBounds 输出）
  bool current_;
  double minx_, miny_, maxx_, maxy_;
  unsigned int bx0_, bxn_, by0_, byn_;

  // ---- 插件列表 ----
  // plugins_: 图层插件（按 YAML 声明顺序依次 updateBounds/updateCosts）
  // filters_: 过滤器插件（在 plugins 全部完成后叠加到 combined_costmap_）
  std::vector<std::shared_ptr<Layer>> plugins_;
  std::vector<std::shared_ptr<Layer>> filters_;

  // initialized_: 是否完成首次更新
  // size_locked_: 尺寸锁定标志（static layer 地图加载后置位）
  // circumscribed_radius_ / inscribed_radius_: 足迹外接/内切圆半径
  // footprint_: 机器人足迹多边形（带 padding 后的最终版本）
  bool initialized_;
  bool size_locked_;
  double circumscribed_radius_, inscribed_radius_;
  std::vector<geometry_msgs::msg::Point> footprint_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__LAYERED_COSTMAP_HPP_
