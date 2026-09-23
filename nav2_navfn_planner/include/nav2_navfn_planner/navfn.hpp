// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//
// Navigation function computation
// Uses Dijkstra's method
// Modified for Euclidean-distance computation
//
// 【中文注释】本文件声明 NavFn 类：Nav2 中 NavFn 全局规划器的核心算法实现。
// 算法类型：基于 Dijkstra / A* 的"导航函数"(Navigation Function) 计算，
// 本质是在栅格代价地图上构建一个从目标点向外扩散的"势场"(potential field)，
// 之后从起点沿势场梯度下降方向回溯得到全局路径。
// 势场扩散支持两种模式：
//   1) Dijkstra（广度优先，无启发式，计算完整势场）；
//   2) A*（最佳优先，带欧氏距离启发式，只扩展必要区域，更快）。
// 坐标约定：原点在左上角，x 向右，y 向下（图像像素坐标系）。
//

#ifndef NAV2_NAVFN_PLANNER__NAVFN_HPP_
#define NAV2_NAVFN_PLANNER__NAVFN_HPP_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

namespace nav2_navfn_planner
{

// cost defs
// 【中文注释】代价地图的 ROS 标准取值约定（0~255）：
#define COST_UNKNOWN_ROS 255  // 255 is unknown cost // 255 表示未知区域
#define COST_OBS 254  // 254 for forbidden regions // 254 表示绝对禁行区（ lethal obstacle）
#define COST_OBS_ROS 253  // ROS values of 253 are obstacles // 253 表示 ROS 代价地图中的致命障碍（含内切膨胀障碍）

// navfn cost values are set to
// COST_NEUTRAL + COST_FACTOR * costmap_cost_value.
// Incoming costmap cost values are in the range 0 to 252.
// With COST_NEUTRAL of 50, the COST_FACTOR needs to be about 0.8 to
// ensure the input values are spread evenly over the output range, 50
// to 253.  If COST_FACTOR is higher, cost values will have a plateau
// around obstacles and the planner will then treat (for example) the
// whole width of a narrow hallway as equally undesirable and thus
// will not plan paths down the center.
// 【中文注释】NavFn 内部代价值的换算公式：cost = COST_NEUTRAL + COST_FACTOR * 原始代价。
// 输入 costmap 代价范围为 0~252。COST_NEUTRAL=50 代表"自由空间"的基准代价，
// COST_FACTOR=0.8 保证输入值均匀映射到 50~253 的输出区间。
// 如果 COST_FACTOR 取得过大，障碍附近的代价会出现"平台化"（多格代价相同），
// 导致规划器把窄走廊整个宽度视为同等糟糕，从而不会沿走廊中线规划——
// 这两个宏是调 NavFn 行为(离障碍远近)的关键参数。

#define COST_NEUTRAL 50  // Set this to "open space" value // 自由空间的基准代价（势场传播的最低单步代价）
#define COST_FACTOR 0.8  // Used for translating costs in NavFn::setCostmap() // 代价缩放系数

// Define the cost type in the case that it is not set. However, this allows
// clients to modify it without changing the file. Arguably, it is better to require it to
// be defined by a user explicitly
#ifndef COSTTYPE
#define COSTTYPE unsigned char  // Whatever is used... // 代价数组的元素类型，默认 unsigned char（与 costmap 一致）
#endif

// potential defs
#define POT_HIGH 1.0e10  // unassigned cell potential // 未被势场波及的栅格的"无穷大"势值哨兵

// priority buffers
// 【中文注释】优先级块缓冲区大小。NavFn 用"块优先级"(block priority) 代替标准堆：
// 待更新栅格按势值区间被分到 cur/next/over 三个块中，避免优先队列的 log(n) 开销。
#define PRIORITYBUFSIZE 10000

/**
  Navigation function call.
  \param costmap Cost map array, of type COSTTYPE; origin is upper left
    NOTE: will be modified to have a border of obstacle costs
  \param nx Width of map in cells
  \param ny Height of map in cells
  \param goal X,Y position of goal cell
  \param start X,Y position of start cell

Returns length of plan if found, and fills an array with x,y interpolated
positions at about 1/2 cell resolution; else returns 0.
*/
// 【中文注释】独立的 C 风格接口：给定代价地图和起终点，用 A* 一次性算出路径。
// costmap 会被原地修改（四周加障碍边框）。返回找到的路径长度，路径点以约半个栅格
// 分辨率插值写入 plan 数组（x,y 交替存放）；找不到路径返回 0。
int create_nav_plan_astar(
  const COSTTYPE * costmap, int nx, int ny,
  int * goal, int * start,
  float * plan, int nplan);

/**
 * @class NavFn
 * @brief Navigation function class. Holds buffers for costmap, navfn map. Maps are pixel-based.
 *  Origin is upper left, x is right, y is down.
 */
// 【中文注释】NavFn 类：持有 costmap 代价数组和导航函数(potential)数组的所有内部缓冲区。
// 地图按像素(栅格)组织：nx 列 * ny 行，栅格一维索引 k = x + y * nx。
class NavFn
{
public:
  /**
   * @brief  Constructs the planner
   * @param nx The x size of the map
   * @param ny The y size of the map
   */
  NavFn(int nx, int ny);

  ~NavFn();

  /**
   * @brief  Sets or resets the size of the map
   * @param nx The x size of the map
   * @param ny The y size of the map
   */
  void setNavArr(int nx, int ny);
  int nx, ny, ns;  /**< size of grid, in pixels */

  /**
   * @brief  Set up the cost array for the planner, usually from ROS
   * @param cmap The costmap
   * @param isROS Whether or not the costmap is coming in in ROS format
   * @param allow_unknown Whether or not the planner should be allowed to plan through
   *   unknown space
   */
  // 【中文注释】导入外部代价地图并换算为 NavFn 内部代价：
  // isROS=true 时按 ROS costmap 约定做 COST_NEUTRAL + COST_FACTOR*v 映射；
  // allow_unknown 决定未知区域(255)是视为"接近障碍的可通行"还是完全禁行。
  void setCostmap(const COSTTYPE * cmap, bool isROS = true, bool allow_unknown = true);

  /**
   * @brief  Calculates a plan using the A* heuristic, returns true if one is found
   * @return True if a plan is found, false otherwise
   */
  bool calcNavFnAstar();

  /**
   * @brief Caclulates the full navigation function using Dijkstra
   */
  bool calcNavFnDijkstra(bool atStart = false);

  /**
   * @brief  Accessor for the x-coordinates of a path
   * @return The x-coordinates of a path
   */
  float * getPathX();

  /**
   * @brief  Accessor for the y-coordinates of a path
   * @return The y-coordinates of a path
   */
  float * getPathY();

  /**
   * @brief  Accessor for the length of a path
   * @return The length of a path, 0 if not found
   */
  int getPathLen();

  /**
   * @brief  Gets the cost of the path found the last time a navigation function was computed
   * @return The cost of the last path found
   */
  float getLastPathCost();

  /** cell arrays */
  // 【中文注释】核心数据结构（均为长度 ns = nx*ny 的一维数组，索引 k = x + y*nx）：
  COSTTYPE * costarr;  /**< cost array in 2D configuration space */
  // 代价数组：由外部 costmap 经 setCostmap() 换算而来（COST_NEUTRAL + COST_FACTOR*v），
  // 值越大表示越难通行，>= COST_OBS 视为障碍，势场不会向其中传播。
  float * potarr;  /**< potential array, navigation function potential */
  // 势值数组（导航函数）：potarr[k] 表示"从栅格 k 到达目标点的累积代价"。
  // 目标点势值为 0，离目标越远势值越大，未初始化为 POT_HIGH(1e10)。
  // 回溯路径时从起点沿 potarr 下降方向走即可到达目标。
  bool * pending;  /**< pending cells during propagation */
  // 待处理标记：栅格已进入某个优先级块、尚未被 updateCell 处理时为 true，
  // 用于防止同一栅格被重复压入优先级块。
  int nobs;  /**< number of obstacle cells */ // 障碍栅格总数（setupNavFn 时统计，用于计算访问率）

  /** block priority buffers */
  // 【中文注释】块优先级缓冲：NavFn 用三块循环使用的数组替代标准二叉堆。
  // 栅格按"势值是否落在当前阈值区间 [curT, curT+priInc)"被放入 cur 或 next，
  // 势值超出的放入 over（溢出块），是一种近似排序的波前(band)传播机制。
  int * pb1, * pb2, * pb3;  /**< storage buffers for priority blocks */ // 三个底层存储块
  int * curP, * nextP, * overP;  /**< priority buffer block ptrs */ // 当前/下一/溢出块的当前指针（三者轮换指向 pb1~pb3）
  int curPe, nextPe, overPe;  /**< end points of arrays */ // 各块中的元素个数（写指针位置）

  /** block priority thresholds */
  float curT;  /**< current threshold */ // 当前优先级阈值：势值低于它的栅格属于"当前波前"
  float priInc;  /**< priority threshold increment */ // 阈值增量：当前块耗尽后 curT += priInc 升级到下一优先级
  /** goal and start positions */
  /**
   * @brief  Sets the goal position for the planner.
   * Note: the navigation cost field computed gives the cost to get to a given point
   * from the goal, not from the start.
   * @param goal the goal position
   */
  void setGoal(int * goal);

  /**
   * @brief  Sets the start position for the planner.
   * Note: the navigation cost field computed gives the cost to get to a given point
   * from the goal, not from the start.
   * @param start the start position
   */
  void setStart(int * start);

  int goal[2];
  int start[2];
  /**
   * @brief  Initialize cell k with cost v for propagation
   * @param k the cell to initialize
   * @param v the cost to give to the cell
   */
  void initCost(int k, float v);

  /** propagation */

  /**
   * @brief  Updates the cell at index n
   * @param n The index to update
   */
  // 【中文注释】Dijkstra 模式的栅格更新：取 4 邻域中两个最低势值邻居，
  // 用"平面波更新"(planar wave update) + 二次插值近似计算本格新势值；
  // 若新势值更小则写入，并把可能受影响的邻居压入优先级块。
  void updateCell(int n);

  /**
   * @brief  Updates the cell at index n using the A* heuristic
   * @param n The index to update
   */
  // 【中文注释】A* 模式的栅格更新：与 updateCell 相同的势值计算，
  // 区别在于把"势值 + 到起点的欧氏距离*COST_NEUTRAL"作为排序键压入优先级块，
  // 使波前优先朝起点方向扩展，从而减少扩展的栅格总数（启发式加速）。
  void updateCellAstar(int n);
  /**
   * @brief  Set up navigation potential arrays for new propagation
   * @param keepit whether or not use COST_NEUTRAL
   */
  // 【中文注释】传播前的初始化：所有势值重置为 POT_HIGH、梯度清零、
  // costmap 四周加障碍边框（防止越界）、重置优先级块和 pending 标记、
  // 以目标点势值 0 作为波源（initCost），并统计障碍栅格数。
  // keepit=true 时保留 costarr（不清成 COST_NEUTRAL）。
  void setupNavFn(bool keepit = false);

  /**
   * @brief  Run propagation for <cycles> iterations, or until start is reached using
   * breadth-first Dijkstra method
   * @param cycles The maximum number of iterations to run for
   * @param atStart Whether or not to stop when the start point is reached
   * @return true if the start point is reached
   */
  // 【中文注释】Dijkstra 主传播循环：逐轮处理当前优先级块中的栅格，
  // 块间轮换(cur<->next)，块耗尽时提升阈值 curT 并启用溢出块 over，
  // 直到起点栅格势值被更新（atStart=true 时提前终止）或达到 cycles 上限。
  bool propNavFnDijkstra(int cycles, bool atStart = false);

  /**
   * @brief  Run propagation for <cycles> iterations, or until start is reached using
   * the best-first A* method with Euclidean distance heuristic
   * @param cycles The maximum number of iterations to run for
   * @return true if the start point is reached
   */
  // 【中文注释】A* 主传播循环：初始阈值加入起终点欧氏距离（启发式下界），
  // 配合 updateCellAstar 的距离排序键实现最佳优先扩展，起点被波及时即成功。
  bool propNavFnAstar(int cycles);  /**< returns true if start point found */

  /** gradient and paths */
  // 【中文注释】梯度与路径相关数据：
  float * gradx, * grady;  /**< gradient arrays, size of potential array */
  // 梯度数组：gradCell() 基于 4 邻域势值差计算归一化梯度，指向势值下降最快方向（即指向目标）。
  float * pathx, * pathy;  /**< path points, as subpixel cell coordinates */
  // 路径点数组：以"亚像素/子栅格坐标"存储（含小数偏移 dx/dy），比整栅格路径更平滑。
  int npath;  /**< number of path points */ // 实际路径点个数
  int npathbuf;  /**< size of pathx, pathy buffers */ // 路径缓冲容量（按需扩容）

  float last_path_cost_;  /**< Holds the cost of the path found the last time A* was called */
  // 上一次 A* 搜索结束时起点栅格的势值，即整条路径的累积代价。

  /**
   * @brief  Calculates the path for at mose <n> cycles
   * @param n The maximum number of cycles to run for
   * @return The lenght of the path found, 0 if none
   */
  // 【中文注释】路径回溯核心：在已计算好的势场上，从起点(或指定 st)出发，
  // 以 pathStep 步长沿负梯度方向迭代前进，做双线性插值得到亚栅格路径点，
  // 直到接近目标或达到 n 次迭代上限。返回路径点数；0 表示失败。
  int calcPath(int n, int * st = NULL);

  /**
   * @brief  Calculate gradient at a cell
   * @param n Cell number <n>
   * @return float norm
   */
  // 【中文注释】计算栅格 n 处的势场梯度并写入 gradx/grady（结果已归一化）。
  // 若该格在障碍内，则用邻近可行格构造指向可行域的"虚拟梯度"。
  float gradCell(int n);  /**< calculates gradient at cell <n>, returns norm */

  float pathStep;  /**< step size for following gradient */ // 梯度回溯步长，默认 0.5（半个栅格分辨率）

  /** display callback */
  /**< <n> is the number of cycles between updates  */
  // void display(void fn(NavFn * nav), int n = 100);
  // int displayInt;  /**< save second argument of display() above */
  // void (* displayFn)(NavFn * nav);  /**< display function itself */

  /** save costmap */
  /**< write out costmap and start/goal states as fname.pgm and fname.txt */
  // void savemap(const char * fname);
};

}  // namespace nav2_navfn_planner

#endif  // NAV2_NAVFN_PLANNER__NAVFN_HPP_
