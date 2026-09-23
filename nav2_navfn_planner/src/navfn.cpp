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
// Path calculation uses no interpolation when pot field is at max in
//   nearby cells
//
// Path calc has sanity check that it succeeded
//
// 【中文注释】本文件是 NavFn 规划器的核心算法实现，算法流程分两大阶段：
//
// 阶段一：势场构建（从目标点"注水"扩散）
//   - setupNavFn(): 重置势值数组，目标点势值设为 0 作为波源；
//   - propNavFnDijkstra() / propNavFnAstar(): 以"块优先级波前传播"的方式，
//     逐层向外更新每个栅格的势值 potarr[k]（含义：从栅格 k 走到目标的累积代价）。
//     Dijkstra 是无偏的广度优先扩散（波前等势值层推进）；
//     A* 额外叠加"到起点的欧氏距离"作为排序键，让波前偏向起点方向，
//     少扩展无关节点，速度更快但不保证完整势场的一致性。
//
// 阶段二：路径回溯（从起点沿梯度下降"顺流而下"）
//   - calcPath(): 从起点出发，用 gradCell() 计算的势场梯度做双线性插值，
//     以 pathStep(默认半栅格) 步长迭代走向势值更低的区域，直到接近目标；
//   - 势值未知(POT_HIGH)区域无法用梯度，退化为 8 邻域最小势值贪心走格。
//
// 关键设计：用三个循环轮换的整数数组块(cur/next/over)替代优先队列，
// 以"近似有序"换取 O(1) 的入队出队开销，这是 NavFn 在大地图上依然很快的原因。
//

#include "nav2_navfn_planner/navfn.hpp"

#include <algorithm>
#include "rclcpp/rclcpp.hpp"

namespace nav2_navfn_planner
{

//
// function to perform nav fn calculation
// keeps track of internal buffers, will be more efficient
//   if the size of the environment does not change
//

// Example usage:
/*
int
create_nav_plan_astar(
  COSTTYPE * costmap, int nx, int ny,
  int * goal, int * start,
  float * plan, int nplan)
{
  static NavFn * nav = NULL;

  if (nav == NULL) {
    nav = new NavFn(nx, ny);
  }

  if (nav->nx != nx || nav->ny != ny) {  // check for compatibility with previous call
    delete nav;
    nav = new NavFn(nx, ny);
  }

  nav->setGoal(goal);
  nav->setStart(start);

  nav->costarr = costmap;
  nav->setupNavFn(true);

  // calculate the nav fn and path
  nav->priInc = 2 * COST_NEUTRAL;
  nav->propNavFnAstar(std::max(nx * ny / 20, nx + ny));

  // path
  int len = nav->calcPath(nplan);

  if (len > 0) {  // found plan
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Path found, %d steps\n", len);
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] No path found\n");
  }

  if (len > 0) {
    for (int i = 0; i < len; i++) {
      plan[i * 2] = nav->pathx[i];
      plan[i * 2 + 1] = nav->pathy[i];
    }
  }

  return len;
}
*/

//
// create nav fn buffers
//

NavFn::NavFn(int xs, int ys)
{
  // create cell arrays
  // 【中文注释】构造函数：分配代价/势值/梯度等核心数组（在 setNavArr 中完成），
  // 以及三个优先级块缓冲。所有指针先置空以便析构时安全 delete[]。
  costarr = NULL;
  potarr = NULL;
  pending = NULL;
  gradx = grady = NULL;
  setNavArr(xs, ys);

  // priority buffers
  // 【中文注释】三个优先级块底层存储，尺寸固定为 PRIORITYBUFSIZE(10000)。
  pb1 = new int[PRIORITYBUFSIZE];
  pb2 = new int[PRIORITYBUFSIZE];
  pb3 = new int[PRIORITYBUFSIZE];

  // for Dijkstra (breadth-first), set to COST_NEUTRAL
  // for A* (best-first), set to COST_NEUTRAL
  // 【中文注释】优先级阈值增量：当前块耗尽后 curT 每次提升 2*COST_NEUTRAL(100)，
  // 意味着一个"波前层"覆盖势值差 100 以内的栅格。
  priInc = 2 * COST_NEUTRAL;

  // goal and start
  goal[0] = goal[1] = 0;
  start[0] = start[1] = 0;

  // display function
  // displayFn = NULL;
  // displayInt = 0;

  // path buffers
  npathbuf = npath = 0;
  pathx = pathy = NULL;
  pathStep = 0.5;
}


NavFn::~NavFn()
{
  if (costarr) {
    delete[] costarr;
  }
  if (potarr) {
    delete[] potarr;
  }
  if (pending) {
    delete[] pending;
  }
  if (gradx) {
    delete[] gradx;
  }
  if (grady) {
    delete[] grady;
  }
  if (pathx) {
    delete[] pathx;
  }
  if (pathy) {
    delete[] pathy;
  }
  if (pb1) {
    delete[] pb1;
  }
  if (pb2) {
    delete[] pb2;
  }
  if (pb3) {
    delete[] pb3;
  }
}


//
// set goal, start positions for the nav fn
//

void
NavFn::setGoal(int * g)
{
  // 【中文注释】设置目标栅格坐标。注意：势场是从目标点向外扩散的，
  // 即 potarr 存的是"到目标的代价"，这是 NavFn 与常规 A*（从起点扩散）相反的地方。
  goal[0] = g[0];
  goal[1] = g[1];
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Setting goal to %d,%d\n", goal[0], goal[1]);
}

void
NavFn::setStart(int * g)
{
  // 【中文注释】设置起点栅格坐标，传播时用它判断波前是否到达起点。
  start[0] = g[0];
  start[1] = g[1];
  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"), "[NavFn] Setting start to %d,%d\n", start[0],
    start[1]);
}

//
// Set/Reset map size
//

void
NavFn::setNavArr(int xs, int ys)
{
  // 【中文注释】设置/重置地图尺寸：nx 宽、ny 高、ns 总栅格数。
  // 地图尺寸变化（如 rolling window 滑动）时先释放旧数组再重新分配。
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Array is %d x %d\n", xs, ys);

  nx = xs;
  ny = ys;
  ns = nx * ny;

  // 释放旧的缓冲区，避免内存泄漏
  if (costarr) {
    delete[] costarr;
  }
  if (potarr) {
    delete[] potarr;
  }
  if (pending) {
    delete[] pending;
  }

  if (gradx) {
    delete[] gradx;
  }
  if (grady) {
    delete[] grady;
  }

  costarr = new COSTTYPE[ns];  // cost array, 2d config space // 内部代价数组
  memset(costarr, 0, ns * sizeof(COSTTYPE));
  potarr = new float[ns];  // navigation potential array // 势值数组，初始内容稍后在 setupNavFn 中重置为 POT_HIGH
  pending = new bool[ns];  // 待处理标记数组
  memset(pending, 0, ns * sizeof(bool));
  gradx = new float[ns];  // x 方向梯度数组
  grady = new float[ns];  // y 方向梯度数组
}


//
// set up cost array, usually from ROS
//

void
NavFn::setCostmap(const COSTTYPE * cmap, bool isROS, bool allow_unknown)
{
  // 【中文注释】把外部代价地图拷贝进 costarr，并做数值域换算：
  //   * 253/254 的障碍值 -> COST_OBS(254)，绝对不可通行；
  //   * 0~252 的可行代价 -> COST_NEUTRAL(50) + COST_FACTOR(0.8)*v，均匀铺到 50~253；
  //   * 255 的未知值 -> allow_unknown 为 true 时当作"接近障碍的可行格"(253)，
  //     否则保持 COST_OBS 禁行。
  // 默认先赋 COST_OBS，只有通过检查的格子才被改写为可行值。
  COSTTYPE * cm = costarr;
  if (isROS) {  // ROS-type cost array
    // 【中文注释】ROS costmap 分支：逐格扫描整张地图做上述映射
    for (int i = 0; i < ny; i++) {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
        // This transforms the incoming cost values:
        // COST_OBS                 -> COST_OBS (incoming "lethal obstacle")
        // COST_OBS_ROS             -> COST_OBS (incoming "inscribed inflated obstacle")
        // values in range 0 to 252 -> values from COST_NEUTRAL to COST_OBS_ROS.
        *cm = COST_OBS;
        int v = *cmap;
        if (v < COST_OBS_ROS) {
          // 【中文注释】正常可行格：线性缩放到 NavFn 内部代价区间
          v = COST_NEUTRAL + COST_FACTOR * v;
          if (v >= COST_OBS) {
            v = COST_OBS - 1;  // 防止缩放后溢出为障碍
          }
          *cm = v;
        } else if (v == COST_UNKNOWN_ROS && allow_unknown) {
          // 【中文注释】未知区域但允许穿越：给最高可行代价(253)，
          // 规划器会尽量避开但必要时可以通过
          v = COST_OBS - 1;
          *cm = v;
        }
      }
    }
  } else {  // not a ROS map, just a PGM
    // 【中文注释】非 ROS 地图（如 PGM 图像）分支：额外把四周 7 格宽的边框
    // 设为障碍（continue 跳过换算，保持默认 COST_OBS），防止传播越界。
    for (int i = 0; i < ny; i++) {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
        *cm = COST_OBS;
        if (i < 7 || i > ny - 8 || j < 7 || j > nx - 8) {
          continue;  // don't do borders
        }
        int v = *cmap;
        if (v < COST_OBS_ROS) {
          v = COST_NEUTRAL + COST_FACTOR * v;
          if (v >= COST_OBS) {
            v = COST_OBS - 1;
          }
          *cm = v;
        } else if (v == COST_UNKNOWN_ROS) {
          v = COST_OBS - 1;
          *cm = v;
        }
      }
    }
  }
}

bool
NavFn::calcNavFnDijkstra(bool atStart)
{
  // 【中文注释】Dijkstra 入口：先做传播前初始化（保留 costarr），再执行波前传播。
  setupNavFn(true);

  // calculate the nav fn and path
  // 【中文注释】传播循环上限取 max(总栅格数/20, nx+ny)：
  // 大地图上一般不需要扩展所有栅格就能覆盖起点，小地图则保证至少扩展对角线长度。
  return propNavFnDijkstra(std::max(nx * ny / 20, nx + ny), atStart);
}


//
// calculate navigation function, given a costmap, goal, and start
//

bool
NavFn::calcNavFnAstar()
{
  // 【中文注释】A* 入口：初始化后用带欧氏启发式的最佳优先传播，
  // 起点势值一旦被更新即提前结束，比 Dijkstra 快得多。
  setupNavFn(true);

  // calculate the nav fn and path
  return propNavFnAstar(std::max(nx * ny / 20, nx + ny));
}

//
// returning values
//
// 【中文注释】简单访问器：返回路径坐标数组和路径点数
float * NavFn::getPathX() {return pathx;}
float * NavFn::getPathY() {return pathy;}
int NavFn::getPathLen() {return npath;}

// inserting onto the priority blocks
// 【中文注释】压入优先级块的三个宏（cur/next/over 对应三个目标块）：
// 入队条件：索引合法(0~ns-1)、尚未 pending（避免重复入队）、非障碍、块未满。
// 入队后立即置 pending 标记。宏形式保证内联高效，这是传播主循环的热点路径。
#define push_cur(n)  {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && curPe < PRIORITYBUFSIZE) \
    {curP[curPe++] = n; pending[n] = true;}}
#define push_next(n) {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && nextPe < PRIORITYBUFSIZE) \
    {nextP[nextPe++] = n; pending[n] = true;}}
#define push_over(n) {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && overPe < PRIORITYBUFSIZE) \
    {overP[overPe++] = n; pending[n] = true;}}


// Set up navigation potential arrays for new propagation

void
NavFn::setupNavFn(bool keepit)
{
  // reset values in propagation arrays
  // 【中文注释】传播前重置：所有势值设为 POT_HIGH(未触达)，
  // 梯度清零；keepit=false 时还会把代价数组整体重置为 COST_NEUTRAL
  // （即把地图当作均匀自由空间，通常在无外部 costmap 时使用）。
  for (int i = 0; i < ns; i++) {
    potarr[i] = POT_HIGH;
    if (!keepit) {
      costarr[i] = COST_NEUTRAL;
    }
    gradx[i] = grady[i] = 0.0;
  }

  // outer bounds of cost array
  // 【中文注释】给代价数组四周边框全部置为 COST_OBS：
  // 由于 updateCell 直接用 n±1、n±nx 访问邻居（无边界检查），
  // 用障碍边框可以防止波前传播和梯度计算越界访问内存。
  COSTTYPE * pc;
  pc = costarr;  // 上边框
  for (int i = 0; i < nx; i++) {
    *pc++ = COST_OBS;
  }
  pc = costarr + (ny - 1) * nx;  // 下边框
  for (int i = 0; i < nx; i++) {
    *pc++ = COST_OBS;
  }
  pc = costarr;  // 左边框
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = COST_OBS;
  }
  pc = costarr + nx - 1;  // 右边框
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = COST_OBS;
  }

  // priority buffers
  // 【中文注释】重置块优先级状态：初始阈值 curT = COST_OBS(254)，
  // 三个块指针分别指向 pb1/pb2/pb3，元素计数清零，pending 标记全部清除。
  curT = COST_OBS;
  curP = pb1;
  curPe = 0;
  nextP = pb2;
  nextPe = 0;
  overP = pb3;
  overPe = 0;
  memset(pending, 0, ns * sizeof(bool));

  // set goal
  // 【中文注释】目标点作为波源：势值设为 0，并把它的 4 邻居压入当前块，
  // 随后传播从这些邻居开始一轮一轮向外扩散。
  int k = goal[0] + goal[1] * nx;
  initCost(k, 0);

  // find # of obstacle cells
  // 【中文注释】统计障碍栅格总数 nobs，仅供调试日志计算"可通行栅格访问率"。
  pc = costarr;
  int ntot = 0;
  for (int i = 0; i < ns; i++, pc++) {
    if (*pc >= COST_OBS) {
      ntot++;  // number of cells that are obstacles
    }
  }
  nobs = ntot;
}


// initialize a goal-type cost for starting propagation

void
NavFn::initCost(int k, float v)
{
  // 【中文注释】以势值 v 初始化栅格 k（通常是目标点，v=0），
  // 然后把上下左右四个邻居压入"当前优先级块"，作为传播的第一圈波前。
  // push_cur 宏内部会自动过滤越界/障碍/pending 的格子。
  potarr[k] = v;
  push_cur(k + 1);
  push_cur(k - 1);
  push_cur(k - nx);
  push_cur(k + nx);
}


//
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

#define INVSQRT2 0.707106781

inline void
NavFn::updateCell(int n)
{
  // get neighbors
  // 【中文注释】读取当前栅格 n 的 4 邻域势值：
  // l 左(n-1)、r 右(n+1)、u 上(n-nx)、d 下(n+nx)。
  // 注意：因为地图四周有障碍边框，这里可以放心做无边界检查的越界访问。
  float u, d, l, r;
  l = potarr[n - 1];
  r = potarr[n + 1];
  u = potarr[n - nx];
  d = potarr[n + nx];
  // ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n",
  //  potarr[n], l, r, u, d);
  // ROS_INFO("[Update] cost: %d\n", costarr[n]);

  // find lowest, and its lowest neighbor
  // 【中文注释】分别取左右、上下两组中的较小者：
  // tc = min(l, r)，ta = min(u, d)；最终 ta 为 4 邻域最小势值，tc 为次小（另一轴）。
  float ta, tc;
  if (l < r) {tc = l;} else {tc = r;}
  if (u < d) {ta = u;} else {ta = d;}

  // do planar wave update
  if (costarr[n] < COST_OBS) {  // don't propagate into obstacles
    // 【中文注释】平面波更新（本算法最核心的公式）：
    // hf = 本格的通行代价（traversability factor），代表穿过该格的成本；
    // 直观理解：新势值 ≈ 最小邻居势值 + 本格代价，但由于波可能同时从
    // 水平+垂直两个方向斜着传来，双邻居插值能得到比"单邻居+代价"更小的估计。
    float hf = static_cast<float>(costarr[n]);  // traversability factor
    float dc = tc - ta;  // relative cost between ta,tc
    if (dc < 0) {  // ta is lowest
      // 【中文注释】保证 ta 始终是最小者，dc = |次小 - 最小| >= 0
      dc = -dc;
      ta = tc;
    }

    // calculate new potential
    float pot;
    if (dc >= hf) {  // if too large, use ta-only update
      // 【中文注释】情形一：次小邻居势值远高于最小邻居(dc >= hf)，
      // 波只从最小邻居方向传来，直接用"最小邻居势值 + 本格代价"。
      pot = ta + hf;
    } else {  // two-neighbor interpolation update
      // use quadratic approximation
      // might speed this up through table lookup, but still have to
      //   do the divide
      // 【中文注释】情形二：两个方向的波强度相近，用二次多项式近似
      // 求解 Eikonal 方程的插值解：d = dc/hf 归一化后，
      // v = -0.2301*d^2 + 0.5307*d + 0.7040（拟合的解曲线），
      // 使斜向传播的等效代价因子小于 1（类比 0.707=1/sqrt(2) 的对角线长度）。
      float d = dc / hf;
      float v = -0.2301 * d * d + 0.5307 * d + 0.7040;
      pot = ta + hf * v;
    }

    //      ROS_INFO("[Update] new pot: %d\n", costarr[n]);

    // now add affected neighbors to priority blocks
    // 【中文注释】只有当新势值确实比旧值小时才写入，并把"势值可能因此
    // 需要更新的邻居"重新入队。邻居代价乘以 INVSQRT2(1/sqrt(2)) 是
    // 斜向传播时的等效代价（对角线长度投影）。
    if (pot < potarr[n]) {
      float le = INVSQRT2 * static_cast<float>(costarr[n - 1]);
      float re = INVSQRT2 * static_cast<float>(costarr[n + 1]);
      float ue = INVSQRT2 * static_cast<float>(costarr[n - nx]);
      float de = INVSQRT2 * static_cast<float>(costarr[n + nx]);
      potarr[n] = pot;
      // 【中文注释】按新势值把邻居分入低代价块（势值仍在当前阈值区间内，
      // 入 next 块）或溢出块（超出当前优先级区间，入 over 块推迟处理）。
      if (pot < curT) {  // low-cost buffer block
        if (l > pot + le) {push_next(n - 1);}
        if (r > pot + re) {push_next(n + 1);}
        if (u > pot + ue) {push_next(n - nx);}
        if (d > pot + de) {push_next(n + nx);}
      } else {  // overflow block
        if (l > pot + le) {push_over(n - 1);}
        if (r > pot + re) {push_over(n + 1);}
        if (u > pot + ue) {push_over(n - nx);}
        if (d > pot + de) {push_over(n + nx);}
      }
    }
  }
}

//
// Use A* method for setting priorities
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

#define INVSQRT2 0.707106781

inline void
NavFn::updateCellAstar(int n)
{
  // get neighbors
  // 【中文注释】与 updateCell 相同的框架：读取 4 邻域势值，做平面波更新。
  // 区别在于最终入队时的排序键叠加了 A* 启发式（见下方 dist 计算）。
  float u, d, l, r;
  l = potarr[n - 1];
  r = potarr[n + 1];
  u = potarr[n - nx];
  d = potarr[n + nx];
  // ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n",
  // potarr[n], l, r, u, d);
  // ROS_INFO("[Update] cost of %d: %d\n", n, costarr[n]);

  // find lowest, and its lowest neighbor
  // 【中文注释】ta = 4 邻域最小势值，tc = 次小势值
  float ta, tc;
  if (l < r) {tc = l;} else {tc = r;}
  if (u < d) {ta = u;} else {ta = d;}

  // do planar wave update
  if (costarr[n] < COST_OBS) {  // don't propagate into obstacles
    // 【中文注释】平面波势值计算，与 updateCell 完全一致：
    // hf 为本格通行代价；dc 大时退化为单邻居更新，dc 小时用二次插值。
    float hf = static_cast<float>(costarr[n]);  // traversability factor
    float dc = tc - ta;  // relative cost between ta,tc
    if (dc < 0) {  // ta is lowest
      dc = -dc;
      ta = tc;
    }

    // calculate new potential
    float pot;
    if (dc >= hf) {  // if too large, use ta-only update
      pot = ta + hf;
    } else {  // two-neighbor interpolation update
      // use quadratic approximation
      // might speed this up through table lookup, but still have to
      //   do the divide
      float d = dc / hf;
      float v = -0.2301 * d * d + 0.5307 * d + 0.7040;
      pot = ta + hf * v;
    }

    // ROS_INFO("[Update] new pot: %d\n", costarr[n]);

    // now add affected neighbors to priority blocks
    if (pot < potarr[n]) {
      float le = INVSQRT2 * static_cast<float>(costarr[n - 1]);
      float re = INVSQRT2 * static_cast<float>(costarr[n + 1]);
      float ue = INVSQRT2 * static_cast<float>(costarr[n - nx]);
      float de = INVSQRT2 * static_cast<float>(costarr[n + nx]);

      // calculate distance
      // 【中文注释】A* 的关键差异：计算本格到起点的欧氏距离作为启发式项 h(n)，
      // 乘以 COST_NEUTRAL 换算到与势值相同的量纲（自由空间每格代价 50）。
      // 注意方向：势场是从目标扩散的 g 值，这里叠加的是"到起点"的距离，
      // 使 f = g + h 的波前优先朝起点方向推进。
      int x = n % nx;
      int y = n / nx;
      float dist = hypot(x - start[0], y - start[1]) * static_cast<float>(COST_NEUTRAL);

      potarr[n] = pot;  // potarr 保存纯 g 值（不含启发式）
      pot += dist;      // 入队排序键使用 f = g + h
      // 【中文注释】按 f 值把受影响邻居分入 next 块（低于当前阈值）或 over 块。
      if (pot < curT) {  // low-cost buffer block
        if (l > pot + le) {push_next(n - 1);}
        if (r > pot + re) {push_next(n + 1);}
        if (u > pot + ue) {push_next(n - nx);}
        if (d > pot + de) {push_next(n + nx);}
      } else {
        if (l > pot + le) {push_over(n - 1);}
        if (r > pot + re) {push_over(n + 1);}
        if (u > pot + ue) {push_over(n - nx);}
        if (d > pot + de) {push_over(n + nx);}
      }
    }
  }
}


//
// main propagation function
// Dijkstra method, breadth-first
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)
//

bool
NavFn::propNavFnDijkstra(int cycles, bool atStart)
{
  // 【中文注释】Dijkstra 广度优先主循环：
  // 每一轮(cycle)处理完整个"当前块"，再把 next 块提升为新的当前块，
  // 类似 BFS 按层扩展；当当前层耗尽时提升优先级阈值并启用溢出块。
  int nwv = 0;  // max priority block size
  int nc = 0;  // number of cells put into priority blocks
  int cycle = 0;  // which cycle we're on

  // set up start cell
  // 【中文注释】起点栅格的一维索引，用于检测波前是否已到达起点
  int startCell = start[1] * nx + start[0];

  for (; cycle < cycles; cycle++) {  // go for this many cycles, unless interrupted
    if (curPe == 0 && nextPe == 0) {  // priority blocks empty
      // 【中文注释】当前块和下一块都为空：可达区域已全部扩展完，提前结束
      break;
    }

    // stats
    // 【中文注释】统计量：累计处理栅格数 nc、历史最大块大小 nwv（仅用于日志）
    nc += curPe;
    if (curPe > nwv) {
      nwv = curPe;
    }

    // reset pending flags on current priority buffer
    // 【中文注释】先把当前块所有格子的 pending 标记清掉，
    // 这样处理过程中这些格子若再次被更新可以重新入队
    int * pb = curP;
    int i = curPe;
    while (i-- > 0) {
      pending[*(pb++)] = false;
    }

    // process current priority buffer
    // 【中文注释】逐个更新当前块中的栅格势值（核心计算在 updateCell 中）
    pb = curP;
    i = curPe;
    while (i-- > 0) {
      updateCell(*pb++);
    }

    // if (displayInt > 0 && (cycle % displayInt) == 0) {
    //   displayFn(this);
    // }

    // swap priority blocks curP <=> nextP
    // 【中文注释】块轮换：next 块升格为新的当前块，旧当前块变成空的 next 块
    curPe = nextPe;
    nextPe = 0;
    pb = curP;  // swap buffers
    curP = nextP;
    nextP = pb;

    // see if we're done with this priority level
    // 【中文注释】当前优先级层处理完毕：提升阈值 curT += priInc，
    // 并把溢出块(over)中势值较高的格子接管为新的当前块继续处理。
    if (curPe == 0) {
      curT += priInc;  // increment priority threshold
      curPe = overPe;  // set current to overflow block
      overPe = 0;
      pb = curP;  // swap buffers
      curP = overP;
      overP = pb;
    }

    // check if we've hit the Start cell
    // 【中文注释】atStart=true 时：起点势值一旦低于 POT_HIGH（波前已触达）
    // 就立即停止传播——不需要算完整张势场，只需覆盖起点即可回溯路径。
    if (atStart) {
      if (potarr[startCell] < POT_HIGH) {
        break;
      }
    }
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"),
    "[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
    cycle, nc, (int)((nc * 100.0) / (ns - nobs)), nwv);

  // 【中文注释】返回 true 表示在 cycles 轮内结束（即传播完成或提前到达起点）
  return (cycle < cycles) ? true : false;
}

//
// main propagation function
// A* method, best-first
// uses Euclidean distance heuristic
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)
//

bool
NavFn::propNavFnAstar(int cycles)
{
  // 【中文注释】A* 最佳优先主循环：结构与 propNavFnDijkstra 相同，
  // 差异有两点：
  //   1) 初始阈值加入起终点欧氏距离（最优路径势值的下界估计），
  //      使第一层波前直接覆盖到起点方向的区间；
  //   2) 每个 updateCellAstar 的入队键是 f = g + h，波前偏向起点推进。
  int nwv = 0;  // max priority block size
  int nc = 0;  // number of cells put into priority blocks
  int cycle = 0;  // which cycle we're on

  // set initial threshold, based on distance
  // 【中文注释】初始优先级阈值 = 基础阈值 + 起终点欧氏距离*COST_NEUTRAL。
  // 因为任何合法路径的 g 值都不可能小于"自由空间直线距离的代价"，
  // 用它抬高阈值可以让第一批 f 值较低（靠近起点）的格子尽快被处理。
  float dist = hypot(goal[0] - start[0], goal[1] - start[1]) * static_cast<float>(COST_NEUTRAL);
  curT = dist + curT;

  // set up start cell
  int startCell = start[1] * nx + start[0];

  // do main cycle
  for (; cycle < cycles; cycle++) {  // go for this many cycles, unless interrupted
    if (curPe == 0 && nextPe == 0) {  // priority blocks empty
      // 【中文注释】两块皆空：全部可达区域都已扩展（目标不可达的情况）
      break;
    }

    // stats
    nc += curPe;
    if (curPe > nwv) {
      nwv = curPe;
    }

    // reset pending flags on current priority buffer
    // 【中文注释】清除当前块格子的 pending 标记，允许其再次入队
    int * pb = curP;
    int i = curPe;
    while (i-- > 0) {
      pending[*(pb++)] = false;
    }

    // process current priority buffer
    // 【中文注释】用 A* 版本的栅格更新处理当前块（排序键含启发式）
    pb = curP;
    i = curPe;
    while (i-- > 0) {
      updateCellAstar(*pb++);
    }

    // if (displayInt > 0 && (cycle % displayInt) == 0) {
    //   displayFn(this);
    // }

    // swap priority blocks curP <=> nextP
    // 【中文注释】块轮换：next -> cur，旧 cur 清空作为新 next
    curPe = nextPe;
    nextPe = 0;
    pb = curP;  // swap buffers
    curP = nextP;
    nextP = pb;

    // see if we're done with this priority level
    // 【中文注释】当前层耗尽：提升阈值，溢出块接管为当前块
    if (curPe == 0) {
      curT += priInc;  // increment priority threshold
      curPe = overPe;  // set current to overflow block
      overPe = 0;
      pb = curP;  // swap buffers
      curP = overP;
      overP = pb;
    }

    // check if we've hit the Start cell
    // 【中文注释】A* 模式下无条件检查：起点势值一旦有效即结束传播，
    // 起点势值即整条路径的累积代价。
    if (potarr[startCell] < POT_HIGH) {
      break;
    }
  }

  last_path_cost_ = potarr[startCell];

  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"),
    "[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
    cycle, nc, (int)((nc * 100.0) / (ns - nobs)), nwv);

  // 【中文注释】起点势值有效 -> 找到路径；仍为 POT_HIGH -> 目标不可达
  if (potarr[startCell] < POT_HIGH) {
    return true;  // finished up here}
  } else {
    return false;
  }
}


float NavFn::getLastPathCost()
{
  // 【中文注释】返回上次 A* 传播结束时起点的势值，即整条路径的总代价
  return last_path_cost_;
}


//
// Path construction
// Find gradient at array points, interpolate path
// Use step size of pathStep, usually 0.5 pixel
//
// Some sanity checks:
//  1. Stuck at same index position
//  2. Doesn't get near goal
//  3. Surrounded by high potentials
//

int
NavFn::calcPath(int n, int * st)
{
  // test write
  // savemap("test");

  // check path arrays
  // 【中文注释】路径回溯主函数：在已算好的势场上，从起点(或 st)出发，
  // 沿势值下降方向以 pathStep(0.5) 步长逐步走向目标，最多迭代 n 次。
  // 首先确保路径缓冲区容量足够（不足则释放重分配）。
  if (npathbuf < n) {
    if (pathx) {delete[] pathx;}
    if (pathy) {delete[] pathy;}
    pathx = new float[n];
    pathy = new float[n];
    npathbuf = n;
  }

  // set up start position at cell
  // st is always upper left corner for 4-point bilinear interpolation
  // 【中文注释】回溯起点：默认为机器人 start；stc 为所在栅格的一维索引。
  // stc 与 (dx,dy) 共同表示"亚栅格"位置：stc 是包围当前点的左上角栅格，
  // (dx,dy) 是相对该栅格原点的偏移，供双线性插值使用。
  if (st == NULL) {st = start;}
  int stc = st[1] * nx + st[0];

  // set up offset
  // 【中文注释】dx/dy 为栅格内小数偏移，初始为 0（位于栅格原点）
  float dx = 0;
  float dy = 0;
  npath = 0;

  // go for <n> cycles at most
  for (int i = 0; i < n; i++) {
    // check if near goal
    // 【中文注释】终点判定：把亚栅格位置四舍五入到最近栅格，
    // 若其势值 < COST_NEUTRAL(50)，说明已非常接近目标（目标势值为 0），
    // 直接把目标坐标写入为最后一个路径点并成功返回。
    int nearest_point = std::max(
      0,
      std::min(
        nx * ny - 1, stc + static_cast<int>(round(dx)) +
        static_cast<int>(nx * round(dy))));
    if (potarr[nearest_point] < COST_NEUTRAL) {
      pathx[npath] = static_cast<float>(goal[0]);
      pathy[npath] = static_cast<float>(goal[1]);
      return ++npath;  // done!
    }

    if (stc < nx || stc > ns - nx) {  // would be out of bounds
      // 【中文注释】回溯位置到达地图上下边界：路径失败
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Out of bounds");
      return 0;
    }

    // add to path
    // 【中文注释】记录当前路径点 = 整栅格坐标 + 亚栅格偏移
    pathx[npath] = stc % nx + dx;
    pathy[npath] = stc / nx + dy;
    npath++;

    // 【中文注释】振荡检测：若当前点与两个迭代前的点重合，
    // 说明梯度回溯在两格间来回跳动，需转入网格贪心模式修正。
    bool oscillation_detected = false;
    if (npath > 2 &&
      pathx[npath - 1] == pathx[npath - 3] &&
      pathy[npath - 1] == pathy[npath - 3])
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[PathCalc] oscillation detected, attempting fix.");
      oscillation_detected = true;
    }

    int stcnx = stc + nx;
    int stcpx = stc - nx;

    // check for potentials at eight positions near cell
    // 【中文注释】退化条件检查：若当前格 3x3 邻域内任一格势值未知(POT_HIGH)
    // 或发生振荡，则无法可靠地做梯度插值，转而使用"8 邻域最小势值贪心"的走格模式。
    if (potarr[stc] >= POT_HIGH ||
      potarr[stc + 1] >= POT_HIGH ||
      potarr[stc - 1] >= POT_HIGH ||
      potarr[stcnx] >= POT_HIGH ||
      potarr[stcnx + 1] >= POT_HIGH ||
      potarr[stcnx - 1] >= POT_HIGH ||
      potarr[stcpx] >= POT_HIGH ||
      potarr[stcpx + 1] >= POT_HIGH ||
      potarr[stcpx - 1] >= POT_HIGH ||
      oscillation_detected)
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[Path] Pot fn boundary, following grid (%0.1f/%d)", potarr[stc], npath);

      // check eight neighbors to find the lowest
      // 【中文注释】网格模式：逐一比较 8 个邻居（上三、中二、下三）的势值，
      // 直接跳到势值最小的邻居格，并把亚栅格偏移清零（回到整格坐标）。
      int minc = stc;
      int minp = potarr[stc];
      int st = stcpx - 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st = stc - 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st = stc + 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st = stcnx - 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      stc = minc;
      dx = 0;
      dy = 0;

      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"), "[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
        potarr[stc], pathx[npath - 1], pathy[npath - 1]);

      if (potarr[stc] >= POT_HIGH) {
        // 【中文注释】最小邻居仍是 POT_HIGH：周围全未触达，起点与目标不连通，失败
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, high potential");
        // savemap("navfn_highpot");
        return 0;
      }
    } else {  // have a good gradient here
      // get grad at four positions near cell
      // 【中文注释】梯度模式（正常模式）：对覆盖当前位置的 4 个栅格
      // （左上 stc、右上 stc+1、左下 stcnx、右下 stcnx+1）计算势场梯度。
      gradCell(stc);
      gradCell(stc + 1);
      gradCell(stcnx);
      gradCell(stcnx + 1);


      // get interpolated gradient
      // 【中文注释】双线性插值：先沿 x 插值(x1/x2, y1/y2)，再沿 y 插值，
      // 得到当前位置处的连续梯度向量 (x, y)——即势值下降最快的方向。
      float x1 = (1.0 - dx) * gradx[stc] + dx * gradx[stc + 1];
      float x2 = (1.0 - dx) * gradx[stcnx] + dx * gradx[stcnx + 1];
      float x = (1.0 - dy) * x1 + dy * x2;  // interpolated x
      float y1 = (1.0 - dx) * grady[stc] + dx * grady[stc + 1];
      float y2 = (1.0 - dx) * grady[stcnx] + dx * grady[stcnx + 1];
      float y = (1.0 - dy) * y1 + dy * y2;  // interpolated y

#if 0
      // show gradients
      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n",
        gradx[stc], grady[stc], gradx[stc + 1], grady[stc + 1],
        gradx[stcnx], grady[stcnx], gradx[stcnx + 1], grady[stcnx + 1],
        x, y);
#endif

      // check for zero gradient, failed
      // 【中文注释】梯度为零：局部势场平坦（常见于代价平台区），无法前进，失败
      if (x == 0.0 && y == 0.0) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Zero gradient");
        return 0;
      }

      // move in the right direction
      // 【中文注释】沿梯度方向移动 pathStep(0.5) 步长：
      // ss = pathStep/|grad| 把梯度归一化后累加到亚栅格偏移 (dx, dy) 上。
      float ss = pathStep / hypot(x, y);
      dx += x * ss;
      dy += y * ss;

      // check for overflow
      // 【中文注释】偏移越界即切换到相邻栅格：dx 超过 ±1 表示跨入右/左格，
      // dy 超过 ±1 表示跨入下/上格，同时把偏移折回 [-1,1] 区间。
      if (dx > 1.0) {stc++; dx -= 1.0;}
      if (dx < -1.0) {stc--; dx += 1.0;}
      if (dy > 1.0) {stc += nx; dy -= 1.0;}
      if (dy < -1.0) {stc -= nx; dy += 1.0;}
    }

    //      ROS_INFO("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
    //      potarr[stc], x, y, pathx[npath-1], pathy[npath-1]);
  }

  //  return npath;  // out of cycles, return failure
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, path too long");
  // savemap("navfn_pathlong");
  return 0;  // out of cycles, return failure
}


//
// gradient calculations
//

// calculate gradient at a cell
// positive value are to the right and down
// 【中文注释】计算栅格 n 的势场梯度并写入 gradx[n]/grady[n]（已归一化）。
// 梯度方向 = 势值下降最快的方向，即指向目标；正值方向为右/下。
float
NavFn::gradCell(int n)
{
  // 【中文注释】缓存检查：gradx+grady > 0 表示该格梯度已计算过（非零），
  // 直接返回 1.0 跳过重复计算（梯度只依赖势值，势值在回溯阶段不再变化）。
  if (gradx[n] + grady[n] > 0.0) {  // check this cell
    return 1.0;
  }

  if (n < nx || n > ns - nx) {  // would be out of bounds
    // 【中文注释】第一行/最后一行栅格：上下邻居越界，不计算梯度
    return 0.0;
  }

  float cv = potarr[n];
  float dx = 0.0;
  float dy = 0.0;

  // check for in an obstacle
  // 【中文注释】障碍内(POT_HIGH)栅格：没有有效势值可差分，
  // 改用"指向最近可行邻居"的大数值虚拟梯度（±COST_OBS），
  // 让回溯路径能够贴着障碍边缘离开障碍区域。
  if (cv >= POT_HIGH) {
    if (potarr[n - 1] < POT_HIGH) {
      dx = -COST_OBS;
    } else if (potarr[n + 1] < POT_HIGH) {
      dx = COST_OBS;
    }
    if (potarr[n - nx] < POT_HIGH) {
      dy = -COST_OBS;
    } else if (potarr[n + nx] < POT_HIGH) {
      dy = COST_OBS;
    }
  } else {  // not in an obstacle
    // dx calc, average to sides
    // 【中文注释】正常栅格：中心差分近似梯度。
    // dx = (左邻势值 - 本格) + (本格 - 右邻势值)，即左右势值差的组合；
    // 邻居为 POT_HIGH（未触达）时跳过该侧，只用单侧差分。
    if (potarr[n - 1] < POT_HIGH) {
      dx += potarr[n - 1] - cv;
    }
    if (potarr[n + 1] < POT_HIGH) {
      dx += cv - potarr[n + 1];
    }

    // dy calc, average to sides
    // 【中文注释】y 方向同理：上下邻居的中心差分
    if (potarr[n - nx] < POT_HIGH) {
      dy += potarr[n - nx] - cv;
    }
    if (potarr[n + nx] < POT_HIGH) {
      dy += cv - potarr[n + nx];
    }
  }

  // normalize
  // 【中文注释】归一化梯度为单位向量写入 gradx/grady（方向保留、模长变 1），
  // 回溯时的步长由 pathStep 控制；norm 为 0 时保持梯度为零。
  float norm = hypot(dx, dy);
  if (norm > 0) {
    norm = 1.0 / norm;
    gradx[n] = norm * dx;
    grady[n] = norm * dy;
  }
  return norm;
}


//
// display function setup
// <n> is the number of cycles to wait before displaying,
//     use 0 to turn it off

// void
// NavFn::display(void fn(NavFn * nav), int n)
// {
//   displayFn = fn;
//   displayInt = n;
// }


//
// debug writes
// saves costmap and start/goal
//

// void
// NavFn::savemap(const char * fname)
// {
//   char fn[4096];

//   RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Saving costmap and start/goal points");
//   // write start and goal points
//   snprintf(fn, sizeof(fn), "%s.txt", fname);
//   FILE * fp = fopen(fn, "w");
//   if (!fp) {
//     RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Can't open file %s", fn);
//     return;
//   }
//   fprintf(fp, "Goal: %d %d\nStart: %d %d\n", goal[0], goal[1], start[0], start[1]);
//   fclose(fp);

//   // write cost array
//   if (!costarr) {
//     return;
//   }
//   snprintf(fn, sizeof(fn), "%s.pgm", fname);
//   fp = fopen(fn, "wb");
//   if (!fp) {
//     RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Can't open file %s", fn);
//     return;
//   }
//   fprintf(fp, "P5\n%d\n%d\n%d\n", nx, ny, 0xff);
//   fwrite(costarr, 1, nx * ny, fp);
//   fclose(fp);
// }

}  // namespace nav2_navfn_planner
