// Copyright (c) 2022, Samsung Research America
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

// ============================================================================
// 文件说明（中文）：
// 简单梯度下降路径平滑插件（SimpleSmoother）的实现。
// 模块定位：nav2_smoother 的默认平滑算法插件，实现 nav2_core::Smoother 接口。
//
// 算法核心（迭代平滑 / 弹性带思想的简化版）：
//   对路径内部每个点 i，在 x、y 两个维度上反复执行如下更新：
//     y_i += w_data * (x_i - y_i) + w_smooth * (y_{i+1} + y_{i-1} - 2*y_i)
//   其中 x_i 是原始路径点（数据项），y_i 是平滑中的路径点：
//     - 数据项 w_data*(x_i - y_i)：把点拉回原始位置，保证平滑后路径
//       不会偏离原路径太远（数据保真）；
//     - 平滑项 w_smooth*(y_{i+1}+y_{i-1}-2*y_i)：二阶中心差分（离散拉普拉斯），
//       即把点拉向两侧邻居的中点，使路径曲率最小、趋于平滑。
//   两项加权平衡即得到"平滑且不离谱"的路径。
//   收敛条件：所有点的单轮累计位移 change < tolerance（容差）。
//
// 端点约束：循环只处理下标 1..n-2，首尾点始终不动，
// 保证平滑前后路径的起点与终点一致。
//
// 额外机制：
//   - 每轮迭代后用代价地图检查新路径点代价，若撞上障碍物
//     （代价 > MAX_NON_OBSTACLE 且非未知区域）则回退到上一轮结果并失败返回；
//   - 迭代次数上限 max_its 与时间预算 max_time 双重限制，超限回退；
//   - 收敛后可递归精炼（refinement，最多 4 次）进一步消除残余抖动。
// ============================================================================

#include <vector>
#include <memory>
#include "nav2_smoother/simple_smoother.hpp"

namespace nav2_smoother
{
using namespace smoother_utils;  // NOLINT
using namespace nav2_util::geometry_utils;  // NOLINT
using namespace std::chrono;  // NOLINT
using nav2_util::declare_parameter_if_not_declared;

// ---------------------------------------------------------------------------
// 插件配置回调：由 SmootherServer 在 on_configure 阶段调用。
// 保存代价地图订阅器（用于平滑过程中的碰撞检查），并读取参数：
//   tolerance   收敛容差（单轮累计位移阈值，默认 1e-10）
//   max_its     最大迭代次数（默认 1000）
//   w_data      数据保真项权重（默认 0.2）
//   w_smooth    平滑项权重（默认 0.3）
//   do_refinement 是否在收敛后做二次精炼（默认 true）
// ---------------------------------------------------------------------------
void SimpleSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>/*footprint_sub*/)
{
  costmap_sub_ = costmap_sub;

  auto node = parent.lock();
  logger_ = node->get_logger();

  // 若参数未配置则按默认值声明（declare_parameter_if_not_declared 工具函数）
  declare_parameter_if_not_declared(
    node, name + ".tolerance", rclcpp::ParameterValue(1e-10));
  declare_parameter_if_not_declared(
    node, name + ".max_its", rclcpp::ParameterValue(1000));
  declare_parameter_if_not_declared(
    node, name + ".w_data", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, name + ".w_smooth", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, name + ".do_refinement", rclcpp::ParameterValue(true));

  node->get_parameter(name + ".tolerance", tolerance_);
  node->get_parameter(name + ".max_its", max_its_);
  node->get_parameter(name + ".w_data", data_w_);
  node->get_parameter(name + ".w_smooth", smooth_w_);
  node->get_parameter(name + ".do_refinement", do_refinement_);
}

// ---------------------------------------------------------------------------
// 平滑主入口：按方向切段后逐段平滑。
//   1. 从订阅器获取最新全局代价地图快照（shared_ptr，供碰撞检查）；
//   2. 重置精炼计数器、记录起始时间；
//   3. findDirectionalPathSegments 切分出同方向段（尖点/原地旋转处切断），
//      防止平滑把前进段和倒车段"拉直"；
//   4. 长度 > 9 的段拷贝出来单独平滑，再写回原路径。
// ---------------------------------------------------------------------------
bool SimpleSmoother::smooth(
  nav_msgs::msg::Path & path,
  const rclcpp::Duration & max_time)
{
  // 获取全局代价地图（getCostmap 返回共享指针，可能为空）
  auto costmap = costmap_sub_->getCostmap();

  refinement_ctr_ = 0;
  steady_clock::time_point start = steady_clock::now();
  double time_remaining = max_time.seconds();

  bool success = true, reversing_segment;
  nav_msgs::msg::Path curr_path_segment;
  curr_path_segment.header = path.header;

  // 将路径按行驶方向切分为若干段
  std::vector<PathSegment> path_segments = findDirectionalPathSegments(path);

  for (unsigned int i = 0; i != path_segments.size(); i++) {
    // 段太短（<= 9 个点）不值得平滑，跳过
    if (path_segments[i].end - path_segments[i].start > 9) {
      // Populate path segment
      // 拷贝当前段 [start, end] 到临时路径
      curr_path_segment.poses.clear();
      std::copy(
        path.poses.begin() + path_segments[i].start,
        path.poses.begin() + path_segments[i].end + 1,
        std::back_inserter(curr_path_segment.poses));

      // Make sure we're still able to smooth with time remaining
      // 计算剩余时间预算并传给本段的平滑实现
      steady_clock::time_point now = steady_clock::now();
      time_remaining = max_time.seconds() - duration_cast<duration<double>>(now - start).count();

      // Smooth path segment naively
      // 执行梯度下降式迭代平滑（含碰撞检查与超时保护）
      success = success && smoothImpl(
        curr_path_segment, reversing_segment, costmap.get(), time_remaining);

      // Assemble the path changes to the main path
      // 将平滑后的段写回原路径
      std::copy(
        curr_path_segment.poses.begin(),
        curr_path_segment.poses.end(),
        path.poses.begin() + path_segments[i].start);
    }
  }

  return success;
}

// ---------------------------------------------------------------------------
// 平滑核心实现（单个方向段）：
// 迭代更新内部点的 x/y 坐标直至收敛（change < tolerance）。
// 任何失败（超迭代数 / 超时 / 引入碰撞）都会回退到 last_path
// （上一个可行状态）并返回 false，保证输出始终是可行路径。
// ---------------------------------------------------------------------------
bool SimpleSmoother::smoothImpl(
  nav_msgs::msg::Path & path,
  bool & reversing_segment,
  const nav2_costmap_2d::Costmap2D * costmap,
  const double & max_time)
{
  // 记录本段平滑的起始时刻，用于时间预算控制
  steady_clock::time_point a = steady_clock::now();
  rclcpp::Duration max_dur = rclcpp::Duration::from_seconds(max_time);

  int its = 0;
  double change = tolerance_;  // 初始值设为容差以进入循环
  const unsigned int & path_size = path.poses.size();
  double x_i, y_i, y_m1, y_ip1, y_i_org;
  unsigned int mx, my;

  nav_msgs::msg::Path new_path = path;   // 当前正在更新的平滑路径
  nav_msgs::msg::Path last_path = path;  // 上一轮的可行路径（失败时回退用）

  // 主迭代循环：累计位移 change 小于容差即认为收敛
  while (change >= tolerance_) {
    its += 1;
    change = 0.0;

    // Make sure the smoothing function will converge
    // 迭代次数超限：回退到上一轮结果，更新朝向后失败返回
    if (its >= max_its_) {
      RCLCPP_WARN(
        logger_,
        "Number of iterations has exceeded limit of %i.", max_its_);
      path = last_path;
      updateApproximatePathOrientations(path, reversing_segment);
      return false;
    }

    // Make sure still have time left to process
    // 时间预算耗尽：同样回退并失败返回
    steady_clock::time_point b = steady_clock::now();
    rclcpp::Duration timespan(duration_cast<duration<double>>(b - a));
    if (timespan > max_dur) {
      RCLCPP_WARN(
        logger_,
        "Smoothing time exceeded allowed duration of %0.2f.", max_time);
      path = last_path;
      updateApproximatePathOrientations(path, reversing_segment);
      return false;
    }

    // 一轮迭代：遍历所有内部点（首尾点固定，保证端点不变）
    for (unsigned int i = 1; i != path_size - 1; i++) {
      // 对 x（j=0）和 y（j=1）两个维度分别执行平滑更新
      for (unsigned int j = 0; j != 2; j++) {
        // x_i：原始路径点坐标（数据项参考）
        // y_i：当前平滑路径点坐标；y_m1/y_ip1：其前/后邻居坐标
        x_i = getFieldByDim(path.poses[i], j);
        y_i = getFieldByDim(new_path.poses[i], j);
        y_m1 = getFieldByDim(new_path.poses[i - 1], j);
        y_ip1 = getFieldByDim(new_path.poses[i + 1], j);
        y_i_org = y_i;

        // Smooth based on local 3 point neighborhood and original data locations
        // 核心更新公式：
        //   数据项 w_data*(x_i - y_i)：向原始位置回拉（保真）
        //   平滑项 w_smooth*(y_{i+1}+y_{i-1}-2y_i)：向两邻居中点拉（平滑）
        y_i += data_w_ * (x_i - y_i) + smooth_w_ * (y_ip1 + y_m1 - (2.0 * y_i));
        setFieldByDim(new_path.poses[i], j, y_i);
        // 累计本轮所有点的位移量，作为收敛判据
        change += abs(y_i - y_i_org);
      }

      // validate update is admissible, only checks cost if a valid costmap pointer is provided
      // 可行性校验：若提供了代价地图，把更新后的点映射到栅格并查询代价
      float cost = 0.0;
      if (costmap) {
        costmap->worldToMap(
          getFieldByDim(new_path.poses[i], 0),
          getFieldByDim(new_path.poses[i], 1),
          mx, my);
        cost = static_cast<float>(costmap->getCost(mx, my));
      }

      // 代价高于最大非障碍值（即撞上障碍/膨胀致命区）且不是未知区域：
      // 平滑引入了不可行碰撞，回退到上一轮结果并失败返回
      if (cost > nav2_costmap_2d::MAX_NON_OBSTACLE && cost != nav2_costmap_2d::NO_INFORMATION) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("SmacPlannerSmoother"),
          "Smoothing process resulted in an infeasible collision. "
          "Returning the last path before the infeasibility was introduced.");
        path = last_path;
        updateApproximatePathOrientations(path, reversing_segment);
        return false;
      }
    }

    // 本轮迭代完成：以当前结果作为下一轮的"最近可行解"
    last_path = new_path;
  }

  // Lets do additional refinement, it shouldn't take more than a couple milliseconds
  // but really puts the path quality over the top.
  // 收敛后递归精炼（最多 4 次）：以收敛结果为起点再做梯度平滑，
  // 可以进一步消除小幅抖动，显著提升路径质量
  if (do_refinement_ && refinement_ctr_ < 4) {
    refinement_ctr_++;
    smoothImpl(new_path, reversing_segment, costmap, max_time);
  }

  // 根据平滑后的位置重建路径朝向，并输出结果
  updateApproximatePathOrientations(new_path, reversing_segment);
  path = new_path;
  return true;
}

// ---------------------------------------------------------------------------
// 工具函数：按维度编号读取位姿坐标值
// dim=0 返回 x，dim=1 返回 y，其他返回 z（本模块实际只用到 0/1）
// ---------------------------------------------------------------------------
double SimpleSmoother::getFieldByDim(
  const geometry_msgs::msg::PoseStamped & msg, const unsigned int & dim)
{
  if (dim == 0) {
    return msg.pose.position.x;
  } else if (dim == 1) {
    return msg.pose.position.y;
  } else {
    return msg.pose.position.z;
  }
}

// ---------------------------------------------------------------------------
// 工具函数：按维度编号写入位姿坐标值（与 getFieldByDim 对应）
// ---------------------------------------------------------------------------
void SimpleSmoother::setFieldByDim(
  geometry_msgs::msg::PoseStamped & msg, const unsigned int dim,
  const double & value)
{
  if (dim == 0) {
    msg.pose.position.x = value;
  } else if (dim == 1) {
    msg.pose.position.y = value;
  } else {
    msg.pose.position.z = value;
  }
}

}  // namespace nav2_smoother

#include "pluginlib/class_list_macros.hpp"
// 通过 pluginlib 宏导出插件，供 SmootherServer 按参数动态加载
PLUGINLIB_EXPORT_CLASS(nav2_smoother::SimpleSmoother, nav2_core::Smoother)
