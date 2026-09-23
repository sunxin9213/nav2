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
// Savitzky-Golay（SG）滤波路径平滑插件的实现。
// 模块定位：nav2_smoother 的算法插件之一，实现 nav2_core::Smoother 接口。
// 算法原理：Savitzky-Golay 滤波是一种经典的时域平滑（低通）滤波方法，
// 通过对滑动窗口内的数据点做固定系数的加权卷积，在平滑噪声的同时
// 较好地保留信号的局部特征（如峰值、宽度），优于简单的移动平均。
// 本实现使用 7 点 SG 滤波核，对路径的 x、y 坐标分别卷积平滑，
// 首尾各 3 个边界点用"复制边界"的方式近似处理；可选地多次迭代精炼。
// ============================================================================

#include <vector>
#include <memory>
#include "nav2_smoother/savitzky_golay_smoother.hpp"

namespace nav2_smoother
{

using namespace smoother_utils;  // NOLINT
using namespace nav2_util::geometry_utils;  // NOLINT
using namespace std::chrono;  // NOLINT
using nav2_util::declare_parameter_if_not_declared;

// ---------------------------------------------------------------------------
// 插件配置回调：由 SmootherServer 在 on_configure 阶段调用。
// 读取本插件的两个参数：
//   do_refinement：平滑后是否再做若干轮精炼（默认 true）
//   refinement_num：精炼的迭代次数（默认 2）
// 注意：SG 平滑器不使用 TF 与代价地图（不做碰撞检查），形参仅占位。
// ---------------------------------------------------------------------------
void SavitzkyGolaySmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/*costmap_sub*/,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>/*footprint_sub*/)
{
  auto node = parent.lock();
  logger_ = node->get_logger();

  // 若参数文件中未声明这些参数，则用默认值声明（避免未配置时抛异常）
  declare_parameter_if_not_declared(
    node, name + ".do_refinement", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, name + ".refinement_num", rclcpp::ParameterValue(2));
  node->get_parameter(name + ".do_refinement", do_refinement_);
  node->get_parameter(name + ".refinement_num", refinement_num_);
}

// ---------------------------------------------------------------------------
// 平滑主入口：对整条路径执行 SG 平滑。
// 流程：
//   1. 调用 findDirectionalPathSegments 将路径按行驶方向切段
//      （避免跨过尖点把前进/倒车段拉直）；
//   2. 对长度大于 9 的每段，拷贝出来单独平滑，再拷贝回原路径；
//   3. 每段平滑前检查剩余时间预算，超时则中止并返回 false。
// ---------------------------------------------------------------------------
bool SavitzkyGolaySmoother::smooth(
  nav_msgs::msg::Path & path,
  const rclcpp::Duration & max_time)
{
  steady_clock::time_point start = steady_clock::now();
  double time_remaining = max_time.seconds();

  bool success = true, reversing_segment;
  nav_msgs::msg::Path curr_path_segment;
  curr_path_segment.header = path.header;

  // 按尖点/原地旋转将路径切分为同方向的段
  std::vector<PathSegment> path_segments = findDirectionalPathSegments(path);

  // 逐段处理；SG 滤波需要 7 点窗口，故只平滑长度 > 9 的段
  for (unsigned int i = 0; i != path_segments.size(); i++) {
    if (path_segments[i].end - path_segments[i].start > 9) {
      // Populate path segment
      // 将当前段 [start, end] 拷贝到临时路径对象
      curr_path_segment.poses.clear();
      std::copy(
        path.poses.begin() + path_segments[i].start,
        path.poses.begin() + path_segments[i].end + 1,
        std::back_inserter(curr_path_segment.poses));

      // Make sure we're still able to smooth with time remaining
      // 重新计算剩余时间预算（总时限 - 已耗时）
      steady_clock::time_point now = steady_clock::now();
      time_remaining = max_time.seconds() - duration_cast<duration<double>>(now - start).count();

      // 时间耗尽：告警并整体返回 false（表示未在时限内完成）
      if (time_remaining <= 0.0) {
        RCLCPP_WARN(
          logger_,
          "Smoothing time exceeded allowed duration of %0.2f.", max_time.seconds());
        return false;
      }

      // Smooth path segment
      // 对当前段执行 SG 滤波平滑（success 累积所有段的结果）
      success = success && smoothImpl(curr_path_segment, reversing_segment);

      // Assemble the path changes to the main path
      // 将平滑后的段写回原路径对应区间
      std::copy(
        curr_path_segment.poses.begin(),
        curr_path_segment.poses.end(),
        path.poses.begin() + path_segments[i].start);
    }
  }

  return success;
}

// ---------------------------------------------------------------------------
// SG 平滑核心实现（作用于单个方向段）：
//   - 7 点 SG 滤波系数（-2,3,6,7,6,3,-2)/21，是二次多项式拟合的最小二乘解；
//   - applyFilter：对一个 7 点窗口做加权求和（x、y 同时处理）；
//   - applyFilterOverAxes：沿路径滑动窗口。首尾各 3 个点无法取满窗口，
//     采用"边界复制"策略（用最近的边界点填充缺失的窗口位置）；
//     第 0 点与最后一点保持不动（端点固定，避免路径起终点漂移）；
//   - 可选 refine：重复应用滤波若干次以获得更平滑结果；
//   - 最后调用 updateApproximatePathOrientations 重建各点朝向。
// ---------------------------------------------------------------------------
bool SavitzkyGolaySmoother::smoothImpl(
  nav_msgs::msg::Path & path,
  bool & reversing_segment)
{
  // Must be at least 10 in length to enter function
  const unsigned int & path_size = path.poses.size();

  // 7-point SG filter
  // 7 点 Savitzky-Golay 滤波系数（对二次多项式的平滑卷积核，和为 1）
  const std::array<double, 7> filter = {
    -2.0 / 21.0,
    3.0 / 21.0,
    6.0 / 21.0,
    7.0 / 21.0,
    6.0 / 21.0,
    3.0 / 21.0,
    -2.0 / 21.0};

  // 卷积计算：对窗口内 7 个点的 x/y 分别加权求和，返回平滑后的坐标点
  auto applyFilter = [&](const std::vector<geometry_msgs::msg::Point> & data)
    -> geometry_msgs::msg::Point
    {
      geometry_msgs::msg::Point val;
      for (unsigned int i = 0; i != filter.size(); i++) {
        val.x += filter[i] * data[i].x;
        val.y += filter[i] * data[i].y;
      }
      return val;
    };

  auto applyFilterOverAxes =
    [&](std::vector<geometry_msgs::msg::PoseStamped> & plan_pts) -> void
    {
      // Handle initial boundary conditions, first point is fixed
      // 第 1 个点（下标 1）：左侧窗口不足，用第 0 个点复制填充 3 份
      unsigned int idx = 1;
      plan_pts[idx].pose.position = applyFilter(
      {
        plan_pts[idx - 1].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 2].pose.position,
        plan_pts[idx + 3].pose.position});

      // 第 2 个点：左侧缺 1 个窗口点，用 idx-2 复制填充
      idx++;
      plan_pts[idx].pose.position = applyFilter(
      {
        plan_pts[idx - 2].pose.position,
        plan_pts[idx - 2].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 2].pose.position,
        plan_pts[idx + 3].pose.position});

      // Apply nominal filter
      // 中间区域：正常滑动 7 点窗口（左右各 3 个邻点 + 自身）
      for (idx = 3; idx < path_size - 4; ++idx) {
        plan_pts[idx].pose.position = applyFilter(
        {
          plan_pts[idx - 3].pose.position,
          plan_pts[idx - 2].pose.position,
          plan_pts[idx - 1].pose.position,
          plan_pts[idx].pose.position,
          plan_pts[idx + 1].pose.position,
          plan_pts[idx + 2].pose.position,
          plan_pts[idx + 3].pose.position});
      }

      // Handle terminal boundary conditions, last point is fixed
      // 倒数第 3 个点：右侧缺 1 个窗口点，用最右侧可用点复制填充
      idx++;
      plan_pts[idx].pose.position = applyFilter(
      {
        plan_pts[idx - 3].pose.position,
        plan_pts[idx - 2].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 2].pose.position,
        plan_pts[idx + 2].pose.position});

      // 倒数第 2 个点：右侧缺 3 个窗口点，全部用最后一点复制填充
      idx++;
      plan_pts[idx].pose.position = applyFilter(
      {
        plan_pts[idx - 3].pose.position,
        plan_pts[idx - 2].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 1].pose.position});
    };

  // 对整段路径应用一遍 7 点 SG 滤波
  applyFilterOverAxes(path.poses);

  // Lets do additional refinement, it shouldn't take more than a couple milliseconds
  // 可选的多次精炼：再迭代 refinement_num_ 遍滤波，进一步提升平滑度
  if (do_refinement_) {
    for (int i = 0; i < refinement_num_; i++) {
      applyFilterOverAxes(path.poses);
    }
  }

  // 根据平滑后的位置重建路径点朝向（并输出是否倒车段）
  updateApproximatePathOrientations(path, reversing_segment);
  return true;
}

}  // namespace nav2_smoother

#include "pluginlib/class_list_macros.hpp"
// 通过 pluginlib 宏导出插件，使其可被 SmootherServer 动态加载
PLUGINLIB_EXPORT_CLASS(nav2_smoother::SavitzkyGolaySmoother, nav2_core::Smoother)
