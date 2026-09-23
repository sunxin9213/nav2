/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 David V. Lu!!
 *  Copyright (c) 2020, Bytes Robotics
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
// ============================================================================
// 中文注释：距离传感器层（RangeSensorLayer）实现文件
//
// 【文件功能】
// 本文件实现 Nav2 代价地图插件体系中的"距离传感器层"，用于处理声呐/
// 红外等输出"单个距离读数 + 视场角(FOV)"的传感器（sensor_msgs/Range）。
// 与激光/点云不同，这类传感器只知道"锥形视场内某处有障碍"，
// 因此本层采用概率模型：在传感器锥形视场覆盖的三角区域内，
// 用 sensor_model() 计算每个格子被占据的概率，并用贝叶斯更新
// 逐步累积/衰减置信度（本层内部存概率型代价，最终量化为
// FREE_SPACE / LETHAL_OBSTACLE 写入主地图）。
//
// 【在 Nav2 中的位置】
// LayeredCostmap 分层体系的可选感知层（Layer→CostmapLayer 继承链），
// 常用于只有廉价声呐/红外传感器的简单机器人平台。
//
// 【核心概念】
// 1. sensor_msgs/Range 消息：header(坐标系/时间)、radiation_type、
//    field_of_view(视场角)、min_range/max_range(量程)、range(读数)。
// 2. 概率代价：内部 costmap_ 存 0~255 的概率编码（to_cost/to_prob 换算，
//    0.5 概率对应"无信息"），不是标准三值代价。
// 3. sensor_model(r, phi, theta)：
//    - theta：格子相对传感器主轴的角度偏差（gamma 函数按 FOV 衰减）；
//    - phi：格子到传感器的距离（delta 函数用 tanh 刻画"距离接近读数 r
//      时置信度高"）；
//    - 输出 [0,1] 的占据概率。
// 4. 贝叶斯更新：new_prob = (sensor*prior)/((sensor*prior)+((1-sensor)*(1-prior)))，
//    多次读数反复迭代，置信度逐步收敛。
// 5. 锥形视场三角形：由传感器原点 O 与左右两条 FOV 边界端点 A/B 构成，
//    用重心坐标(orient2d)判断格子是否在三角形内；inflate_cone 可外扩。
// ============================================================================

#include <angles/angles.h>
#include <algorithm>
#include <list>
#include <limits>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_costmap_2d/range_sensor_layer.hpp"

// pluginlib 导出宏：注册 RangeSensorLayer 为 Layer 插件
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::RangeSensorLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

using namespace std::literals::chrono_literals;

namespace nav2_costmap_2d
{

RangeSensorLayer::RangeSensorLayer() {}

// ============================================================================
// onInitialize()：初始化状态、读取参数、订阅所有 Range 话题
// ============================================================================
void RangeSensorLayer::onInitialize()
{
  current_ = true;          // 层状态为最新
  was_reset_ = false;
  buffered_readings_ = 0;   // 本周期已处理的读数计数
  last_reading_time_ = clock_->now();        // 上次收到读数的时间（判超时用）
  default_value_ = to_cost(0.5);             // 默认概率 0.5 = "无信息"

  matchSize();    // 与主地图尺寸对齐
  resetRange();   // 复位更新边界

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // ---- 声明并读取层私有参数 ----
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  declareParameter("phi", rclcpp::ParameterValue(1.2));                        // delta 函数的中心参数
  node->get_parameter(name_ + "." + "phi", phi_v_);
  declareParameter("inflate_cone", rclcpp::ParameterValue(1.0));               // 锥形视场外扩比例
  node->get_parameter(name_ + "." + "inflate_cone", inflate_cone_);
  declareParameter("no_readings_timeout", rclcpp::ParameterValue(0.0));        // 无读数超时（秒）
  node->get_parameter(name_ + "." + "no_readings_timeout", no_readings_timeout_);
  declareParameter("clear_threshold", rclcpp::ParameterValue(0.2));            // 判自由的概率下限
  node->get_parameter(name_ + "." + "clear_threshold", clear_threshold_);
  declareParameter("mark_threshold", rclcpp::ParameterValue(0.8));             // 判障碍的概率上限
  node->get_parameter(name_ + "." + "mark_threshold", mark_threshold_);
  declareParameter("clear_on_max_reading", rclcpp::ParameterValue(false));     // 最大量程读数是否触发清除
  node->get_parameter(name_ + "." + "clear_on_max_reading", clear_on_max_reading_);

  // TF 容差（全局参数）
  double temp_tf_tol = 0.0;
  node->get_parameter("transform_tolerance", temp_tf_tol);
  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  // 要订阅的 Range 话题列表
  std::vector<std::string> topic_names{};
  declareParameter("topics", rclcpp::ParameterValue(topic_names));
  node->get_parameter(name_ + "." + "topics", topic_names);

  // 输入传感器类型：VARIABLE(量程型)/FIXED(阈值型)/ALL(按消息自动判断)
  InputSensorType input_sensor_type = InputSensorType::ALL;
  std::string sensor_type_name;
  declareParameter("input_sensor_type", rclcpp::ParameterValue("ALL"));
  node->get_parameter(name_ + "." + "input_sensor_type", sensor_type_name);

  // 统一转大写后比较
  std::transform(
    sensor_type_name.begin(), sensor_type_name.end(),
    sensor_type_name.begin(), ::toupper);
  RCLCPP_INFO(
    logger_, "%s: %s as input_sensor_type given",
    name_.c_str(), sensor_type_name.c_str());

  if (sensor_type_name == "VARIABLE") {
    input_sensor_type = InputSensorType::VARIABLE;
  } else if (sensor_type_name == "FIXED") {
    input_sensor_type = InputSensorType::FIXED;
  } else if (sensor_type_name == "ALL") {
    input_sensor_type = InputSensorType::ALL;
  } else {
    RCLCPP_ERROR(
      logger_, "%s: Invalid input sensor type: %s. Defaulting to ALL.",
      name_.c_str(), sensor_type_name.c_str());
  }

  // Validate topic names list: it must be a (normally non-empty) list of strings
  // 话题列表必须非空
  if (topic_names.empty()) {
    RCLCPP_FATAL(
      logger_, "Invalid topic names list: it must"
      "be a non-empty list of strings");
    return;
  }

  // Traverse the topic names list subscribing to all of them with the same callback method
  // 遍历话题列表，为每个话题创建订阅（回调统一为 bufferIncomingRangeMsg）
  for (auto & topic_name : topic_names) {
    // 按传感器类型选择消息处理函数（std::function 多态分发）
    if (input_sensor_type == InputSensorType::VARIABLE) {
      processRangeMessageFunc_ = std::bind(
        &RangeSensorLayer::processVariableRangeMsg, this,
        std::placeholders::_1);
    } else if (input_sensor_type == InputSensorType::FIXED) {
      processRangeMessageFunc_ = std::bind(
        &RangeSensorLayer::processFixedRangeMsg, this,
        std::placeholders::_1);
    } else if (input_sensor_type == InputSensorType::ALL) {
      processRangeMessageFunc_ = std::bind(
        &RangeSensorLayer::processRangeMsg, this,
        std::placeholders::_1);
    } else {
      RCLCPP_ERROR(
        logger_,
        "%s: Invalid input sensor type: %s. Did you make a new type"
        "and forgot to choose the subscriber for it?",
        name_.c_str(), sensor_type_name.c_str());
    }
    // SensorDataQoS：best-effort，适配传感器数据流
    range_subs_.push_back(
      node->create_subscription<sensor_msgs::msg::Range>(
        topic_name, rclcpp::SensorDataQoS(), std::bind(
          &RangeSensorLayer::bufferIncomingRangeMsg, this,
          std::placeholders::_1)));

    RCLCPP_INFO(
      logger_, "RangeSensorLayer: subscribed to "
      "topic %s", range_subs_.back()->get_topic_name());
  }
  global_frame_ = layered_costmap_->getGlobalFrameID();
}


// ============================================================================
// gamma(theta)：角度偏差衰减函数——格子偏离传感器主轴越远置信度越低
// |theta| 超出半视场角 max_angle_ 时返回 0（视场外完全不可信）
// ============================================================================
double RangeSensorLayer::gamma(double theta)
{
  if (fabs(theta) > max_angle_) {
    return 0.0;
  } else {
    // 视场内：1 - (theta/max_angle)^2，主轴处为 1，边缘处为 0
    return 1 - pow(theta / max_angle_, 2);
  }
}

// ============================================================================
// delta(phi)：距离衰减函数——用 tanh 刻画"格子距离 phi 接近
// 预设读数 phi_v_ 时置信度趋近 1，远离时趋近 0"
// ============================================================================
double RangeSensorLayer::delta(double phi)
{
  return 1 - (1 + tanh(2 * (phi - phi_v_))) / 2;
}

// ============================================================================
// get_deltas()：按角度计算一个栅格步长对应的 dx/dy（未在主流程使用）
// ============================================================================
void RangeSensorLayer::get_deltas(double angle, double * dx, double * dy)
{
  double ta = tan(angle);
  if (ta == 0) {
    *dx = 0;
  } else {
    *dx = resolution_ / ta;
  }

  *dx = copysign(*dx, cos(angle));
  *dy = copysign(resolution_, sin(angle));
}

// ============================================================================
// sensor_model(r, phi, theta)：声呐/红外占据概率模型
// r=传感器读数（米），phi=格子到传感器距离，theta=格子相对主轴角度
// 输出该格被占据的概率 ∈ [0,1]，0.5 表示无信息
// 分段含义：
//   phi < r-2δr：远在读数前方 → (1-λ)*0.5（略偏无信息）
//   r-2δr ≤ phi < r-δr：过渡区，二次插值
//   r-δr ≤ phi < r+δr：读数附近 → 峰值置信 λ
//   phi ≥ r+δr：读数之后 → 0.5（无信息，锥内远处不可判断）
// 其中 λ = delta(phi)*gamma(theta) 综合角度与距离置信度
// ============================================================================
double RangeSensorLayer::sensor_model(double r, double phi, double theta)
{
  double lbda = delta(phi) * gamma(theta);

  double delta = resolution_;

  if (phi >= 0.0 && phi < r - 2 * delta * r) {
    return (1 - lbda) * (0.5);
  } else if (phi < r - delta * r) {
    return lbda * 0.5 * pow((phi - (r - 2 * delta * r)) / (delta * r), 2) +
           (1 - lbda) * .5;
  } else if (phi < r + delta * r) {
    double J = (r - phi) / (delta * r);
    return lbda * ((1 - (0.5) * pow(J, 2)) - 0.5) + 0.5;
  } else {
    return 0.5;
  }
}

// ============================================================================
// bufferIncomingRangeMsg()：订阅回调——仅把消息压入缓冲队列
// （线程安全：互斥锁保护；真正的处理推迟到更新周期，避免回调阻塞）
// ============================================================================
void RangeSensorLayer::bufferIncomingRangeMsg(
  const sensor_msgs::msg::Range::SharedPtr range_message)
{
  range_message_mutex_.lock();
  range_msgs_buffer_.push_back(*range_message);
  range_message_mutex_.unlock();
}

// ============================================================================
// updateCostmap()：取出缓冲的全部消息并逐条处理
// （交换出副本后立即解锁，缩短临界区）
// ============================================================================
void RangeSensorLayer::updateCostmap()
{
  std::list<sensor_msgs::msg::Range> range_msgs_buffer_copy;

  range_message_mutex_.lock();
  range_msgs_buffer_copy = std::list<sensor_msgs::msg::Range>(range_msgs_buffer_);
  range_msgs_buffer_.clear();
  range_message_mutex_.unlock();

  for (auto & range_msgs_it : range_msgs_buffer_copy) {
    processRangeMessageFunc_(range_msgs_it);
  }
}

// ============================================================================
// processRangeMsg()：ALL 模式——按 min_range==max_range 自动判断传感器类型
// ============================================================================
void RangeSensorLayer::processRangeMsg(sensor_msgs::msg::Range & range_message)
{
  if (range_message.min_range == range_message.max_range) {
    processFixedRangeMsg(range_message);
  } else {
    processVariableRangeMsg(range_message);
  }
}

// ============================================================================
// processFixedRangeMsg()：FIXED 型（阈值式）传感器处理
// min_range==max_range：读数只有 ±Inf 两种合法值
//   +Inf = 没检测到物体；-Inf = 检测到物体（距离即 min/max）
// ============================================================================
void RangeSensorLayer::processFixedRangeMsg(sensor_msgs::msg::Range & range_message)
{
  // 非无穷的读数非法
  if (!std::isinf(range_message.range)) {
    RCLCPP_ERROR(
      logger_,
      "Fixed distance ranger (min_range == max_range) in frame %s sent invalid value. "
      "Only -Inf (== object detected) and Inf (== no object detected) are valid.",
      range_message.header.frame_id.c_str());
    return;
  }

  bool clear_sensor_cone = false;

  if (range_message.range > 0) {  // +inf
    // +Inf（没检测到）：默认不做任何事；
    // clear_on_max_reading_=true 时把整个视锥当作自由空间清除
    if (!clear_on_max_reading_) {
      return;  // no clearing at all
    }
    clear_sensor_cone = true;
  }

  // 把读数统一设为传感器的固定距离值
  range_message.range = range_message.min_range;

  updateCostmap(range_message, clear_sensor_cone);
}

// ============================================================================
// processVariableRangeMsg()：VARIABLE 型（量程式）传感器处理
// 读数必须在 [min_range, max_range] 内才有效
// ============================================================================
void RangeSensorLayer::processVariableRangeMsg(sensor_msgs::msg::Range & range_message)
{
  // 超出量程的读数直接丢弃
  if (range_message.range < range_message.min_range || range_message.range >
    range_message.max_range)
  {
    return;
  }

  bool clear_sensor_cone = false;

  // 读到最大量程通常意味着"锥内无障碍"，可配置为触发视锥清除
  if (range_message.range >= range_message.max_range && clear_on_max_reading_) {
    clear_sensor_cone = true;
  }

  updateCostmap(range_message, clear_sensor_cone);
}

// ============================================================================
// updateCostmap(range_message, clear_sensor_cone)：处理单条读数的核心
// 1) TF 把传感器位置与目标点变换到全局坐标系；
// 2) 计算视锥三角形的顶点与更新包围盒；
// 3) 遍历包围盒内的格子，对三角形内（含外扩）的格子做概率更新
// ============================================================================
void RangeSensorLayer::updateCostmap(
  sensor_msgs::msg::Range & range_message,
  bool clear_sensor_cone)
{
  // 半视场角 = FOV / 2
  max_angle_ = range_message.field_of_view / 2;

  geometry_msgs::msg::PointStamped in, out;
  // ---- 第一次 TF 变换：传感器原点位置 ----
  in.header.stamp = range_message.header.stamp;
  in.header.frame_id = range_message.header.frame_id;

  // 检查 TF 可用性（时间戳 + 容差），不可用则丢弃该读数
  if (!tf_->canTransform(
      in.header.frame_id, global_frame_,
      tf2_ros::fromMsg(in.header.stamp),
      tf2_ros::fromRclcpp(transform_tolerance_)))
  {
    RCLCPP_INFO(
      logger_, "Range sensor layer can't transform from %s to %s",
      global_frame_.c_str(), in.header.frame_id.c_str());
    return;
  }

  tf_->transform(in, out, global_frame_, transform_tolerance_);

  // 传感器原点（全局坐标）
  double ox = out.point.x, oy = out.point.y;

  // ---- 第二次 TF 变换：沿传感器主轴方向 distance=range 处的目标点 ----
  in.point.x = range_message.range;

  tf_->transform(in, out, global_frame_, transform_tolerance_);

  double tx = out.point.x, ty = out.point.y;

  // calculate target props
  // 视线方向角 theta 与测距距离 d
  double dx = tx - ox, dy = ty - oy, theta = atan2(dy, dx), d = sqrt(dx * dx + dy * dy);

  // Integer Bounds of Update
  // 更新区域的整数栅格包围盒
  int bx0, by0, bx1, by1;

  // Triangle that will be really updated; the other cells within bounds are ignored
  // This triangle is formed by the origin and left and right sides of sonar cone
  // 视锥三角形三个顶点（栅格坐标）：O=传感器原点，A/B=左右 FOV 边界端点
  int Ox, Oy, Ax, Ay, Bx, By;

  // Bounds includes the origin
  // 原点换算为栅格坐标（允许越界，worldToMapNoBounds），并扩展更新边界
  worldToMapNoBounds(ox, oy, Ox, Oy);
  bx1 = bx0 = Ox;
  by1 = by0 = Oy;
  touch(ox, oy, &min_x_, &min_y_, &max_x_, &max_y_);

  // Update Map with Target Point
  // 目标点（读数处）写入中等置信代价 233 并扩展边界
  unsigned int aa, ab;
  if (worldToMap(tx, ty, aa, ab)) {
    setCost(aa, ab, 233);
    touch(tx, ty, &min_x_, &min_y_, &max_x_, &max_y_);
  }

  double mx, my;

  // Update left side of sonar cone
  // 左边界端点：沿 (theta - 半FOV) 方向、1.2 倍测距处（略外扩）
  mx = ox + cos(theta - max_angle_) * d * 1.2;
  my = oy + sin(theta - max_angle_) * d * 1.2;
  worldToMapNoBounds(mx, my, Ax, Ay);
  bx0 = std::min(bx0, Ax);
  bx1 = std::max(bx1, Ax);
  by0 = std::min(by0, Ay);
  by1 = std::max(by1, Ay);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  // Update right side of sonar cone
  // 右边界端点：沿 (theta + 半FOV) 方向
  mx = ox + cos(theta + max_angle_) * d * 1.2;
  my = oy + sin(theta + max_angle_) * d * 1.2;

  worldToMapNoBounds(mx, my, Bx, By);
  bx0 = std::min(bx0, Bx);
  bx1 = std::max(bx1, Bx);
  by0 = std::min(by0, By);
  by1 = std::max(by1, By);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  // Limit Bounds to Grid
  // 包围盒裁剪到栅格范围内
  bx0 = std::max(0, bx0);
  by0 = std::max(0, by0);
  bx1 = std::min(static_cast<int>(size_x_), bx1);
  by1 = std::min(static_cast<int>(size_y_), by1);

  // 遍历包围盒内的所有格子
  for (unsigned int x = bx0; x <= (unsigned int)bx1; x++) {
    for (unsigned int y = by0; y <= (unsigned int)by1; y++) {
      bool update_xy_cell = true;

      // Unless inflate_cone_ is set to 100 %, we update cells only within the
      // (partially inflated) sensor cone, projected on the costmap as a triangle.
      // 0 % corresponds to just the triangle, but if your sensor fov is very
      // narrow, the covered area can become zero due to cell discretization.
      // See wiki description for more details
      // inflate_cone_ < 1.0 时只更新三角形（含外扩比例）内的格子：
      // 用重心坐标判断点是否在三角形 OAB 内
      if (inflate_cone_ < 1.0) {
        // Determine barycentric coordinates
        // orient2d 计算点相对三角形各边的有向面积（重心坐标分子）
        int w0 = orient2d(Ax, Ay, Bx, By, x, y);
        int w1 = orient2d(Bx, By, Ox, Oy, x, y);
        int w2 = orient2d(Ox, Oy, Ax, Ay, x, y);

        // Barycentric coordinates inside area threshold; this is not mathematically
        // sound at all, but it works!
        // 阈值按外扩比例缩放三角形面积（工程近似）
        float bcciath = -static_cast<float>(inflate_cone_) * area(Ax, Ay, Bx, By, Ox, Oy);
        update_xy_cell = w0 >= bcciath && w1 >= bcciath && w2 >= bcciath;
      }

      if (update_xy_cell) {
        // 栅格坐标 → 世界坐标，做单格概率更新
        double wx, wy;
        mapToWorld(x, y, wx, wy);
        update_cell(ox, oy, theta, range_message.range, wx, wy, clear_sensor_cone);
      }
    }
  }

  buffered_readings_++;              // 读数计数
  last_reading_time_ = clock_->now(); // 刷新最近读数时间
}

// ============================================================================
// update_cell()：单个格子的贝叶斯概率更新
// sensor_model 给出本次读数下该格的占据概率，
// 与格子的先验概率做二项贝叶斯更新后写回
// ============================================================================
void RangeSensorLayer::update_cell(
  double ox, double oy, double ot, double r,
  double nx, double ny, bool clear)
{
  unsigned int x, y;
  if (worldToMap(nx, ny, x, y)) {
    // 格子相对传感器的极坐标：角度偏差 theta、距离 phi
    double dx = nx - ox, dy = ny - oy;
    double theta = atan2(dy, dx) - ot;
    theta = angles::normalize_angle(theta);   // 归一化到 [-π, π]
    double phi = sqrt(dx * dx + dy * dy);
    double sensor = 0.0;
    if (!clear) {
      sensor = sensor_model(r, phi, theta);   // 非清除模式：查概率模型
    }
    // 清除模式 sensor=0：本次读数认为该格"不占据"
    double prior = to_prob(getCost(x, y));    // 先验概率（从存储的代价解码）
    // 贝叶斯二项更新（对数几率形式的等价表达）：
    //   P(occ|z) = sensor*prior / (sensor*prior + (1-sensor)*(1-prior))
    double prob_occ = sensor * prior;
    double prob_not = (1 - sensor) * (1 - prior);
    double new_prob = prob_occ / (prob_occ + prob_not);

    RCLCPP_DEBUG(
      logger_,
      "%f %f | %f %f = %f", dx, dy, theta, phi, sensor);
    RCLCPP_DEBUG(
      logger_,
      "%f | %f %f | %f", prior, prob_occ, prob_not, new_prob);
    // 概率编码回 0~255 代价写回本层地图
    unsigned char c = to_cost(new_prob);
    setCost(x, y, c);
  }
}

// ============================================================================
// resetRange()：复位累计的更新边界为极值（下一周期重新累计）
// ============================================================================
void RangeSensorLayer::resetRange()
{
  min_x_ = min_y_ = std::numeric_limits<double>::max();
  max_x_ = max_y_ = -std::numeric_limits<double>::max();
}

// ============================================================================
// updateBounds()：分层更新第一阶段——处理缓冲读数并合并更新边界
// ============================================================================
void RangeSensorLayer::updateBounds(
  double robot_x, double robot_y,
  double robot_yaw, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  robot_yaw = 0 + robot_yaw;  // Avoid error if variable not in use
  // 滚动窗口：原点平移到机器人中心
  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  // 处理缓冲区中的所有读数（会更新 min_x_ 等累计边界）
  updateCostmap();

  // 把累计边界合并进全局边界
  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);

  resetRange();   // 复位累计边界

  if (!enabled_) {
    current_ = true;
    return;
  }

  // 传感器超时检测：超过 no_readings_timeout_ 秒没有读数则标记不最新
  if (buffered_readings_ == 0) {
    if (no_readings_timeout_ > 0.0 &&
      (clock_->now() - last_reading_time_).seconds() >
      no_readings_timeout_)
    {
      RCLCPP_WARN(
        logger_,
        "No range readings received for %.2f seconds, while expected at least every %.2f seconds.",
        (clock_->now() - last_reading_time_).seconds(),
        no_readings_timeout_);
      current_ = false;
    }
  }
}

// ============================================================================
// updateCosts()：分层更新第二阶段——把概率代价量化后写入主地图
// 概率 > mark_threshold_ → LETHAL_OBSTACLE；
// 概率 < clear_threshold_ → FREE_SPACE；
// 介于两者之间 → 不改动主地图（保持不确定）
// ============================================================================
void RangeSensorLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  // 主地图一维数组与行跨度（span = size_x）
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  // 概率阈值换算成 0~255 编码
  unsigned char clear = to_cost(clear_threshold_), mark = to_cost(mark_threshold_);

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;   // 行首一维下标：it = i + j*span
    for (int i = min_i; i < max_i; i++) {
      unsigned char prob = costmap_[it];
      unsigned char current;
      if (prob == nav2_costmap_2d::NO_INFORMATION) {
        // 本层该格无信息：跳过，不覆盖主地图
        it++;
        continue;
      } else if (prob > mark) {
        // 高置信占据 → 致命障碍
        current = nav2_costmap_2d::LETHAL_OBSTACLE;
      } else if (prob < clear) {
        // 高置信自由 → 自由空间
        current = nav2_costmap_2d::FREE_SPACE;
      } else {
        // 置信度不足：跳过
        it++;
        continue;
      }

      // 只在主地图为未知或低于新代价时写入（避免降低已有危险等级）
      unsigned char old_cost = master_array[it];

      if (old_cost == NO_INFORMATION || old_cost < current) {
        master_array[it] = current;
      }
      it++;
    }
  }

  buffered_readings_ = 0;   // 清零读数计数（下一周期重新统计）

  // if not current due to reset, set current now after clearing
  // 刚复位过：写入完成后恢复"最新"状态
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }
}

// ============================================================================
// reset()：复位（去激活→清地图→重新激活）
// ============================================================================
void RangeSensorLayer::reset()
{
  RCLCPP_DEBUG(logger_, "Reseting range sensor layer...");
  deactivate();
  resetMaps();
  was_reset_ = true;
  activate();
}

// deactivate()：生命周期去激活——清空读数缓冲
void RangeSensorLayer::deactivate()
{
  range_msgs_buffer_.clear();
}

// activate()：生命周期激活——同样清空缓冲（丢弃停机期旧数据）
void RangeSensorLayer::activate()
{
  range_msgs_buffer_.clear();
}

}  // namespace nav2_costmap_2d
