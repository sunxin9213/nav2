// Copyright (c) 2022 Samsung R&D Institute Russia
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
// limitations under the License.

// ============================================================================
// 文件说明（中文）：
// Polygon（多边形安全区基类）的实现。
// 模块定位：nav2_collision_monitor 的安全区抽象。围绕机器人定义
// 多层保护区域（本体坐标系下的多边形），配合动作类型：
//   STOP     该区内出现足够多的障碍点 -> 请求停车；
//   SLOWDOWN 区内障碍 -> 按 slowdown_ratio 减速；
//   APPROACH 以机器人真实足迹为多边形，结合 getCollisionTime 的
//            运动仿真预测碰撞时间，提前平滑降速逼近障碍。
// 核心算法：
//   - isPointInside：射线法（ray crossing）判断点在多边形内；
//   - getCollisionTime：以 simulation_time_step 步长前推机器人状态
//     （projectState），每步把障碍点变换回机器人坐标系
//     （transformPoints）检测是否进入足迹（碰撞），
//     返回首次碰撞时刻（0 = 已碰撞，-1 = 不会碰撞）。
// 子类 Circle 用圆近似（更快），VelocityPolygon 按速度方向偏移多边形。
// ============================================================================

#include "nav2_collision_monitor/polygon.hpp"

#include <exception>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include "nav2_util/node_utils.hpp"

#include "nav2_collision_monitor/kinematics.hpp"

namespace nav2_collision_monitor
{

// ---------------------------------------------------------------------------
// 构造函数：初始化多边形名称、动作类型（默认 DO_NOTHING）、
// 减速比、TF 缓冲与坐标系、TF 容差等公共成员。
// ---------------------------------------------------------------------------
Polygon::Polygon(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & polygon_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
: node_(node), polygon_name_(polygon_name), action_type_(DO_NOTHING),
  slowdown_ratio_(0.0), footprint_sub_(nullptr), tf_buffer_(tf_buffer),
  base_frame_id_(base_frame_id), transform_tolerance_(transform_tolerance)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Polygon", polygon_name_.c_str());
}

// 析构：清空多边形并注销动态参数回调
Polygon::~Polygon()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Polygon", polygon_name_.c_str());
  poly_.clear();
  dyn_params_handler_.reset();
}

// ---------------------------------------------------------------------------
// 配置：
//   1. 读取参数（动作类型、点数阈值、可视化等；APPROACH 模式还
//      会得到足迹话题并创建足迹订阅器）；
//   2. 可视化开启时预填充多边形顶点并创建 PolygonStamped 发布器；
//   3. 注册动态参数回调（enabled 可运行时修改）。
// ---------------------------------------------------------------------------
bool Polygon::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string polygon_pub_topic, footprint_topic;

  if (!getParameters(polygon_pub_topic, footprint_topic)) {
    return false;
  }

  // APPROACH 模式需要订阅机器人足迹（安全区即足迹本身）
  if (!footprint_topic.empty()) {
    footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
      node, footprint_topic, *tf_buffer_,
      base_frame_id_, tf2::durationToSec(transform_tolerance_));
  }

  if (visualize_) {
    // Fill polygon_ points for future usage
    // 预填充可视化用的多边形顶点
    std::vector<Point> poly;
    getPolygon(poly);
    for (const Point & p : poly) {
      geometry_msgs::msg::Point32 p_s;
      p_s.x = p.x;
      p_s.y = p.y;
      // p_s.z will remain 0.0
      polygon_.points.push_back(p_s);
    }

    rclcpp::QoS polygon_qos = rclcpp::SystemDefaultsQoS();  // set to default
    polygon_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>(
      polygon_pub_topic, polygon_qos);
  }

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&Polygon::dynamicParametersCallback, this, std::placeholders::_1));

  return true;
}

// 激活：激活可视化发布器（若开启）
void Polygon::activate()
{
  if (visualize_) {
    polygon_pub_->on_activate();
  }
}

// 停用：停用可视化发布器
void Polygon::deactivate()
{
  if (visualize_) {
    polygon_pub_->on_deactivate();
  }
}

// ---- 各类只读属性访问器 ----
std::string Polygon::getName() const   // 多边形配置名
{
  return polygon_name_;
}

ActionType Polygon::getActionType() const  // 触发动作类型（STOP/SLOWDOWN/APPROACH）
{
  return action_type_;
}

bool Polygon::getEnabled() const   // 是否启用
{
  return enabled_;
}

int Polygon::getMaxPoints() const  // 触发动作所需的区内最大点数阈值
{
  return max_points_;
}

double Polygon::getSlowdownRatio() const  // 减速比（SLOWDOWN 模式）
{
  return slowdown_ratio_;
}

double Polygon::getTimeBeforeCollision() const  // 碰撞预测时域（APPROACH 模式）
{
  return time_before_collision_;
}

// 返回多边形顶点（本体坐标系）
void Polygon::getPolygon(std::vector<Point> & poly) const
{
  poly = poly_;
}

// ---------------------------------------------------------------------------
// 更新多边形：若订阅了机器人足迹（APPROACH 模式），
// 用最新足迹（本体坐标系）刷新多边形顶点与可视化缓存。
// ---------------------------------------------------------------------------
void Polygon::updatePolygon()
{
  if (footprint_sub_ != nullptr) {
    // Get latest robot footprint from footprint subscriber
    // 取最新足迹并转换到本体坐标系
    std::vector<geometry_msgs::msg::Point> footprint_vec;
    std_msgs::msg::Header footprint_header;
    footprint_sub_->getFootprintInRobotFrame(footprint_vec, footprint_header);

    // 按足迹点数调整多边形大小
    std::size_t new_size = footprint_vec.size();
    poly_.resize(new_size);
    polygon_.points.resize(new_size);

    geometry_msgs::msg::Point32 p_s;
    for (std::size_t i = 0; i < new_size; i++) {
      poly_[i] = {footprint_vec[i].x, footprint_vec[i].y};
      p_s.x = footprint_vec[i].x;
      p_s.y = footprint_vec[i].y;
      polygon_.points[i] = p_s;
    }
  }
}

// ---------------------------------------------------------------------------
// 统计落在多边形内的观测点数量（逐点 isPointInside 判定）
// ---------------------------------------------------------------------------
int Polygon::getPointsInside(const std::vector<Point> & points) const
{
  int num = 0;
  for (const Point & point : points) {
    if (isPointInside(point)) {
      num++;
    }
  }
  return num;
}

// ---------------------------------------------------------------------------
// 碰撞时间预测（APPROACH 模式核心）：
// 以当前速度前推仿真机器人运动（时长上限 time_before_collision_，
// 步长 simulation_time_step_），每一步：
//   - projectState 前推位姿与速度；
//   - transformPoints 把原始障碍点变换到"预测位姿"的坐标系
//     （等价于机器人前进而障碍不动）；
//   - 检查进入足迹多边形的点数是否超过 max_points_（碰撞）。
// 返回值：0 表示当前已碰撞；t>0 表示 t 秒后碰撞（用于计算逼近减速
// 的目标速度）；-1 表示预测时域内不会碰撞。
// ---------------------------------------------------------------------------
double Polygon::getCollisionTime(
  const std::vector<Point> & collision_points,
  const Velocity & velocity) const
{
  // Initial robot pose is {0,0} in base_footprint coordinates
  // 初始位姿取本体坐标系原点
  Pose pose = {0.0, 0.0, 0.0};
  Velocity vel = velocity;

  // Array of points transformed to the frame concerned with pose on each simulation step
  // 每步仿真时被变换到当前预测位姿坐标系的障碍点
  std::vector<Point> points_transformed = collision_points;

  // Check static polygon
  // 首先检查当前时刻是否已碰撞（区内点数已达阈值）
  if (getPointsInside(points_transformed) >= max_points_) {
    return 0.0;
  }

  // Robot movement simulation
  // 运动仿真：按步长前推并逐步检测
  for (double time = 0.0; time <= time_before_collision_; time += simulation_time_step_) {
    // Shift the robot pose towards to the vel during simulation_time_step_ time interval
    // NOTE: vel is changing during the simulation
    // 前推一个时间步（速度向量也随之旋转）
    projectState(simulation_time_step_, pose, vel);
    // Transform collision_points to the frame concerned with current robot pose
    // 把障碍点变换到新位姿坐标系（等价于机器人移动到预测位置）
    points_transformed = collision_points;
    transformPoints(pose, points_transformed);
    // If the collision occurred on this stage, return the actual time before a collision
    // as if robot was moved with given velocity
    // 足迹内点数超阈值 => 碰撞，返回碰撞时刻
    if (getPointsInside(points_transformed) > max_points_) {
      return time;
    }
  }

  // There is no collision
  // 整个预测时域内未发生碰撞
  return -1.0;
}

// ---------------------------------------------------------------------------
// 发布多边形可视化消息（PolygonStamped，本体坐标系）。
// 未开启可视化时为空操作。
// ---------------------------------------------------------------------------
void Polygon::publish() const
{
  if (!visualize_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Fill PolygonStamped struct
  std::unique_ptr<geometry_msgs::msg::PolygonStamped> poly_s =
    std::make_unique<geometry_msgs::msg::PolygonStamped>();
  poly_s->header.stamp = node->now();
  poly_s->header.frame_id = base_frame_id_;
  poly_s->polygon = polygon_;

  // Publish polygon
  polygon_pub_->publish(std::move(poly_s));
}

// ---------------------------------------------------------------------------
// 读取所有多边形安全区共有的参数：
//   action_type   动作类型（stop/slowdown/approach，必填）
//   enabled       是否启用
//   max_points    触发动作的区内点数阈值（过滤噪声，默认 3）
//   slowdown_ratio      减速比（仅 SLOWDOWN，默认 0.5）
//   time_before_collision / simulation_time_step
//                      碰撞预测时域与仿真步长（仅 APPROACH）
//   visualize     是否发布可视化
//   polygon_pub_topic  可视化话题名（仅 visualize 时）
// ---------------------------------------------------------------------------
bool Polygon::getCommonParameters(std::string & polygon_pub_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  try {
    // Get action type.
    // Leave it not initialized: the will cause an error if it will not set.
    // 动作类型必填：未配置时读取会抛异常
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".action_type", rclcpp::PARAMETER_STRING);
    const std::string at_str =
      node->get_parameter(polygon_name_ + ".action_type").as_string();
    if (at_str == "stop") {
      action_type_ = STOP;
    } else if (at_str == "slowdown") {
      action_type_ = SLOWDOWN;
    } else if (at_str == "approach") {
      action_type_ = APPROACH;
    } else {  // Error if something else
      RCLCPP_ERROR(logger_, "[%s]: Unknown action type: %s", polygon_name_.c_str(), at_str.c_str());
      return false;
    }

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".enabled", rclcpp::ParameterValue(true));
    enabled_ = node->get_parameter(polygon_name_ + ".enabled").as_bool();

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".max_points", rclcpp::ParameterValue(3));
    max_points_ = node->get_parameter(polygon_name_ + ".max_points").as_int();

    // SLOWDOWN 模式专属：减速比
    if (action_type_ == SLOWDOWN) {
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".slowdown_ratio", rclcpp::ParameterValue(0.5));
      slowdown_ratio_ = node->get_parameter(polygon_name_ + ".slowdown_ratio").as_double();
    }

    // APPROACH 模式专属：碰撞预测时域与仿真步长
    if (action_type_ == APPROACH) {
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".time_before_collision", rclcpp::ParameterValue(2.0));
      time_before_collision_ =
        node->get_parameter(polygon_name_ + ".time_before_collision").as_double();
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".simulation_time_step", rclcpp::ParameterValue(0.1));
      simulation_time_step_ =
        node->get_parameter(polygon_name_ + ".simulation_time_step").as_double();
    }

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".visualize", rclcpp::ParameterValue(false));
    visualize_ = node->get_parameter(polygon_name_ + ".visualize").as_bool();
    if (visualize_) {
      // Get polygon topic parameter in case if it is going to be published
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".polygon_pub_topic", rclcpp::ParameterValue(polygon_name_));
      polygon_pub_topic = node->get_parameter(polygon_name_ + ".polygon_pub_topic").as_string();
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Error while getting common polygon parameters: %s",
      polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

// ---------------------------------------------------------------------------
// 基类参数读取：公共参数 + 多边形顶点（points 参数，[x1,y1,x2,y2,...]）。
// APPROACH 模式特殊：多边形即机器人足迹，从 footprint_topic 动态获取，
// 不从参数读取顶点。
// ---------------------------------------------------------------------------
bool Polygon::getParameters(std::string & polygon_pub_topic, std::string & footprint_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!getCommonParameters(polygon_pub_topic)) {
    return false;
  }

  try {
    if (action_type_ == APPROACH) {
      // Obtain the footprint topic to make a footprint subscription for approach polygon
      // APPROACH：读取足迹话题并创建订阅（安全区 = 足迹，稍后动态更新）
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".footprint_topic",
        rclcpp::ParameterValue("local_costmap/published_footprint"));
      footprint_topic =
        node->get_parameter(polygon_name_ + ".footprint_topic").as_string();

      // This is robot footprint: do not need to get polygon points from ROS parameters.
      // It will be set dynamically later.
      return true;
    } else {
      // Make it empty otherwise
      // 非 APPROACH：无足迹订阅
      footprint_topic.clear();
    }

    // Leave it not initialized: the will cause an error if it will not set
    // 读取多边形顶点参数（必填的 double 数组）
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".points", rclcpp::PARAMETER_DOUBLE_ARRAY);
    std::vector<double> poly_row =
      node->get_parameter(polygon_name_ + ".points").as_double_array();
    // Check for points format correctness
    // 格式校验：至少 4 个顶点（8 个数）且个数为偶数
    if (poly_row.size() <= 6 || poly_row.size() % 2 != 0) {
      RCLCPP_ERROR(
        logger_,
        "[%s]: Polygon has incorrect points description",
        polygon_name_.c_str());
      return false;
    }

    // Obtain polygon vertices
    // 相邻两个数组成一个顶点 (x, y)
    Point point;
    bool first = true;
    for (double val : poly_row) {
      if (first) {
        point.x = val;
      } else {
        point.y = val;
        poly_.push_back(point);
      }
      first = !first;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Error while getting polygon parameters: %s",
      polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

// ---------------------------------------------------------------------------
// 动态参数回调：支持运行时修改 enabled（动态启停安全区）
// ---------------------------------------------------------------------------
rcl_interfaces::msg::SetParametersResult
Polygon::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      if (param_name == polygon_name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

// ---------------------------------------------------------------------------
// 射线法（ray crossings）判断点是否在多边形内：
// 从待测点沿 X+ 方向发射水平射线，统计与多边形边的交点数；
// 交点数为奇数 => 点在多边形内。
// 参考：Shimrat, Moshe. "Algorithm 112: position of point relative to
// polygon." Communications of the ACM 5.8 (1962): 434.
// ---------------------------------------------------------------------------
inline bool Polygon::isPointInside(const Point & point) const
{
  // Adaptation of Shimrat, Moshe. "Algorithm 112: position of point relative to polygon."
  // Communications of the ACM 5.8 (1962): 434.
  // Implementation of ray crossings algorithm for point in polygon task solving.
  // Y coordinate is fixed. Moving the ray on X+ axis starting from given point.
  // Odd number of intersections with polygon boundaries means the point is inside polygon.
  const int poly_size = poly_.size();
  int i, j;  // Polygon vertex iterators
  bool res = false;  // Final result, initialized with already inverted value

  // Starting from the edge where the last point of polygon is connected to the first
  // 从"末顶点->首顶点"的闭合边开始，遍历所有边
  i = poly_size - 1;
  for (j = 0; j < poly_size; j++) {
    // Checking the edge only if given point is between edge boundaries by Y coordinates.
    // One of the condition should contain equality in order to exclude the edges
    // parallel to X+ ray.
    // 只有当边的两个端点分居射线上下（y 值跨过 point.y）时才可能相交
    if ((point.y <= poly_[i].y) == (point.y > poly_[j].y)) {
      // Calculating the intersection coordinate of X+ ray
      // 计算该边与水平射线的交点 x 坐标（线性插值）
      const double x_inter = poly_[i].x +
        (point.y - poly_[i].y) * (poly_[j].x - poly_[i].x) /
        (poly_[j].y - poly_[i].y);
      // If intersection with checked edge is greater than point.x coordinate, inverting the result
      // 交点在待测点右侧 => 计数翻转（奇偶性判定）
      if (x_inter > point.x) {
        res = !res;
      }
    }
    i = j;
  }
  return res;
}

}  // namespace nav2_collision_monitor
