/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "dwb_plugins/kinematic_parameters.hpp"

#include <memory>
#include <string>
#include <vector>

#include "nav_2d_utils/parameters.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace dwb_plugins
{

// 构造：为原子指针分配初始运动学参数对象
// kinematics_ 使用原子指针实现读写并发安全（copy-on-write 模式）
KinematicsHandler::KinematicsHandler()
{
  kinematics_.store(new KinematicParameters);
}

// 析构：释放当前持有的运动学参数对象
KinematicsHandler::~KinematicsHandler()
{
  delete kinematics_.load();
}

// 初始化：声明/读取全部运动学参数，并注册动态参数回调
void KinematicsHandler::initialize(
  const nav2_util::LifecycleNode::SharedPtr & nh,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;

  // 声明速度上下限参数（min_vel_x 为负表示允许倒车；
  // min_vel_y 仅全向底盘使用；theta 为角速度上限）
  declare_parameter_if_not_declared(nh, plugin_name + ".min_vel_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".min_vel_y", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".max_vel_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".max_vel_y", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".max_vel_theta",
    rclcpp::ParameterValue(0.0));
  // 声明合成速度/角速度的死区与上限参数
  declare_parameter_if_not_declared(
    nh, plugin_name + ".min_speed_xy",
    rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".max_speed_xy",
    rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".min_speed_theta",
    rclcpp::ParameterValue(0.0));
  // 声明加速度/减速度上限参数（采样窗口的物理约束来源）
  declare_parameter_if_not_declared(nh, plugin_name + ".acc_lim_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".acc_lim_y", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".acc_lim_theta",
    rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".decel_lim_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".decel_lim_y", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".decel_lim_theta",
    rclcpp::ParameterValue(0.0));

  KinematicParameters kinematics;

  // 读取所有参数值到局部结构体
  nh->get_parameter(plugin_name + ".min_vel_x", kinematics.min_vel_x_);
  nh->get_parameter(plugin_name + ".min_vel_y", kinematics.min_vel_y_);
  nh->get_parameter(plugin_name + ".max_vel_x", kinematics.max_vel_x_);
  nh->get_parameter(plugin_name + ".max_vel_y", kinematics.max_vel_y_);
  nh->get_parameter(plugin_name + ".max_vel_theta", kinematics.max_vel_theta_);
  nh->get_parameter(plugin_name + ".min_speed_xy", kinematics.min_speed_xy_);
  nh->get_parameter(plugin_name + ".max_speed_xy", kinematics.max_speed_xy_);
  nh->get_parameter(plugin_name + ".min_speed_theta", kinematics.min_speed_theta_);
  nh->get_parameter(plugin_name + ".acc_lim_x", kinematics.acc_lim_x_);
  nh->get_parameter(plugin_name + ".acc_lim_y", kinematics.acc_lim_y_);
  nh->get_parameter(plugin_name + ".acc_lim_theta", kinematics.acc_lim_theta_);
  nh->get_parameter(plugin_name + ".decel_lim_x", kinematics.decel_lim_x_);
  nh->get_parameter(plugin_name + ".decel_lim_y", kinematics.decel_lim_y_);
  nh->get_parameter(plugin_name + ".decel_lim_theta", kinematics.decel_lim_theta_);

  // 记录"基准"最大速度：限速后仍可用它们按比例恢复/缩放
  kinematics.base_max_vel_x_ = kinematics.max_vel_x_;
  kinematics.base_max_vel_y_ = kinematics.max_vel_y_;
  kinematics.base_max_speed_xy_ = kinematics.max_speed_xy_;
  kinematics.base_max_vel_theta_ = kinematics.max_vel_theta_;

  // Add callback for dynamic parameters
  // 注册动态参数回调（运行期调参）
  dyn_params_handler_ = nh->add_on_set_parameters_callback(
    std::bind(&KinematicsHandler::dynamicParametersCallback, this, _1));

  // 缓存速度平方值，避免评价器内循环反复开方/乘法
  kinematics.min_speed_xy_sq_ = kinematics.min_speed_xy_ * kinematics.min_speed_xy_;
  kinematics.max_speed_xy_sq_ = kinematics.max_speed_xy_ * kinematics.max_speed_xy_;

  update_kinematics(kinematics);  // 原子替换发布
}

// 限速接口：由速度限制过滤器调用。
// percentage=true 时 speed_limit 为百分比；false 时为绝对线速度上限
void KinematicsHandler::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  // 先复制一份当前参数（copy-on-write）
  KinematicParameters kinematics(*kinematics_.load());

  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    // 特殊值：解除限速，恢复基准参数
    kinematics.max_speed_xy_ = kinematics.base_max_speed_xy_;
    kinematics.max_vel_x_ = kinematics.base_max_vel_x_;
    kinematics.max_vel_y_ = kinematics.base_max_vel_y_;
    kinematics.max_vel_theta_ = kinematics.base_max_vel_theta_;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      // 百分比模式：各分量按比例缩放
      kinematics.max_speed_xy_ = kinematics.base_max_speed_xy_ * speed_limit / 100.0;
      kinematics.max_vel_x_ = kinematics.base_max_vel_x_ * speed_limit / 100.0;
      kinematics.max_vel_y_ = kinematics.base_max_vel_y_ * speed_limit / 100.0;
      kinematics.max_vel_theta_ = kinematics.base_max_vel_theta_ * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      // 绝对值模式：仅当新上限更小时生效
      if (speed_limit < kinematics.base_max_speed_xy_) {
        kinematics.max_speed_xy_ = speed_limit;
        // Handling components and angular velocity changes:
        // Max velocities are being changed in the same proportion
        // as absolute linear speed changed in order to preserve
        // robot moving trajectories to be the same after speed change.
        // 各分量（含角速度）按同一比例缩放，保证限速前后轨迹形状不变
        const double ratio = speed_limit / kinematics.base_max_speed_xy_;
        kinematics.max_vel_x_ = kinematics.base_max_vel_x_ * ratio;
        kinematics.max_vel_y_ = kinematics.base_max_vel_y_ * ratio;
        kinematics.max_vel_theta_ = kinematics.base_max_vel_theta_ * ratio;
      }
    }
  }

  // Do not forget to update max_speed_xy_sq_ as well
  // 同步更新缓存的速度平方值
  kinematics.max_speed_xy_sq_ = kinematics.max_speed_xy_ * kinematics.max_speed_xy_;

  update_kinematics(kinematics);
}

// 动态参数回调：逐个匹配参数名并更新对应字段
rcl_interfaces::msg::SetParametersResult
KinematicsHandler::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  KinematicParameters kinematics(*kinematics_.load());  // 复制当前值

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // 只处理 double 类型；max_* 更新时同步基准值
    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".min_vel_x") {
        kinematics.min_vel_x_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_vel_y") {
        kinematics.min_vel_y_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_vel_x") {
        kinematics.max_vel_x_ = parameter.as_double();
        kinematics.base_max_vel_x_ = kinematics.max_vel_x_;  // 视为新基准
      } else if (name == plugin_name_ + ".max_vel_y") {
        kinematics.max_vel_y_ = parameter.as_double();
        kinematics.base_max_vel_y_ = kinematics.max_vel_y_;
      } else if (name == plugin_name_ + ".max_vel_theta") {
        kinematics.max_vel_theta_ = parameter.as_double();
        kinematics.base_max_vel_theta_ = kinematics.max_vel_theta_;
      } else if (name == plugin_name_ + ".min_speed_xy") {
        kinematics.min_speed_xy_ = parameter.as_double();
        kinematics.min_speed_xy_sq_ = kinematics.min_speed_xy_ * kinematics.min_speed_xy_;
      } else if (name == plugin_name_ + ".max_speed_xy") {
        kinematics.max_speed_xy_ = parameter.as_double();
        kinematics.base_max_speed_xy_ = kinematics.max_speed_xy_;
      } else if (name == plugin_name_ + ".min_speed_theta") {
        kinematics.min_speed_theta_ = parameter.as_double();
        kinematics.max_speed_xy_sq_ = kinematics.max_speed_xy_ * kinematics.max_speed_xy_;
      } else if (name == plugin_name_ + ".acc_lim_x") {
        kinematics.acc_lim_x_ = parameter.as_double();
      } else if (name == plugin_name_ + ".acc_lim_y") {
        kinematics.acc_lim_y_ = parameter.as_double();
      } else if (name == plugin_name_ + ".acc_lim_theta") {
        kinematics.acc_lim_theta_ = parameter.as_double();
      } else if (name == plugin_name_ + ".decel_lim_x") {
        kinematics.decel_lim_x_ = parameter.as_double();
      } else if (name == plugin_name_ + ".decel_lim_y") {
        kinematics.decel_lim_y_ = parameter.as_double();
      } else if (name == plugin_name_ + ".decel_lim_theta") {
        kinematics.decel_lim_theta_ = parameter.as_double();
      }
    }
  }
  update_kinematics(kinematics);  // 原子发布更新后的副本
  result.successful = true;
  return result;
}

// 原子替换：释放旧对象指针，存入新副本。
// 读方通过 getKinematics() 拿到的是旧副本的拷贝，永不悬空
void KinematicsHandler::update_kinematics(KinematicParameters kinematics)
{
  delete kinematics_.load();
  kinematics_.store(new KinematicParameters(kinematics));
}

}  // namespace dwb_plugins
