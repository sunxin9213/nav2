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

#ifndef DWB_PLUGINS__KINEMATIC_PARAMETERS_HPP_
#define DWB_PLUGINS__KINEMATIC_PARAMETERS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dwb_plugins
{

/**
 * @struct KinematicParameters
 * @brief A struct containing one representation of the robot's kinematics
 * （运动学参数结构体：保存某一时刻机器人速度/加速度的可行范围。
 *  由 KinematicsHandler 管理，通过原子指针发布，
 *  供 DWB 采样器（速度迭代器）与评价器随时拷贝使用。）
 */
struct KinematicParameters
{
  friend class KinematicsHandler;  // 仅处理器可写入字段

  // X 轴（前进方向）访问器：最小速度（负=可倒车）/最大速度/加减速上限
  inline double getMinX() {return min_vel_x_;}
  inline double getMaxX() {return max_vel_x_;}
  inline double getAccX() {return acc_lim_x_;}
  inline double getDecelX() {return decel_lim_x_;}

  // Y 轴（侧向，仅全向底盘）访问器
  inline double getMinY() {return min_vel_y_;}
  inline double getMaxY() {return max_vel_y_;}
  inline double getAccY() {return acc_lim_y_;}
  inline double getDecelY() {return decel_lim_y_;}

  // 合成线速度（xy 平面模长）的死区与上限
  inline double getMinSpeedXY() {return min_speed_xy_;}
  inline double getMaxSpeedXY() {return max_speed_xy_;}

  // 角速度：范围对称 [-max, max]
  inline double getMinTheta() {return -max_vel_theta_;}
  inline double getMaxTheta() {return max_vel_theta_;}
  inline double getAccTheta() {return acc_lim_theta_;}
  inline double getDecelTheta() {return decel_lim_theta_;}
  inline double getMinSpeedTheta() {return min_speed_theta_;}

  // 平方缓存访问器（比较时免去开方）
  inline double getMinSpeedXY_SQ() {return min_speed_xy_sq_;}
  inline double getMaxSpeedXY_SQ() {return max_speed_xy_sq_;}

protected:
  // For parameter descriptions, see cfg/KinematicParams.cfg
  double min_vel_x_{0};           // X 最小线速度（负值=允许倒车）
  double min_vel_y_{0};           // Y 最小侧向速度
  double max_vel_x_{0};           // X 最大线速度
  double max_vel_y_{0};           // Y 最大侧向速度
  double base_max_vel_x_{0};      // 未限速的基准 X 最大速度（用于恢复/比例缩放）
  double base_max_vel_y_{0};      // 基准 Y 最大速度
  double max_vel_theta_{0};       // 最大角速度
  double base_max_vel_theta_{0};  // 基准最大角速度
  double min_speed_xy_{0};        // 最小合成速度死区（小于则视为停止）
  double max_speed_xy_{0};        // 最大合成速度
  double base_max_speed_xy_{0};   // 基准最大合成速度
  double min_speed_theta_{0};     // 最小角速度死区
  double acc_lim_x_{0};           // X 加速度上限
  double acc_lim_y_{0};           // Y 加速度上限
  double acc_lim_theta_{0};       // 角加速度上限
  double decel_lim_x_{0};         // X 减速度上限
  double decel_lim_y_{0};         // Y 减速度上限
  double decel_lim_theta_{0};     // 角减速度上限

  // Cached square values of min_speed_xy and max_speed_xy
  double min_speed_xy_sq_{0};     // 最小合成速度的平方（缓存）
  double max_speed_xy_sq_{0};     // 最大合成速度的平方（缓存）
};

/**
 * @class KinematicsHandler
 * @brief A class managing the representation of the robot's kinematics
 * （运动学参数管理器：负责参数声明/读取、动态更新回调与限速处理，
 *  通过原子指针 copy-on-write 地发布最新参数快照，保证并发读取安全。）
 */
class KinematicsHandler
{
public:
  KinematicsHandler();
  ~KinematicsHandler();
  void initialize(const nav2_util::LifecycleNode::SharedPtr & nh, const std::string & plugin_name);

  // 获取当前参数快照（值拷贝，线程安全）
  inline KinematicParameters getKinematics() {return *kinematics_.load();}

  // 限速接口：百分比或绝对值两种模式
  void setSpeedLimit(const double & speed_limit, const bool & percentage);

  using Ptr = std::shared_ptr<KinematicsHandler>;

protected:
  // 原子指针持有当前参数（copy-on-write：更新时整体替换）
  std::atomic<KinematicParameters *> kinematics_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;  // 动态参数回调句柄
  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  // 动态参数变更回调（见 kinematic_parameters.cpp）
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
  void update_kinematics(KinematicParameters kinematics);  // 原子替换发布新副本
  std::string plugin_name_;  // 插件名（参数命名空间前缀）
};

}  // namespace dwb_plugins

#endif  // DWB_PLUGINS__KINEMATIC_PARAMETERS_HPP_
