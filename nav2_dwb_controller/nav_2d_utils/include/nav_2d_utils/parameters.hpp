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

#ifndef NAV_2D_UTILS__PARAMETERS_HPP_
#define NAV_2D_UTILS__PARAMETERS_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

// TODO(crdelsey): Remove when code is re-enabled
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
namespace nav_2d_utils
{

/**
 * @brief Search for a parameter and load it, or use the default value
 *
 * This templated function shortens a commonly used ROS pattern in which you
 * search for a parameter and get its value if it exists, otherwise returning a default value.
 *
 * @param nh NodeHandle to start the parameter search from
 * @param param_name Name of the parameter to search for
 * @param default_value Value to return if not found
 * @return Value of parameter if found, otherwise the default_value
 */

// ---------------------------------------------------------------------------
// 参数获取工具（ROS2 参数 API 学习素材）：
// ROS2（rclcpp）与 ROS1 不同：参数必须先"声明"（declare_parameter）才能
// 读取，且重复声明会抛异常。常见样板是：
//   1. 若参数未声明，用默认值声明之；
//   2. 读取参数值（此时若 YAML 中提供了覆盖值，读到的是覆盖值，
//      否则是刚声明的默认值）。
// 本模板函数把这两步打包成一行调用，模板参数 param_t 决定参数类型
// （double/int/string/bool 等）。
// ---------------------------------------------------------------------------
template<class param_t>
param_t searchAndGetParam(
  const nav2_util::LifecycleNode::SharedPtr & nh, const std::string & param_name,
  const param_t & default_value)
{
  param_t value;
  // 第一步：若未声明则用默认值声明（已声明则跳过，不抛异常）
  nav2_util::declare_parameter_if_not_declared(
    nh, param_name,
    rclcpp::ParameterValue(default_value));
  // 第二步：读取参数值（YAML 覆盖值或默认值），写入 value
  nh->get_parameter(param_name, value);
  return value;  // 返回读取到的值
}

}  // namespace nav_2d_utils
#pragma GCC diagnostic pop

#endif  // NAV_2D_UTILS__PARAMETERS_HPP_
