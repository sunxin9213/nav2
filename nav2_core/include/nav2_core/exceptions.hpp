/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  Copyright (c) 2019, Intel Corporation
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
// 【文件概况】导航核心异常类 exceptions
// ----------------------------------------------------------------------------
// 概述: 集中定义 nav2 各算法插件向上层 server 传播错误的标准异常类型。
//       server 捕获后统一转换为 Action 失败结果（含错误码），而不是
//       让进程崩溃。
// 主要内容:
//   1. PlannerException: 规划器插件基类异常，表示规划失败的通用原因
// 使用场景: planner 插件（navfn/smac/theta_star 等）在起点不可用、
//           目标不可达等情况抛出；ControllerException 等专用异常在各
//           server/插件包内另行定义。
// ============================================================================

#ifndef NAV2_CORE__EXCEPTIONS_HPP_
#define NAV2_CORE__EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>
#include <memory>

namespace nav2_core
{

// 规划器异常基类: 携带规划失败原因向 Planner Server 传播
class PlannerException : public std::runtime_error
{
public:
  // 携带规划失败原因向 Planner Server 传播；上层可将异常转换为 Action 失败结果。
  // description: 人类可读的失败原因描述
  explicit PlannerException(const std::string description)
  : std::runtime_error(description) {}
  // 异常的共享指针别名，便于在跨线程/异步场景传递
  using Ptr = std::shared_ptr<PlannerException>;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__EXCEPTIONS_HPP_
