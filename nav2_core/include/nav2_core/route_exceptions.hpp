// Copyright (c) 2023, Samsung Research America
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
// 【文件概况】路网路由异常类族 route_exceptions
// ----------------------------------------------------------------------------
// 概述: nav2_route（路网图导航系统）专用的异常类型族，全部继承自
//       RouteException。路由服务器/相关逻辑通过抛出这些细分类异常，
//       让上层能够区分失败原因并作出不同处理（重试、换边、报错等）。
// 主要内容:
//   1. RouteException:                路由异常基类（继承 std::runtime_error）
//   2. OperationFailed:               通用操作失败
//   3. NoValidRouteCouldBeFound:      图上找不到有效路由
//   4. TimedOut:                      路由计算/操作超时
//   5. RouteTFError:                  TF 变换错误（无法定位机器人到图坐标系）
//   6. NoValidGraph:                  没有可用的路由图（图未加载/为空）
//   7. IndeterminantNodesOnGraph:     起终点无法唯一映射到图上的节点
//   8. InvalidEdgeScorerUse:          边评分器（edge scorer）被错误使用
// 使用场景: nav2_route 的路由计算、节点搜索、边搜索等环节抛出；
//           ComputeRoute / ComputeAndTrackRoute Action 的服务端捕获后
//           转换为对应错误码返回。
// ============================================================================

#ifndef NAV2_CORE__ROUTE_EXCEPTIONS_HPP_
#define NAV2_CORE__ROUTE_EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>
#include <memory>

namespace nav2_core
{

// 路由异常基类: 所有 nav2_route 异常的公共父类
class RouteException : public std::runtime_error
{
public:
  // description: 人类可读的失败原因描述
  explicit RouteException(const std::string & description)
  : std::runtime_error(description) {}
};

// 通用操作失败（无法归入更具体类别的路由操作错误）
class OperationFailed : public RouteException
{
public:
  explicit OperationFailed(const std::string & description)
  : RouteException(description) {}
};

// 图上找不到有效路由: 起终点之间不存在可通行路径
class NoValidRouteCouldBeFound : public RouteException
{
public:
  explicit NoValidRouteCouldBeFound(const std::string & description)
  : RouteException(description) {}
};

// 路由计算或操作超时
class TimedOut : public RouteException
{
public:
  explicit TimedOut(const std::string & description)
  : RouteException(description) {}
};

// TF 变换错误: 无法将机器人位姿变换到路由图所在坐标系
class RouteTFError : public RouteException
{
public:
  explicit RouteTFError(const std::string & description)
  : RouteException(description) {}
};

// 没有有效的路由图（图未加载、为空或损坏）
class NoValidGraph : public RouteException
{
public:
  explicit NoValidGraph(const std::string & description)
  : RouteException(description) {}
};

// 起终点在图上的节点无法唯一确定（如落点距多个节点过近、歧义）
class IndeterminantNodesOnGraph : public RouteException
{
public:
  explicit IndeterminantNodesOnGraph(const std::string & description)
  : RouteException(description) {}
};

// 边评分器使用方式错误（如对不适用的边类型调用评分）
class InvalidEdgeScorerUse : public RouteException
{
public:
  explicit InvalidEdgeScorerUse(const std::string & description)
  : RouteException(description) {}
};

}  // namespace nav2_core

#endif  // NAV2_CORE__ROUTE_EXCEPTIONS_HPP_
