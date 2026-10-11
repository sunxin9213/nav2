// Copyright (c) 2019 Intel Corporation
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
// 【文件概况】"完全清除代价地图"服务客户端（特化封装）
// ----------------------------------------------------------------------------
// 概述: 继承通用 ServiceClient 模板并特化为 nav2_msgs/srv/ClearEntireCostmap
//       服务的薄封装类，仅提供类型别名来简化调用方代码。
// 主要内容:
//   1. 构造函数: 直接把服务名转发给基类
//   2. clearEntirelyCostmapServiceRequest/Response: 请求/响应类型别名
// 使用场景: 行为树 ClearCostmap 类动作节点调用 costmap 服务器的
//           clear_entirely_costmap 服务时使用。
// ============================================================================

#ifndef NAV2_UTIL__CLEAR_ENTIRELY_COSTMAP_SERVICE_CLIENT_HPP_
#define NAV2_UTIL__CLEAR_ENTIRELY_COSTMAP_SERVICE_CLIENT_HPP_

#include <string>
#include "nav2_util/service_client.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

namespace nav2_util
{
/**
 * @class nav2_util::ClearEntirelyCostmapServiceClient
 * @brief A service client to clear costmaps entirely
 * "完全清除代价地图"服务客户端：继承通用 ServiceClient 模板并特化为
 * nav2_msgs::srv::ClearEntireCostmap 服务，用于请求 costmap 服务器
 * 清空指定代价地图的全部数据（请求/响应均为空字段）。
 */
class ClearEntirelyCostmapServiceClient
  : public nav2_util::ServiceClient<nav2_msgs::srv::ClearEntireCostmap>
{
public:
  /**
* @brief A constructor for nav2_util::ClearEntirelyCostmapServiceClient
   */
  // 直接把服务名转发给基类构造函数
  explicit ClearEntirelyCostmapServiceClient(const std::string & service_name)
  : nav2_util::ServiceClient<nav2_msgs::srv::ClearEntireCostmap>(service_name)
  {
  }

  // 类型别名简化：从基类提取该服务的 Request/Response 类型
  using clearEntirelyCostmapServiceRequest =
    nav2_util::ServiceClient<nav2_msgs::srv::ClearEntireCostmap>::RequestType;
  using clearEntirelyCostmapServiceResponse =
    nav2_util::ServiceClient<nav2_msgs::srv::ClearEntireCostmap>::ResponseType;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__CLEAR_ENTIRELY_COSTMAP_SERVICE_CLIENT_HPP_
