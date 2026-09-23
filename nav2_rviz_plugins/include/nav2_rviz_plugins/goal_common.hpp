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

#ifndef NAV2_RVIZ_PLUGINS__GOAL_COMMON_HPP_
#define NAV2_RVIZ_PLUGINS__GOAL_COMMON_HPP_

#include "nav2_rviz_plugins/goal_pose_updater.hpp"

namespace nav2_rviz_plugins
{
// 全局唯一的 GoalPoseUpdater 实例（定义在 nav2_panel.cpp 中）：
// GoalTool 与 Nav2 面板分属不同插件，通过这个全局对象 + Qt 信号槽解耦通信
extern GoalPoseUpdater GoalUpdater;
}  // nav2_rviz_plugins

#endif  //  NAV2_RVIZ_PLUGINS__GOAL_COMMON_HPP_
