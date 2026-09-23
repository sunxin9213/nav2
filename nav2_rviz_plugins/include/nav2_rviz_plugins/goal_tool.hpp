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

#ifndef NAV2_RVIZ_PLUGINS__GOAL_TOOL_HPP_
#define NAV2_RVIZ_PLUGINS__GOAL_TOOL_HPP_

#include <QObject>

#include <memory>

#include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{

class DisplayContext;

namespace properties
{
class StringProperty;
}  // namespace properties
}  // namespace rviz_common

namespace nav2_rviz_plugins
{

// "Nav2 Goal" RViz 工具：继承 PoseTool（点选+拖拽定向），
// 用户设定的位姿通过全局 GoalUpdater 的 Qt 信号转发给 Nav2 面板
class RVIZ_DEFAULT_PLUGINS_PUBLIC GoalTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  GoalTool();
  ~GoalTool() override;

  // 插件初始化：设置名称与图标
  void onInitialize() override;

protected:
  // 用户设定目标点完成时回调（x/y 为坐标，theta 为朝向角）
  void onPoseSet(double x, double y, double theta) override;
};

}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__GOAL_TOOL_HPP_
