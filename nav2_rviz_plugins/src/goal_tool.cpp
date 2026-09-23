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

#include "nav2_rviz_plugins/goal_tool.hpp"

#include <memory>
#include <string>

#include "nav2_rviz_plugins/goal_common.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"

namespace nav2_rviz_plugins
{

// GoalTool：RViz 的 "Nav2 Goal" 工具（工具栏上的 2D Goal Pose 按钮），
// 继承自默认插件的 PoseTool，用户在 3D 视图中点选并拖出方向即设置导航目标点
GoalTool::GoalTool()
: rviz_default_plugins::tools::PoseTool()
{
  // 快捷键设为 'g'：在 RViz 中按 g 直接激活该工具
  shortcut_key_ = 'g';
}

GoalTool::~GoalTool()
{
}

void GoalTool::onInitialize()
{
  // 先完成基类初始化，再设置工具显示名与图标
  PoseTool::onInitialize();
  setName("Nav2 Goal");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/SetGoal.png"));
}

void
GoalTool::onPoseSet(double x, double y, double theta)
{
  // Set goal pose on global object GoalUpdater to update nav2 Panel
  // 用户完成选点后触发：通过全局 GoalUpdater 对象发出 Qt 信号，
  // 通知 Nav2 Panel 更新目标点位姿（坐标系取 RViz 当前固定坐标系）
  GoalUpdater.setGoal(x, y, theta, context_->getFixedFrame());
}

}  // namespace nav2_rviz_plugins

// pluginlib 导出宏：把 GoalTool 注册为 RViz 工具插件
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::GoalTool, rviz_common::Tool)
