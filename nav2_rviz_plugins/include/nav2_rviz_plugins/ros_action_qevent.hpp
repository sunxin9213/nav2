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

#ifndef NAV2_RVIZ_PLUGINS__ROS_ACTION_QEVENT_HPP_
#define NAV2_RVIZ_PLUGINS__ROS_ACTION_QEVENT_HPP_

#include <QAbstractTransition>

namespace nav2_rviz_plugins
{

// ROS Action 的两态状态（面板按钮状态机用）：激活 / 未激活
enum class QActionState
{
  ACTIVE,
  INACTIVE
};

/// Custom Event to track state of ROS Action
// 自定义 Qt 事件：携带 ROS Action 的新状态，
// 事件类型号取 QEvent::User+1，避免与内建事件类型冲突
struct ROSActionQEvent : public QEvent
{
  explicit ROSActionQEvent(QActionState state)
  : QEvent(QEvent::Type(QEvent::User + 1)),
    state_(state) {}

  QActionState state_;
};

/// Custom Transition to check whether ROS Action state has changed
// 自定义状态机迁移：仅当收到 ROSActionQEvent 且其携带的状态
// 与本转换记录的状态不同时才允许迁移（即"状态确实发生变化"）
class ROSActionQTransition : public QAbstractTransition
{
public:
  explicit ROSActionQTransition(QActionState initial_status)
  : status_(initial_status)
  {}

  ~ROSActionQTransition() {}

protected:
  // 事件测试：非本类型事件直接拒绝；状态未变化也拒绝
  virtual bool eventTest(QEvent * e)
  {
    if (e->type() != QEvent::Type(QEvent::User + 1)) {  // ROSActionEvent
      return false;
    }
    ROSActionQEvent * action_event = static_cast<ROSActionQEvent *>(e);
    return status_ != action_event->state_;
  }

  // 迁移发生时的动作（这里不需要额外处理，留空）
  virtual void onTransition(QEvent *) {}
  // 本转换所记录的上一次 Action 状态
  QActionState status_;
};

}  // namespace nav2_rviz_plugins

#endif  //  NAV2_RVIZ_PLUGINS__ROS_ACTION_QEVENT_HPP_
