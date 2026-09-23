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

#ifndef NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_
#define NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_

#include <QtWidgets>
#include <QBasicTimer>
#undef NO_ERROR

#include <memory>
#include <string>
#include <vector>

#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_rviz_plugins/ros_action_qevent.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_util/geometry_utils.hpp"

class QPushButton;

namespace nav2_rviz_plugins
{

class InitialThread;

/// Panel to interface to the nav2 stack
// Nav2 控制面板：RViz 左下角的导航控制界面。
// 功能：启动/关闭/暂停/恢复 Nav2 生命周期节点组、发送导航目标
//（单点导航 / 路点跟随 / 途经点导航三种 Action），
// 并通过 Qt 状态机（QStateMachine）管理按钮在不同状态下的显示与行为。
class Nav2Panel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit Nav2Panel(QWidget * parent = 0);
  virtual ~Nav2Panel();

  // 面板初始化：创建 Action 反馈/状态话题的订阅
  void onInitialize() override;

  /// Load and save configuration data
  // RViz 配置的加载与保存（当前无自定义配置项，直接调用基类）
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  // Qt 槽函数（由状态机/信号触发）
  void startThread();              // 状态机启动后启动初始检测线程
  void onStartup();                // 启动导航与定位生命周期节点组
  void onShutdown();               // 复位（deactivate+cleanup）生命周期节点组
  void onCancel();                 // 取消当前导航 Action
  void onPause();                  // 暂停：deactivate 所有节点
  void onResume();                 // 恢复：activate 所有节点
  void onAccumulatedWp();          // 路点收集完成，开始 FollowWaypoints
  void onAccumulatedNTP();         // 途经点收集完成，开始 NavigateThroughPoses
  void onAccumulating();           // 进入路点收集模式，清空旧数据
  void onNewGoal(double x, double y, double theta, QString frame);  // 收到新目标点

private:
  void loadLogFiles();
  void onCancelButtonPressed();    // 实际执行取消（在线程池中调用）
  void timerEvent(QTimerEvent * event) override;  // 定时轮询 Action 状态驱动状态机

  // Marker 的自增 id 计数器（保证每个可视化 marker 唯一）
  int unique_id {0};

  // Call to send NavigateToPose action request for goal poses
  // 三种导航模式的 Action 发送函数
  void startWaypointFollowing(std::vector<geometry_msgs::msg::PoseStamped> poses);
  void startNavigation(geometry_msgs::msg::PoseStamped);
  void startNavThroughPoses(std::vector<geometry_msgs::msg::PoseStamped> poses);
  // 三种 Action 的 GoalHandle 类型别名
  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using WaypointFollowerGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
  using NavThroughPosesGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>;

  // The (non-spinning) client node used to invoke the action client
  // 承载 Action 客户端的 ROS 节点（不在独立执行器中 spin，
  // 而是按需用 spin_until_future_complete / spin_some 处理）
  rclcpp::Node::SharedPtr client_node_;

  // Timeout value when waiting for action servers to respnd
  // 等待 Action 服务器响应的超时（100ms）
  std::chrono::milliseconds server_timeout_;

  // A timer used to check on the completion status of the action
  // 200ms 周期定时器：轮询目标状态并向状态机投递 ROSActionQEvent
  QBasicTimer timer_;

  // The NavigateToPose action client
  // 三个 Action 客户端：单点导航 / 路点跟随 / 途经点导航
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr
    waypoint_follower_action_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr
    nav_through_poses_action_client_;

  // Navigation action feedback subscribers
  // 直接订阅 Action 的底层 feedback/status 话题，用于更新面板显示
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr
    navigation_feedback_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>::SharedPtr
    nav_through_poses_feedback_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::GoalStatusMessage>::SharedPtr
    navigation_goal_status_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::GoalStatusMessage>::SharedPtr
    nav_through_poses_goal_status_sub_;

  // Goal-related state
  // 目标相关的状态：三种 Goal 消息与对应的句柄
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;
  nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;
  NavThroughPosesGoalHandle::SharedPtr nav_through_poses_goal_handle_;

  // The client used to control the nav2 stack
  // 生命周期管理客户端：分别控制导航组与定位组节点
  std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> client_nav_;
  std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> client_loc_;

  // 三个功能按钮：启动/复位、暂停/恢复、导航模式切换
  QPushButton * start_reset_button_{nullptr};
  QPushButton * pause_resume_button_{nullptr};
  QPushButton * navigation_mode_button_{nullptr};

  // 四个状态显示标签：导航/定位状态、目标状态、反馈信息
  QLabel * navigation_status_indicator_{nullptr};
  QLabel * localization_status_indicator_{nullptr};
  QLabel * navigation_goal_status_indicator_{nullptr};
  QLabel * navigation_feedback_indicator_{nullptr};

  // 按钮状态机与初始检测线程
  QStateMachine state_machine_;
  InitialThread * initial_thread_;

  // ===== 状态机的各个状态 =====
  QState * pre_initial_{nullptr};   // 等待初始状态检测（按钮全部禁用）
  QState * initial_{nullptr};       // 未启动（按钮显示 Startup）
  QState * idle_{nullptr};          // 已启动空闲（按钮显示 Reset/Pause）
  QState * reset_{nullptr};         // 复位过渡态（触发 onShutdown 后回 initial）
  QState * paused_{nullptr};        // 暂停态（按钮显示 Resume）
  QState * resumed_{nullptr};       // 恢复过渡态（触发 onResume 后回 idle）
  // The following states are added to allow for the state of the button to only expose reset
  // while the NavigateToPoses action is not active. While running, the user will be allowed to
  // cancel the action. The ROSActionTransition allows for the state of the action to be detected
  // and the button state to change automatically.
  QState * running_{nullptr};       // 单点导航执行中（按钮显示 Cancel）
  QState * canceled_{nullptr};      // 取消过渡态（触发 onCancel 后回 idle）
  // The following states are added to allow to collect several poses to perform a waypoint-mode
  // navigation or navigate through poses mode.
  QState * accumulating_{nullptr};  // 路点收集模式（连点多个目标）
  QState * accumulated_wp_{nullptr};            // 路点收集完成后执行 FollowWaypoints
  QState * accumulated_nav_through_poses_{nullptr};  // 途经点收集完成后执行 NTP

  // 路点收集模式下累积的目标点列表
  std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;

  // Publish the visual markers with the waypoints
  // 发布路点可视化 marker（箭头+圆点+序号文本）
  void updateWpNavigationMarkers();

  // Create unique id numbers for markers
  int getUniqueId();

  void resetUniqueId();

  // create label string from goal status msg
  // 把 Action 状态/反馈消息转成面板显示的 HTML 富文本
  static inline QString getGoalStatusLabel(
    int8_t status = action_msgs::msg::GoalStatus::STATUS_UNKNOWN);

  // create label string from feedback msg
  static inline QString getNavToPoseFeedbackLabel(
    nav2_msgs::action::NavigateToPose::Feedback msg =
    nav2_msgs::action::NavigateToPose::Feedback());
  static inline QString getNavThroughPosesFeedbackLabel(
    nav2_msgs::action::NavigateThroughPoses::Feedback =
    nav2_msgs::action::NavigateThroughPoses::Feedback());
  template<typename T>
  static inline std::string toLabel(T & msg);

  // round off double to the specified precision and convert to string
  // 按指定精度四舍五入并转为字符串
  static inline std::string toString(double val, int precision = 0);

  // Waypoint navigation visual markers publisher
  // 路点 marker 发布器（transient_local：新订阅者也能收到最新 marker）
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wp_navigation_markers_pub_;
};

// 初始检测线程：面板创建后异步探测导航/定位生命周期管理器是否在线，
// 避免阻塞 UI 线程；探测结果通过 Qt 信号通知面板
class InitialThread : public QThread
{
  Q_OBJECT

public:
  using SystemStatus = nav2_lifecycle_manager::SystemStatus;

  explicit InitialThread(
    std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> & client_nav,
    std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> & client_loc)
  : client_nav_(client_nav), client_loc_(client_loc)
  {}

  // 线程主函数：分别探测导航组与定位组的状态
  void run() override
  {
    SystemStatus status_nav = SystemStatus::TIMEOUT;
    SystemStatus status_loc = SystemStatus::TIMEOUT;

    // 无限重试直到导航管理器可达（每次探测超时 1s）
    while (status_nav == SystemStatus::TIMEOUT) {
      if (status_nav == SystemStatus::TIMEOUT) {
        status_nav = client_nav_->is_active(std::chrono::seconds(1));
      }
    }

    // try to communicate twice, might not actually be up if in SLAM mode
    // 定位组只尝试两次：SLAM 模式下可能没有独立定位管理器
    bool tried_loc_bringup_once = false;
    while (status_loc == SystemStatus::TIMEOUT) {
      status_loc = client_loc_->is_active(std::chrono::seconds(1));
      if (tried_loc_bringup_once) {
        break;
      }
      tried_loc_bringup_once = true;
    }

    // 根据探测结果发出信号，驱动状态机进入 initial_ 或 idle_
    if (status_nav == SystemStatus::ACTIVE) {
      emit navigationActive();
    } else {
      emit navigationInactive();
    }

    if (status_loc == SystemStatus::ACTIVE) {
      emit localizationActive();
    } else {
      emit localizationInactive();
    }
  }

signals:
  // 导航/定位系统在线状态的信号
  void navigationActive();
  void navigationInactive();
  void localizationActive();
  void localizationInactive();

private:
  std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> client_nav_;
  std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> client_loc_;
};

}  // namespace nav2_rviz_plugins

#endif  //  NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_
