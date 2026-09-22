// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp"

class NavigateToPoseActionServer : public TestActionServer<nav2_msgs::action::NavigateToPose>
{
public:
// 初始化 NavigateToPoseActionServer 对象，读取参数并准备运行时依赖。
  NavigateToPoseActionServer()
// 初始化 TestActionServer 对象，读取参数并准备运行时依赖。
  : TestActionServer("navigate_to_pose")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
  override
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
    goal_handle->succeed(result);
  }
};

class NavigateToPoseActionTestFixture : public ::testing::Test
{
public:
// 初始化 SetUpTestCase 对象，读取参数并准备运行时依赖。
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("navigate_to_pose_action_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
// 实现该逻辑，并处理相关状态转换。
      std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
// 实现该逻辑，并处理相关状态转换。
      std::chrono::milliseconds(10));
    config_->blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout",
// 实现该逻辑，并处理相关状态转换。
      std::chrono::milliseconds(1000));
    config_->blackboard->set<bool>("initial_pose_received", false);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::NavigateToPoseAction>(
          name, "navigate_to_pose", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::NavigateToPoseAction>(
      "NavigateToPose", builder);
  }

// 初始化 TearDownTestCase 对象，读取参数并准备运行时依赖。
  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    action_server_.reset();
    factory_.reset();
  }

// 初始化测试环境和共享状态，准备后续断言与执行。
  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<NavigateToPoseActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr NavigateToPoseActionTestFixture::node_ = nullptr;
std::shared_ptr<NavigateToPoseActionServer>
NavigateToPoseActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * NavigateToPoseActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> NavigateToPoseActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> NavigateToPoseActionTestFixture::tree_ = nullptr;

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(NavigateToPoseActionTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <NavigateToPose goal="{goal}" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  geometry_msgs::msg::PoseStamped pose;

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(action_server_->getCurrentGoal()->pose, pose);

  tree_->rootNode()->halt();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);

  // set new goal
  pose.pose.position.x = -2.5;
  pose.pose.orientation.x = 1.0;
  config_->blackboard->set<geometry_msgs::msg::PoseStamped>("goal", pose);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(action_server_->getCurrentGoal()->pose, pose);
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
}

// 实现该逻辑，并处理相关状态转换。
int main(int argc, char ** argv)
{
// 实现该逻辑，并处理相关状态转换。
  ::testing::InitGoogleTest(&argc, argv);

// 实现该逻辑，并处理相关状态转换。
  rclcpp::init(argc, argv);

  NavigateToPoseActionTestFixture::action_server_ =
// 实现该逻辑，并处理相关状态转换。
    std::make_shared<NavigateToPoseActionServer>();

// 实现该逻辑，并处理相关状态转换。
  std::thread server_thread([]() {
// 实现该逻辑，并处理相关状态转换。
      rclcpp::spin(NavigateToPoseActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

// 实现该逻辑，并处理相关状态转换。
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
