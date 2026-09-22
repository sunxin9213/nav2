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

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/follow_path_action.hpp"

class FollowPathActionServer : public TestActionServer<nav2_msgs::action::FollowPath>
{
public:
// 初始化 FollowPathActionServer 对象，读取参数并准备运行时依赖。
  FollowPathActionServer()
// 初始化 TestActionServer 对象，读取参数并准备运行时依赖。
  : TestActionServer("follow_path")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::FollowPath>> goal_handle)
  override
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<nav2_msgs::action::FollowPath::Result>();
    goal_handle->succeed(result);
  }
};

class FollowPathActionTestFixture : public ::testing::Test
{
public:
// 初始化 SetUpTestCase 对象，读取参数并准备运行时依赖。
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("follow_path_action_test_fixture");
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
        return std::make_unique<nav2_behavior_tree::FollowPathAction>(
          name, "follow_path", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::FollowPathAction>(
      "FollowPath", builder);
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

  static std::shared_ptr<FollowPathActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr FollowPathActionTestFixture::node_ = nullptr;
std::shared_ptr<FollowPathActionServer>
FollowPathActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * FollowPathActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> FollowPathActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> FollowPathActionTestFixture::tree_ = nullptr;

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(FollowPathActionTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <FollowPath path="{path}" controller_id="FollowPath"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  nav_msgs::msg::Path path;
  path.poses.resize(1);
  path.poses[0].pose.position.x = 1.0;
  config_->blackboard->set<nav_msgs::msg::Path>("path", path);

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("controller_id"), std::string("FollowPath"));
  EXPECT_EQ(action_server_->getCurrentGoal()->path.poses.size(), 1u);
  EXPECT_EQ(action_server_->getCurrentGoal()->path.poses[0].pose.position.x, 1.0);
  EXPECT_EQ(action_server_->getCurrentGoal()->controller_id, std::string("FollowPath"));

  tree_->rootNode()->halt();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);

  // set new goal
  path.poses[0].pose.position.x = -2.5;
  config_->blackboard->set<nav_msgs::msg::Path>("path", path);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(action_server_->getCurrentGoal()->path.poses.size(), 1u);
  EXPECT_EQ(action_server_->getCurrentGoal()->path.poses[0].pose.position.x, -2.5);
}

// 实现该逻辑，并处理相关状态转换。
int main(int argc, char ** argv)
{
// 实现该逻辑，并处理相关状态转换。
  ::testing::InitGoogleTest(&argc, argv);

// 实现该逻辑，并处理相关状态转换。
  rclcpp::init(argc, argv);

  FollowPathActionTestFixture::action_server_ =
// 实现该逻辑，并处理相关状态转换。
    std::make_shared<FollowPathActionServer>();

// 实现该逻辑，并处理相关状态转换。
  std::thread server_thread([]() {
// 实现该逻辑，并处理相关状态转换。
      rclcpp::spin(FollowPathActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

// 实现该逻辑，并处理相关状态转换。
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
