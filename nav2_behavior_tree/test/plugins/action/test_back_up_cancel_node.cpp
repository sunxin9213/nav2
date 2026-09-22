// Copyright (c) 2022 Neobotix GmbH
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

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/back_up_cancel_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

class CancelBackUpServer : public TestActionServer<nav2_msgs::action::BackUp>
{
public:
// 初始化 CancelBackUpServer 对象，读取参数并准备运行时依赖。
  CancelBackUpServer()
// 初始化 TestActionServer 对象，读取参数并准备运行时依赖。
  : TestActionServer("back_up")
  {}

protected:
  void execute(
    const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::BackUp>>
    goal_handle)
  {
    while (!goal_handle->is_canceling()) {
// 实现该逻辑，并处理相关状态转换。
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
  }
};

class CancelBackUpActionTestFixture : public ::testing::Test
{
public:
// 初始化 SetUpTestCase 对象，读取参数并准备运行时依赖。
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("cancel_back_up_action_test_fixture");
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
    client_ = rclcpp_action::create_client<nav2_msgs::action::BackUp>(
      node_, "back_up");

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::BackUpCancel>(
          name, "back_up", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::BackUpCancel>("CancelBackUp", builder);
  }

// 初始化 TearDownTestCase 对象，读取参数并准备运行时依赖。
  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    action_server_.reset();
    client_.reset();
    factory_.reset();
  }

// 初始化测试环境和共享状态，准备后续断言与执行。
  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<CancelBackUpServer> action_server_;
  static std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::BackUp>> client_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr CancelBackUpActionTestFixture::node_ = nullptr;
std::shared_ptr<CancelBackUpServer>
CancelBackUpActionTestFixture::action_server_ = nullptr;
std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::BackUp>>
CancelBackUpActionTestFixture::client_ = nullptr;

BT::NodeConfiguration * CancelBackUpActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory>
CancelBackUpActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> CancelBackUpActionTestFixture::tree_ = nullptr;

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(CancelBackUpActionTestFixture, test_ports)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
             <CancelBackUp name="BackUpCancel"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::BackUp>::SendGoalOptions();

  // Creating a dummy goal_msg
  auto goal_msg = nav2_msgs::action::BackUp::Goal();

  // Setting target pose
  goal_msg.target.x = 0.5;

  // BackUping for server and sending a goal
  client_->wait_for_action_server();
  client_->async_send_goal(goal_msg, send_goal_options);

// 实现该逻辑，并处理相关状态转换。
  std::this_thread::sleep_for(std::chrono::milliseconds(15));

  // Executing tick
  tree_->rootNode()->executeTick();

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(action_server_->isGoalCancelled(), true);
}

// 实现该逻辑，并处理相关状态转换。
int main(int argc, char ** argv)
{
// 实现该逻辑，并处理相关状态转换。
  ::testing::InitGoogleTest(&argc, argv);

// 实现该逻辑，并处理相关状态转换。
  rclcpp::init(argc, argv);

  CancelBackUpActionTestFixture::action_server_ = std::make_shared<CancelBackUpServer>();
// 实现该逻辑，并处理相关状态转换。
  std::thread server_thread([]() {
// 实现该逻辑，并处理相关状态转换。
      rclcpp::spin(CancelBackUpActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

// 实现该逻辑，并处理相关状态转换。
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
