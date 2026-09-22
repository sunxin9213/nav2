// Copyright (c) 2022 Joshua Wallace
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
#include "nav2_behavior_tree/plugins/action/assisted_teleop_action.hpp"

class AssistedTeleopActionServer : public TestActionServer<nav2_msgs::action::AssistedTeleop>
{
public:
// 初始化 AssistedTeleopActionServer 对象，读取参数并准备运行时依赖。
  AssistedTeleopActionServer()
// 初始化 TestActionServer 对象，读取参数并准备运行时依赖。
  : TestActionServer("assisted_teleop")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::AssistedTeleop>>
    goal_handle)
  override
  {
    nav2_msgs::action::AssistedTeleop::Result::SharedPtr result =
// 实现该逻辑，并处理相关状态转换。
      std::make_shared<nav2_msgs::action::AssistedTeleop::Result>();
    bool return_success = getReturnSuccess();
    if (return_success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }
};

class AssistedTeleopActionTestFixture : public ::testing::Test
{
public:
// 初始化 SetUpTestCase 对象，读取参数并准备运行时依赖。
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("backup_action_test_fixture");
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
    config_->blackboard->set<int>("number_recoveries", 0);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::AssistedTeleopAction>(
          name, "assisted_teleop", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::AssistedTeleopAction>("AssistedTeleop", builder);
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
  void SetUp() override
  {
    config_->blackboard->set("number_recoveries", 0);
  }

// 初始化测试环境和共享状态，准备后续断言与执行。
  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<AssistedTeleopActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr AssistedTeleopActionTestFixture::node_ = nullptr;
std::shared_ptr<AssistedTeleopActionServer>
AssistedTeleopActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * AssistedTeleopActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> AssistedTeleopActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> AssistedTeleopActionTestFixture::tree_ = nullptr;

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(AssistedTeleopActionTestFixture, test_ports)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <AssistedTeleop />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<double>("time_allowance"), 10.0);

  xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <AssistedTeleop time_allowance="20"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<double>("time_allowance"), 20.0);
}

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(AssistedTeleopActionTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <AssistedTeleop is_recovery="true"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 0);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 1);

  auto goal = action_server_->getCurrentGoal();
  EXPECT_EQ(goal->time_allowance.sec, 10.0);
}

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(AssistedTeleopActionTestFixture, test_failure)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <AssistedTeleop is_recovery="true"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  action_server_->setReturnSuccess(false);
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 0);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
    tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 1);

  auto goal = action_server_->getCurrentGoal();
  EXPECT_EQ(goal->time_allowance.sec, 10.0);
}

// 实现该逻辑，并处理相关状态转换。
int main(int argc, char ** argv)
{
// 实现该逻辑，并处理相关状态转换。
  ::testing::InitGoogleTest(&argc, argv);

// 实现该逻辑，并处理相关状态转换。
  rclcpp::init(argc, argv);

  AssistedTeleopActionTestFixture::action_server_ = std::make_shared<AssistedTeleopActionServer>();
// 实现该逻辑，并处理相关状态转换。
  std::thread server_thread([]() {
// 实现该逻辑，并处理相关状态转换。
      rclcpp::spin(AssistedTeleopActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

// 实现该逻辑，并处理相关状态转换。
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
