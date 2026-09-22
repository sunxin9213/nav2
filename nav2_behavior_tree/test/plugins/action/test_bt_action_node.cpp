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
#include <vector>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/bt_action_node.hpp"

#include "test_msgs/action/fibonacci.hpp"

using namespace std::chrono_literals; // NOLINT
using namespace std::placeholders;  // NOLINT

class FibonacciActionServer : public rclcpp::Node
{
public:
// 初始化 FibonacciActionServer 对象，读取参数并准备运行时依赖。
  FibonacciActionServer()
  : rclcpp::Node("fibonacci_node", rclcpp::NodeOptions()),
// 实现该逻辑，并处理相关状态转换。
    sleep_duration_(0ms)
  {
    this->action_server_ = rclcpp_action::create_server<test_msgs::action::Fibonacci>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
// 实现该逻辑，并处理相关状态转换。
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
  }

// 实现该逻辑，并处理相关状态转换。
  void setHandleGoalSleepDuration(std::chrono::milliseconds sleep_duration)
  {
    sleep_duration_ = sleep_duration;
  }

// 实现该逻辑，并处理相关状态转换。
  void setServerLoopRate(std::chrono::nanoseconds server_loop_rate)
  {
    server_loop_rate_ = server_loop_rate;
  }

protected:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const test_msgs::action::Fibonacci::Goal>)
  {
    RCLCPP_INFO(this->get_logger(), "Goal is received..");
    if (sleep_duration_ > 0ms) {
// 实现该逻辑，并处理相关状态转换。
      std::this_thread::sleep_for(sleep_duration_);
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>> handle)
  {
    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), handle}.detach();
  }

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>> handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    if (handle) {
      const auto goal = handle->get_goal();
      auto result = std::make_shared<test_msgs::action::Fibonacci::Result>();

      if (goal->order < 0) {
        handle->abort(result);
        return;
      }

      auto & sequence = result->sequence;
      sequence.push_back(0);
      sequence.push_back(1);

// 初始化 Rate rate 对象，读取参数并准备运行时依赖。
      rclcpp::Rate rate(server_loop_rate_);
      for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
        if (handle->is_canceling()) {
          RCLCPP_INFO(this->get_logger(), "Goal is canceling.");
          handle->canceled(result);
          return;
        }

        RCLCPP_INFO(this->get_logger(), "Goal is feedbacking.");
        sequence.push_back(sequence[i] + sequence[i - 1]);
        rate.sleep();
      }

      handle->succeed(result);
    }
  }

protected:
  rclcpp_action::Server<test_msgs::action::Fibonacci>::SharedPtr action_server_;
  std::chrono::milliseconds sleep_duration_;
  std::chrono::nanoseconds server_loop_rate_;
};

class FibonacciAction : public nav2_behavior_tree::BtActionNode<test_msgs::action::Fibonacci>
{
public:
  FibonacciAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
// 初始化 BtActionNode<test_msgs::action::Fibonacci> 对象，读取参数并准备运行时依赖。
  : nav2_behavior_tree::BtActionNode<test_msgs::action::Fibonacci>(xml_tag_name, "fibonacci", conf)
  {}

// 在每次 tick 时读取输入并准备下一步执行状态。
  void on_tick() override
  {
// 实现该逻辑，并处理相关状态转换。
    getInput("order", goal_.order);
  }

// 初始化 NodeStatus on_success 对象，读取参数并准备运行时依赖。
  BT::NodeStatus on_success() override
  {
    config().blackboard->set<std::vector<int>>("sequence", result_.result->sequence);
    return BT::NodeStatus::SUCCESS;
  }

// 初始化 NodeStatus on_cancelled 对象，读取参数并准备运行时依赖。
  BT::NodeStatus on_cancelled() override
  {
    config().blackboard->set<std::vector<int>>("sequence", result_.result->sequence);
    config().blackboard->set<bool>("on_cancelled_triggered", true);
    return BT::NodeStatus::SUCCESS;
  }

// 初始化 PortsList providedPorts 对象，读取参数并准备运行时依赖。
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<int>("order", "Fibonacci order")});
  }
};

class BTActionNodeTestFixture : public ::testing::Test
{
public:
// 初始化 SetUpTestCase 对象，读取参数并准备运行时依赖。
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("bt_action_node_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
    config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 20ms);
    config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
    config_->blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", 1000ms);
    config_->blackboard->set<bool>("initial_pose_received", false);
    config_->blackboard->set<bool>("on_cancelled_triggered", false);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<FibonacciAction>(name, config);
      };

    factory_->registerBuilder<FibonacciAction>("Fibonacci", builder);
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
    // initialize action server and spin on new thread
    action_server_ = std::make_shared<FibonacciActionServer>();
    server_thread_ = std::make_shared<std::thread>(
      []() {
        while (rclcpp::ok() && BTActionNodeTestFixture::action_server_ != nullptr) {
// 实现该逻辑，并处理相关状态转换。
          rclcpp::spin_some(BTActionNodeTestFixture::action_server_);
// 实现该逻辑，并处理相关状态转换。
          std::this_thread::sleep_for(100ns);
        }
      });
  }

// 初始化测试环境和共享状态，准备后续断言与执行。
  void TearDown() override
  {
    action_server_.reset();
    tree_.reset();
    server_thread_->join();
    server_thread_.reset();
  }

  static std::shared_ptr<FibonacciActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
  static std::shared_ptr<std::thread> server_thread_;
};

rclcpp::Node::SharedPtr BTActionNodeTestFixture::node_ = nullptr;
std::shared_ptr<FibonacciActionServer> BTActionNodeTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * BTActionNodeTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> BTActionNodeTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> BTActionNodeTestFixture::tree_ = nullptr;
std::shared_ptr<std::thread> BTActionNodeTestFixture::server_thread_ = nullptr;

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(BTActionNodeTestFixture, test_server_timeout_success)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Fibonacci order="5" />
        </BehaviorTree>
      </root>)";

  // the server timeout is larger than the goal handling duration
  config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 20ms);
  config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // setting a small action server goal handling duration
  action_server_->setHandleGoalSleepDuration(2ms);
  action_server_->setServerLoopRate(10ns);

  // to keep track of the number of ticks it took to reach a terminal result
  int ticks = 0;

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

// 初始化 WallRate loopRate 对象，读取参数并准备运行时依赖。
  rclcpp::WallRate loopRate(10ms);

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickRoot();
    ticks++;
    loopRate.sleep();
  }

  // get calculated fibonacci sequence from blackboard
  auto sequence = config_->blackboard->get<std::vector<int>>("sequence");

  // expected fibonacci sequence for order 5
  std::vector<int> expected = {0, 1, 1, 2, 3, 5};

// 初始化 NodeStatus::SUCCESS); 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);

// 初始化 EXPECT_EQ 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(sequence.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
// 初始化 EXPECT_EQ 对象，读取参数并准备运行时依赖。
    EXPECT_EQ(sequence[i], expected[i]);
  }

  // start a new execution cycle with the previous BT to ensure previous state doesn't leak into
  // the new cycle

  // halt BT for a new execution cycle,
  // get if the on_cancelled is triggered from blackboard and assert
  RCLCPP_INFO(node_->get_logger(), "Tree is halting.");
  tree_->haltTree();
  bool on_cancelled_triggered = config_->blackboard->get<bool>("on_cancelled_triggered");
// 初始化 EXPECT_EQ 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(on_cancelled_triggered, false);

  // setting a large action server goal handling duration
  action_server_->setHandleGoalSleepDuration(100ms);
  action_server_->setServerLoopRate(10ns);

  // reset state variables
  ticks = 0;
  result = BT::NodeStatus::RUNNING;
  config_->blackboard->set<bool>("on_cancelled_triggered", false);

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickRoot();
    ticks++;
    loopRate.sleep();
  }

// 初始化 NodeStatus::FAILURE); 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(result, BT::NodeStatus::FAILURE);

// 初始化 EXPECT_LE 对象，读取参数并准备运行时依赖。
  EXPECT_LE(ticks, 3);
// 初始化 EXPECT_GE 对象，读取参数并准备运行时依赖。
  EXPECT_GE(ticks, 1);
}

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(BTActionNodeTestFixture, test_server_timeout_failure)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Fibonacci order="2" />
        </BehaviorTree>
      </root>)";

  // setting a server timeout smaller than the time the action server will take to accept the goal
  // to simulate a server timeout scenario
  config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 90ms);
  config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // the action server will take 100ms before accepting the goal
  action_server_->setHandleGoalSleepDuration(100ms);
  action_server_->setServerLoopRate(10ns);

  // to keep track of the number of ticks it took to reach a terminal result
  int ticks = 0;

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

// 初始化 WallRate loopRate 对象，读取参数并准备运行时依赖。
  rclcpp::WallRate loopRate(10ms);

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickRoot();
    ticks++;
    loopRate.sleep();
  }

// 初始化 NodeStatus::FAILURE); 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(result, BT::NodeStatus::FAILURE);

// 初始化 EXPECT_EQ 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(ticks, 10);

  // start a new execution cycle with the previous BT to ensure previous state doesn't leak into
  // the new cycle

  // halt BT for a new execution cycle
  // get if the on_cancel is triggered from blackboard and assert
  RCLCPP_INFO(node_->get_logger(), "Tree is halting.");
  tree_->haltTree();
  bool on_cancelled_triggered = config_->blackboard->get<bool>("on_cancelled_triggered");
// 初始化 EXPECT_EQ 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(on_cancelled_triggered, false);

  // setting a small action server goal handling duration
  action_server_->setHandleGoalSleepDuration(25ms);
  action_server_->setServerLoopRate(10ns);

  // reset state variables
  ticks = 0;
  result = BT::NodeStatus::RUNNING;
  config_->blackboard->set<bool>("on_cancelled_triggered", false);

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickRoot();
    ticks++;
    loopRate.sleep();
  }

// 初始化 NodeStatus::SUCCESS); 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);
}

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(BTActionNodeTestFixture, test_server_cancel)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Fibonacci order="1000000" />
        </BehaviorTree>
      </root>)";

  // setting a server timeout smaller than the time the action server will take to accept the goal
  // to simulate a server timeout scenario
  config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 100ms);
  config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // the action server will take 2ms before accepting the goal
  // and the feedback period of the action server will be 50ms
  action_server_->setHandleGoalSleepDuration(2ms);
  action_server_->setServerLoopRate(50ms);

  // to keep track of the number of ticks it took to reach expected tick count
  int ticks = 0;

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

// 初始化 WallRate loopRate 对象，读取参数并准备运行时依赖。
  rclcpp::WallRate loopRate(100ms);

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING && ticks < 5) {
    result = tree_->tickRoot();
    ticks++;
    loopRate.sleep();
  }

  RCLCPP_INFO(node_->get_logger(), "Tree is halting.");
  tree_->haltTree();

  // get if the on_cancel is triggered from blackboard and assert
  // that the on_cancel is triggered after halting node
  bool on_cancelled_triggered = config_->blackboard->get<bool>("on_cancelled_triggered");
// 初始化 EXPECT_EQ 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(on_cancelled_triggered, true);

// 初始化 EXPECT_EQ 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(ticks, 5);

  // send new goal to the action server for a new execution cycle

  // the action server will take 2ms before accepting the goal
  // and the feedback period of the action server will be 1000ms
  action_server_->setHandleGoalSleepDuration(2ms);
  action_server_->setServerLoopRate(50ms);

  // reset state variable
  ticks = 0;
  config_->blackboard->set<bool>("on_cancelled_triggered", false);
  result = BT::NodeStatus::RUNNING;

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING && ticks < 7) {
    result = tree_->tickRoot();
    ticks++;
    loopRate.sleep();
  }

  RCLCPP_INFO(node_->get_logger(), "Tree is halting.");
  tree_->haltTree();

  // get if the on_cancel is triggered from blackboard and assert
  // that the on_cancel is triggered after halting node
  on_cancelled_triggered = config_->blackboard->get<bool>("on_cancelled_triggered");
// 初始化 EXPECT_EQ 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(on_cancelled_triggered, true);

// 初始化 EXPECT_EQ 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(ticks, 7);
}

// 实现该逻辑，并处理相关状态转换。
int main(int argc, char ** argv)
{
// 实现该逻辑，并处理相关状态转换。
  ::testing::InitGoogleTest(&argc, argv);

// 实现该逻辑，并处理相关状态转换。
  rclcpp::init(argc, argv);

  int all_successful = RUN_ALL_TESTS();

// 实现该逻辑，并处理相关状态转换。
  rclcpp::shutdown();

  return all_successful;
}
