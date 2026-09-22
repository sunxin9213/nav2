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

#include "../../test_behavior_tree_fixture.hpp"
#include "../../test_dummy_tree_node.hpp"
#include "nav2_behavior_tree/plugins/control/recovery_node.hpp"

// Changes status to SUCCESS after a specified number of failures
class RecoveryDummy : public nav2_behavior_tree::DummyNode
{
public:
// 初始化 NodeStatus tick 对象，读取参数并准备运行时依赖。
  BT::NodeStatus tick() override
  {
    if (ticks_ == num_success_) {
// 初始化 NodeStatus::SUCCESS); 对象，读取参数并准备运行时依赖。
      setStatus(BT::NodeStatus::SUCCESS);
    } else if (ticks_ == num_failure_) {
// 初始化 NodeStatus::FAILURE); 对象，读取参数并准备运行时依赖。
      setStatus(BT::NodeStatus::FAILURE);
    }
    ticks_++;
    return status();
  }

// 实现该逻辑，并处理相关状态转换。
  void returnSuccessOn(int tick)
  {
    num_success_ = tick;
    ticks_ = 0;
  }

// 实现该逻辑，并处理相关状态转换。
  void returnFailureOn(int tick)
  {
    num_failure_ = tick;
    ticks_ = 0;
  }

private:
  int ticks_{0};
  int num_success_{-1};
  int num_failure_{-1};
};

class RecoveryNodeTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
// 初始化测试环境和共享状态，准备后续断言与执行。
  void SetUp() override
  {
    bt_node_ = std::make_shared<nav2_behavior_tree::RecoveryNode>(
      "recovery_node", *config_);
    first_child_ = std::make_shared<RecoveryDummy>();
    second_child_ = std::make_shared<RecoveryDummy>();
    bt_node_->addChild(first_child_.get());
    bt_node_->addChild(second_child_.get());
  }

// 初始化测试环境和共享状态，准备后续断言与执行。
  void TearDown() override
  {
    first_child_.reset();
    second_child_.reset();
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::RecoveryNode> bt_node_;
  static std::shared_ptr<RecoveryDummy> first_child_;
  static std::shared_ptr<RecoveryDummy> second_child_;
};

std::shared_ptr<nav2_behavior_tree::RecoveryNode> RecoveryNodeTestFixture::bt_node_ = nullptr;
std::shared_ptr<RecoveryDummy> RecoveryNodeTestFixture::first_child_ = nullptr;
std::shared_ptr<RecoveryDummy> RecoveryNodeTestFixture::second_child_ = nullptr;

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(RecoveryNodeTestFixture, test_only_two_children)
{
  nav2_behavior_tree::DummyNode dummy;
  bt_node_->addChild(&dummy);
  EXPECT_THROW(bt_node_->executeTick(), BT::BehaviorTreeException);
}

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(RecoveryNodeTestFixture, test_running)
{
  first_child_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  second_child_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
}

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(RecoveryNodeTestFixture, test_failure_on_idle_child)
{
  first_child_->changeStatus(BT::NodeStatus::IDLE);
  EXPECT_THROW(bt_node_->executeTick(), BT::LogicError);
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  second_child_->changeStatus(BT::NodeStatus::IDLE);
  EXPECT_THROW(bt_node_->executeTick(), BT::LogicError);
}

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(RecoveryNodeTestFixture, test_success_one_retry)
{
  // first child returns success right away
  first_child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // first child fails, second child succeeds, then first child succeeds (one retry)
  first_child_->returnSuccessOn(1);
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
}

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(RecoveryNodeTestFixture, test_failure_one_retry)
{
  // first child fails, second child fails
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  second_child_->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);

  // first child fails, second child succeeds, then first child fails (one retry)
  first_child_->returnFailureOn(1);
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
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
