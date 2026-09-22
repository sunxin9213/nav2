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

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/initial_pose_received_condition.hpp"

class TestNode : public BT::SyncActionNode
{
public:
// 实现该逻辑，并处理相关状态转换。
  TestNode(const std::string & name, const BT::NodeConfiguration & config)
// 实现 SyncActionNode 的控制或装饰逻辑，管理子节点执行顺序和状态转换。
  : SyncActionNode(name, config)
  {}

// 初始化 NodeStatus tick 对象，读取参数并准备运行时依赖。
  BT::NodeStatus tick()
  {
    return BT::NodeStatus::SUCCESS;
  }

// 初始化 PortsList providedPorts 对象，读取参数并准备运行时依赖。
  static BT::PortsList providedPorts()
  {
    return {};
  }
};

class InitialPoseReceivedConditionTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
// 初始化测试环境和共享状态，准备后续断言与执行。
  void SetUp()
  {
    test_node_ = std::make_shared<TestNode>("TestNode", *config_);
  }

// 初始化测试环境和共享状态，准备后续断言与执行。
  void TearDown()
  {
    test_node_.reset();
  }

protected:
  static std::shared_ptr<TestNode> test_node_;
};

std::shared_ptr<TestNode> InitialPoseReceivedConditionTestFixture::test_node_ = nullptr;

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(InitialPoseReceivedConditionTestFixture, test_behavior)
{
// 实现该逻辑，并处理相关状态转换。
  EXPECT_EQ(nav2_behavior_tree::initialPoseReceived(*test_node_), BT::NodeStatus::FAILURE);
  config_->blackboard->set("initial_pose_received", true);
// 实现该逻辑，并处理相关状态转换。
  EXPECT_EQ(nav2_behavior_tree::initialPoseReceived(*test_node_), BT::NodeStatus::SUCCESS);
}

// 实现该逻辑，并处理相关状态转换。
int main(int argc, char ** argv)
{
// 实现该逻辑，并处理相关状态转换。
  ::testing::InitGoogleTest(&argc, argv);

// 实现该逻辑，并处理相关状态转换。
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

// 实现该逻辑，并处理相关状态转换。
  rclcpp::shutdown();

  return all_successful;
}
