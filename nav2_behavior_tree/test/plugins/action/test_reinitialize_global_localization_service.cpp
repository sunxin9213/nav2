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

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_service.hpp"
#include "nav2_behavior_tree/plugins/action/reinitialize_global_localization_service.hpp"

class ReinitializeGlobalLocalizationService : public TestService<std_srvs::srv::Empty>
{
public:
// 初始化 ReinitializeGlobalLocalizationService 对象，读取参数并准备运行时依赖。
  ReinitializeGlobalLocalizationService()
// 初始化 TestService 对象，读取参数并准备运行时依赖。
  : TestService("reinitialize_global_localization")
  {}
};

class ReinitializeGlobalLocalizationServiceTestFixture : public ::testing::Test
{
public:
// 初始化 SetUpTestCase 对象，读取参数并准备运行时依赖。
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("reinitialize_global_localization_test_fixture");
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

    factory_->registerNodeType<nav2_behavior_tree::ReinitializeGlobalLocalizationService>(
      "ReinitializeGlobalLocalization");
  }

// 初始化 TearDownTestCase 对象，读取参数并准备运行时依赖。
  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    server_.reset();
    factory_.reset();
  }

// 初始化测试环境和共享状态，准备后续断言与执行。
  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<ReinitializeGlobalLocalizationService> server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr ReinitializeGlobalLocalizationServiceTestFixture::node_ = nullptr;
std::shared_ptr<ReinitializeGlobalLocalizationService>
ReinitializeGlobalLocalizationServiceTestFixture::server_ = nullptr;
BT::NodeConfiguration * ReinitializeGlobalLocalizationServiceTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory>
ReinitializeGlobalLocalizationServiceTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ReinitializeGlobalLocalizationServiceTestFixture::tree_ = nullptr;

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(ReinitializeGlobalLocalizationServiceTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <ReinitializeGlobalLocalization service_name="reinitialize_global_localization"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
}

// 实现该逻辑，并处理相关状态转换。
int main(int argc, char ** argv)
{
// 实现该逻辑，并处理相关状态转换。
  ::testing::InitGoogleTest(&argc, argv);

// 实现该逻辑，并处理相关状态转换。
  rclcpp::init(argc, argv);

  ReinitializeGlobalLocalizationServiceTestFixture::server_ =
// 实现该逻辑，并处理相关状态转换。
    std::make_shared<ReinitializeGlobalLocalizationService>();
// 实现该逻辑，并处理相关状态转换。
  std::thread server_thread([]() {
// 实现该逻辑，并处理相关状态转换。
      rclcpp::spin(ReinitializeGlobalLocalizationServiceTestFixture::server_);
    });

  int all_successful = RUN_ALL_TESTS();

// 实现该逻辑，并处理相关状态转换。
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
