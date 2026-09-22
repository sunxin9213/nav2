// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2021 Samsung Research America
// Copyright (c) 2024 Marc Morcos
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
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/get_pose_from_path_action.hpp"
#include "../../test_behavior_tree_fixture.hpp"

class GetPoseFromPathTestFixture : public ::testing::Test
{
public:
// 初始化 SetUpTestCase 对象，读取参数并准备运行时依赖。
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("get_pose_from_path_action_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::GetPoseFromPath>(
          name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::GetPoseFromPath>(
      "GetPoseFromPath", builder);
  }

// 初始化 TearDownTestCase 对象，读取参数并准备运行时依赖。
  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    factory_.reset();
  }

// 初始化测试环境和共享状态，准备后续断言与执行。
  void TearDown() override
  {
    tree_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr GetPoseFromPathTestFixture::node_ = nullptr;
BT::NodeConfiguration * GetPoseFromPathTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> GetPoseFromPathTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> GetPoseFromPathTestFixture::tree_ = nullptr;

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(GetPoseFromPathTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <GetPoseFromPath path="{path}" pose="{pose}" index="{index}" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  nav_msgs::msg::Path path;
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  goals.resize(2);
  goals[0].pose.position.x = 1.0;
  goals[1].pose.position.x = 2.0;
  path.poses = goals;
  path.header.frame_id = "test_frame_1";
  config_->blackboard->set("path", path);

  config_->blackboard->set("index", 0);

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

  geometry_msgs::msg::PoseStamped pose;
  EXPECT_TRUE(config_->blackboard->get<geometry_msgs::msg::PoseStamped>("pose", pose));
// 初始化 EXPECT_EQ 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(pose.header.frame_id, "test_frame_1");
// 初始化 EXPECT_EQ 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(pose.pose.position.x, 1.0);

  // halt node so another goal can be sent
  tree_->haltTree();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);

  // try last element
  config_->blackboard->set("index", -1);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

  EXPECT_TRUE(config_->blackboard->get<geometry_msgs::msg::PoseStamped>("pose", pose));
// 初始化 EXPECT_EQ 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(pose.header.frame_id, "test_frame_1");
// 初始化 EXPECT_EQ 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(pose.pose.position.x, 2.0);
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
