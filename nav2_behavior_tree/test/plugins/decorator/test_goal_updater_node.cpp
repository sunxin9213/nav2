// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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
#include "nav2_behavior_tree/plugins/decorator/goal_updater_node.hpp"


class GoalUpdaterTestFixture : public ::testing::Test
{
public:
// 初始化 SetUpTestCase 对象，读取参数并准备运行时依赖。
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("goal_updater_test_fixture");
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
        return std::make_unique<nav2_behavior_tree::GoalUpdater>(
          name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::GoalUpdater>(
      "GoalUpdater", builder);
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

rclcpp::Node::SharedPtr GoalUpdaterTestFixture::node_ = nullptr;

BT::NodeConfiguration * GoalUpdaterTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> GoalUpdaterTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> GoalUpdaterTestFixture::tree_ = nullptr;

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(GoalUpdaterTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <GoalUpdater input_goal="{goal}" output_goal="{updated_goal}">
            <AlwaysSuccess/>
          </GoalUpdater>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  auto goal_updater_pub =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_update", 10);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_->now();
  goal.pose.position.x = 1.0;
  config_->blackboard->set("goal", goal);

  // tick tree without publishing updated goal and get updated_goal
  tree_->rootNode()->executeTick();
  geometry_msgs::msg::PoseStamped updated_goal;
  config_->blackboard->get("updated_goal", updated_goal);
}

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(GoalUpdaterTestFixture, test_older_goal_update)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <GoalUpdater input_goal="{goal}" output_goal="{updated_goal}">
            <AlwaysSuccess/>
          </GoalUpdater>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  auto goal_updater_pub =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_update", 10);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_->now();
  goal.pose.position.x = 1.0;
  config_->blackboard->set("goal", goal);

  geometry_msgs::msg::PoseStamped goal_to_update;
  goal_to_update.header.stamp = rclcpp::Time(goal.header.stamp) - rclcpp::Duration(1, 0);
  goal_to_update.pose.position.x = 2.0;

  goal_updater_pub->publish(goal_to_update);
  tree_->rootNode()->executeTick();
  geometry_msgs::msg::PoseStamped updated_goal;
  config_->blackboard->get("updated_goal", updated_goal);

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
// 初始化 EXPECT_EQ 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(updated_goal, goal);
}

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(GoalUpdaterTestFixture, test_get_latest_goal_update)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <GoalUpdater input_goal="{goal}" output_goal="{updated_goal}">
            <AlwaysSuccess/>
          </GoalUpdater>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  auto goal_updater_pub =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_update", 10);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_->now();
  goal.pose.position.x = 1.0;
  config_->blackboard->set("goal", goal);

  geometry_msgs::msg::PoseStamped goal_to_update_1;
  goal_to_update_1.header.stamp = node_->now();
  goal_to_update_1.pose.position.x = 2.0;

  geometry_msgs::msg::PoseStamped goal_to_update_2;
  goal_to_update_2.header.stamp = node_->now();
  goal_to_update_2.pose.position.x = 3.0;

  goal_updater_pub->publish(goal_to_update_1);
  goal_updater_pub->publish(goal_to_update_2);
  tree_->rootNode()->executeTick();
  geometry_msgs::msg::PoseStamped updated_goal;
  config_->blackboard->get("updated_goal", updated_goal);

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
// 初始化 EXPECT_EQ 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(updated_goal, goal_to_update_2);
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
