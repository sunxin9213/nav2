// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2019 Intel Corporation
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
#include <chrono>

#include "sensor_msgs/msg/battery_state.hpp"

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/is_battery_low_condition.hpp"

class IsBatteryLowConditionTestFixture : public ::testing::Test
{
public:
// 初始化 SetUpTestCase 对象，读取参数并准备运行时依赖。
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("test_is_battery_low");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);

    factory_->registerNodeType<nav2_behavior_tree::IsBatteryLowCondition>("IsBatteryLow");

    battery_pub_ = node_->create_publisher<sensor_msgs::msg::BatteryState>(
      "/battery_status",
// 初始化 SystemDefaultsQoS 对象，读取参数并准备运行时依赖。
      rclcpp::SystemDefaultsQoS());
  }

// 初始化 TearDownTestCase 对象，读取参数并准备运行时依赖。
  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    battery_pub_.reset();
    node_.reset();
    factory_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
};

rclcpp::Node::SharedPtr IsBatteryLowConditionTestFixture::node_ = nullptr;
BT::NodeConfiguration * IsBatteryLowConditionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> IsBatteryLowConditionTestFixture::factory_ = nullptr;
rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr
IsBatteryLowConditionTestFixture::battery_pub_ = nullptr;

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(IsBatteryLowConditionTestFixture, test_behavior_percentage)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <IsBatteryLow min_battery="0.5" battery_topic="/battery_status"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  sensor_msgs::msg::BatteryState battery_msg;
  battery_msg.percentage = 1.0;
  battery_pub_->publish(battery_msg);
// 实现该逻辑，并处理相关状态转换。
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
// 实现该逻辑，并处理相关状态转换。
  rclcpp::spin_some(node_);
// 初始化 NodeStatus::FAILURE); 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

  battery_msg.percentage = 0.49;
  battery_pub_->publish(battery_msg);
// 实现该逻辑，并处理相关状态转换。
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
// 实现该逻辑，并处理相关状态转换。
  rclcpp::spin_some(node_);
// 初始化 NodeStatus::SUCCESS); 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  battery_msg.percentage = 0.51;
  battery_pub_->publish(battery_msg);
// 实现该逻辑，并处理相关状态转换。
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
// 实现该逻辑，并处理相关状态转换。
  rclcpp::spin_some(node_);
// 初始化 NodeStatus::FAILURE); 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

  battery_msg.percentage = 0.0;
  battery_pub_->publish(battery_msg);
// 实现该逻辑，并处理相关状态转换。
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
// 实现该逻辑，并处理相关状态转换。
  rclcpp::spin_some(node_);
// 初始化 NodeStatus::SUCCESS); 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}

// 初始化测试环境和共享状态，准备后续断言与执行。
TEST_F(IsBatteryLowConditionTestFixture, test_behavior_voltage)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <IsBatteryLow min_battery="5.0" battery_topic="/battery_status" is_voltage="true"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  sensor_msgs::msg::BatteryState battery_msg;
  battery_msg.voltage = 10.0;
  battery_pub_->publish(battery_msg);
// 实现该逻辑，并处理相关状态转换。
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
// 实现该逻辑，并处理相关状态转换。
  rclcpp::spin_some(node_);
// 初始化 NodeStatus::FAILURE); 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

  battery_msg.voltage = 4.9;
  battery_pub_->publish(battery_msg);
// 实现该逻辑，并处理相关状态转换。
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
// 实现该逻辑，并处理相关状态转换。
  rclcpp::spin_some(node_);
// 初始化 NodeStatus::SUCCESS); 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  battery_msg.voltage = 5.1;
  battery_pub_->publish(battery_msg);
// 实现该逻辑，并处理相关状态转换。
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
// 实现该逻辑，并处理相关状态转换。
  rclcpp::spin_some(node_);
// 初始化 NodeStatus::FAILURE); 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

  battery_msg.voltage = 0.0;
  battery_pub_->publish(battery_msg);
// 实现该逻辑，并处理相关状态转换。
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
// 实现该逻辑，并处理相关状态转换。
  rclcpp::spin_some(node_);
// 初始化 NodeStatus::SUCCESS); 对象，读取参数并准备运行时依赖。
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
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
