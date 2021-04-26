// Copyright 2021 The Rebooters
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
// limitations under the License.#include <memory>

#include <string>

#include "gtest/gtest.h"

#include "blackboard/BlackBoardNode.hpp"
#include "blackboard/BlackBoardClient.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(blackboard_node, add_get_entry)
{
  auto blackboard = blackboard::BlackBoardNode::make_shared();
  auto client_1 = blackboard::BlackBoardClient::make_shared();

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 8);
  exe.add_node(blackboard->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  auto entry_1 = blackboard::Entry<bool>::make_shared(true);

  auto entry_base = entry_1->to_base();
  auto entry_2 = blackboard::Entry<std::string>::make_shared("Hi!!");

  client_1->add_entry("room", "my_entry_1", entry_1->to_base());
  client_1->add_entry("room", "my_entry_2", entry_2->to_base());

  auto entry_1_got = blackboard::as<bool>(client_1->get_entry("room", "my_entry_1"));
  auto entry_2_got = blackboard::as<std::string>(client_1->get_entry("room", "my_entry_2"));

  ASSERT_TRUE(entry_1_got->data_);
  ASSERT_EQ(entry_2_got->data_, "Hi!!");

  finish = true;
  t.join();
}

TEST(blackboard, check_entry_parent)
{
  auto blackboard = blackboard::BlackBoardNode::make_shared();
  auto client_1 = blackboard::BlackBoardClient::make_shared();

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 8);
  exe.add_node(blackboard->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  auto entry_1 = blackboard::Entry<bool>::make_shared(true);
  auto entry_2 = blackboard::Entry<std::string>::make_shared("Hi!!");

  client_1->add_entry("room", "my_entry_1", entry_1->to_base());
  client_1->add_entry("room", "my_entry_2", entry_2->to_base());

  auto exists_parent_1 = client_1->exist_parent("room");
  auto exists_entry_2 = client_1->exist_entry("room", "my_entry_2");
  auto unreal_entry = client_1->exist_entry("room", "false_entry");
  auto unreal_parent = client_1->exist_parent("false_parent");

  ASSERT_FALSE(unreal_entry);
  ASSERT_FALSE(unreal_parent);
  ASSERT_TRUE(exists_parent_1);
  ASSERT_TRUE(exists_entry_2);

  finish = true;
  t.join();
}

TEST(blackboard_node, remove_entry_parent)
{
  auto blackboard = blackboard::BlackBoardNode::make_shared();
  auto client_1 = blackboard::BlackBoardClient::make_shared();

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 8);
  exe.add_node(blackboard->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  auto entry_1 = blackboard::Entry<std::string>::make_shared("will_be_removed");
  auto entry_2 = blackboard::Entry<std::string>::make_shared("will_rm_parent");

  client_1->add_entry("parent1", "to_remove", entry_1->to_base());
  client_1->add_entry("parent2", "to_rm_parent", entry_2->to_base());

  client_1->remove_entry("parent1", "to_remove");
  client_1->remove_parent("parent2");

  auto removed_entry = client_1->exist_entry("parent1", "to_remove");
  auto removed_parent = client_1->exist_parent("parent2");
  auto removed_parent_entry = client_1->exist_entry("parent2", "to_rm_parent");

  ASSERT_FALSE(removed_entry);
  ASSERT_FALSE(removed_parent);
  ASSERT_FALSE(removed_parent_entry);

  finish = true;
  t.join();
}

TEST(blackboard, pose_entry)
{
  auto blackboard = blackboard::BlackBoardNode::make_shared();
  auto client_1 = blackboard::BlackBoardClient::make_shared();

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 8);
  exe.add_node(blackboard->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  geometry_msgs::msg::PoseStamped pose_1;

  pose_1.header.frame_id = "/map";

  pose_1.pose.position.x = 3.45;
  pose_1.pose.position.y = -8.3;
  pose_1.pose.position.z = 0.0;

  pose_1.pose.orientation.x = 0.05;
  pose_1.pose.orientation.y = -1.3;
  pose_1.pose.orientation.z = 3.14;
  pose_1.pose.orientation.w = 1.0;

  auto entry_1 = blackboard::Entry<geometry_msgs::msg::PoseStamped>::make_shared(pose_1);
  client_1->add_entry("house", "location", entry_1->to_base());

  auto entry_1_got =
    blackboard::as<geometry_msgs::msg::PoseStamped>(client_1->get_entry("house", "location"));

  ASSERT_EQ(entry_1_got->data_.pose.position, pose_1.pose.position);
  ASSERT_EQ(entry_1_got->data_.pose.orientation, pose_1.pose.orientation);

  pose_1.pose.orientation.x = -0.05;

  ASSERT_NE(entry_1_got->data_.pose.orientation, pose_1.pose.orientation);

  finish = true;
  t.join();
}

TEST(blackboard, octomap_entry)
{
  auto blackboard = blackboard::BlackBoardNode::make_shared();
  auto client_1 = blackboard::BlackBoardClient::make_shared();

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 8);
  exe.add_node(blackboard->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  octomap_msgs::msg::Octomap fake_octomap;

  fake_octomap.binary = true;
  fake_octomap.id = "fake_bathroom";
  fake_octomap.resolution = 1.5;
  fake_octomap.data = {4, 8, 2, 9, 45, 57, 12, 59};

  auto entry_1 = blackboard::Entry<octomap_msgs::msg::Octomap>::make_shared(fake_octomap);
  client_1->add_entry("bathroom", "octomap", entry_1->to_base());

  auto entry_1_got =
    blackboard::as<octomap_msgs::msg::Octomap>(client_1->get_entry("bathroom", "octomap"));

  ASSERT_EQ(entry_1_got->data_.binary, true);
  ASSERT_EQ(entry_1_got->data_.id, "fake_bathroom");
  ASSERT_EQ(entry_1_got->data_.resolution, 1.5);
  ASSERT_EQ(entry_1_got->data_.data, fake_octomap.data);
  ASSERT_NE(entry_1_got->data_.binary, false);

  finish = true;
  t.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
