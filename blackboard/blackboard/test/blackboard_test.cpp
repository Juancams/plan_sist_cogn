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

#include "geometry_msgs/msg/pose.hpp"
#include "blackboard/BlackBoard.hpp"

TEST(blackboard, add_get_entry)
{
  auto blackboard = blackboard::BlackBoard::make_shared();

  auto entry_1 = blackboard::Entry<bool>::make_shared(true);
  auto entry_2 = blackboard::Entry<std::string>::make_shared("Hi!!");
  auto entry_3 = blackboard::Entry<float>::make_shared(3.76);

  blackboard->add_entry("room", "my_entry_1", entry_1->to_base());
  blackboard->add_entry("room", "my_entry_2", entry_2->to_base());
  blackboard->add_entry("room", "my_entry_3", entry_3->to_base());

  auto entry_1_got = blackboard::as<bool>(blackboard->get_entry("room", "my_entry_1"));
  auto entry_2_got = blackboard::as<std::string>(blackboard->get_entry("room", "my_entry_2"));
  auto entry_3_got = blackboard::as<float>(blackboard->get_entry("room", "my_entry_3"));

  ASSERT_TRUE(entry_1_got->data_);
  ASSERT_EQ(entry_2_got->data_, "Hi!!");
  ASSERT_EQ(entry_3_got->data_, (float)3.76);
}

TEST(blackboard, check_entry_parent)
{
  auto blackboard = blackboard::BlackBoard::make_shared();

  auto entry_1 = blackboard::Entry<bool>::make_shared(true);
  auto entry_2 = blackboard::Entry<std::string>::make_shared("entry2");

  blackboard->add_entry("room", "my_entry_1", entry_1->to_base());
  blackboard->add_entry("room", "my_entry_2", entry_2->to_base());

  auto exists_parent_1 = blackboard->exist_parent("room");
  auto exists_entry_2 = blackboard->exist_entry("room", "my_entry_2");
  auto unreal_entry = blackboard->exist_entry("room", "false_entry");
  auto unreal_parent = blackboard->exist_parent("false_parent");

  ASSERT_FALSE(unreal_entry);
  ASSERT_FALSE(unreal_parent);
  ASSERT_TRUE(exists_parent_1);
  ASSERT_TRUE(exists_entry_2);
}

TEST(blackboard, remove_entry_parent)
{
  auto blackboard = blackboard::BlackBoard::make_shared();

  auto entry_1 = blackboard::Entry<std::string>::make_shared("will_be_removed");
  auto entry_2 = blackboard::Entry<std::string>::make_shared("will_rm_parent");

  blackboard->add_entry("parent1", "to_remove", entry_1->to_base());
  blackboard->add_entry("parent2", "to_rm_parent", entry_2->to_base());

  blackboard->remove_entry("parent1", "to_remove");
  blackboard->remove_parent("parent2");

  auto removed_entry = blackboard->exist_entry("parent1", "to_remove");
  auto removed_parent = blackboard->exist_parent("parent2");
  auto removed_parent_entry = blackboard->exist_entry("parent2", "to_rm_parent");

  ASSERT_FALSE(removed_entry);
  ASSERT_FALSE(removed_parent);
  ASSERT_FALSE(removed_parent_entry);
}

TEST(blackboard, pose_entry)
{
  auto blackboard = blackboard::BlackBoard::make_shared();

  geometry_msgs::msg::Pose pose_1;

  pose_1.position.x = 3.45;
  pose_1.position.y = -8.3;
  pose_1.position.z = 0.0;

  pose_1.orientation.x = 0.05;
  pose_1.orientation.y = -1.3;
  pose_1.orientation.z = 3.14;
  pose_1.orientation.w = 1.0;

  auto entry_1 = blackboard::Entry<geometry_msgs::msg::Pose>::make_shared(pose_1);
  blackboard->add_entry("house", "location", entry_1->to_base());

  auto entry_1_got =
    blackboard::as<geometry_msgs::msg::Pose>(blackboard->get_entry("house", "location"));

  ASSERT_EQ(entry_1_got->data_.position, pose_1.position);
  ASSERT_EQ(entry_1_got->data_.orientation, pose_1.orientation);

  pose_1.orientation.x = -0.05;

  ASSERT_NE(entry_1_got->data_.orientation, pose_1.orientation);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
