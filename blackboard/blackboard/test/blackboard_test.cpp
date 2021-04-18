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

#include "blackboard/BlackBoard.hpp"

TEST(blackboard, add_get_entry)
{
  auto blackboard = blackboard::BlackBoard::make_shared();

  auto entry_1 = blackboard::Entry<bool>::make_shared(true);

  auto entry_base = entry_1->to_base();
  auto entry_2 = blackboard::Entry<std::string>::make_shared("Hi!!");

  blackboard->add_entry("room", "my_entry_1", entry_1->to_base());
  blackboard->add_entry("room", "my_entry_2", entry_2->to_base());

  auto entry_1_got = blackboard::as<bool>(blackboard->get_entry("room", "my_entry_1"));
  auto entry_2_got = blackboard::as<std::string>(blackboard->get_entry("room", "my_entry_2"));

  ASSERT_TRUE(entry_1_got->data_);
  ASSERT_EQ(entry_2_got->data_, "Hi!!");
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

TEST(blackboard, rm_and_modify)
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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
