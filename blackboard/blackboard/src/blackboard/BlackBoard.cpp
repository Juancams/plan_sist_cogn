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

#include <memory>
#include <string>
#include <vector>

#include "blackboard/BlackBoard.hpp"

namespace blackboard
{

void
BlackBoard::add_entry(
  const std::string & parent_key, const std::string & key, EntryBase::Ptr entry)
{
  if (!exist_entry(parent_key, key)) {
    entries_[parent_key][key] = entry;
  }
}

EntryBase::Ptr
BlackBoard::get_entry(const std::string & parent_key, const std::string & key)
{
  if (!exist_entry(parent_key, key)) {
    return nullptr;
  }

  return entries_[parent_key][key];
}

bool
BlackBoard::exist_parent(const std::string & parent_key)
{
  return entries_.find(parent_key) != entries_.end();
}

bool
BlackBoard::exist_entry(const std::string & parent_key, const std::string & key)
{
  return exist_parent(parent_key) &&
         (entries_[parent_key].find(key) != entries_[parent_key].end());
}

void
BlackBoard::remove_entry(const std::string & parent_key, const std::string & key)
{
  if (BlackBoard::exist_entry(parent_key, key)) {
    entries_[parent_key].erase(key);
  }
}

void
BlackBoard::remove_parent(const std::string & parent_key)
{
  if (BlackBoard::exist_parent(parent_key)) {
    entries_[parent_key].clear();
    entries_.erase(parent_key);
  }
}

std::vector<std::string>
BlackBoard::get_key_parents(const std::string & key)
{
  std::vector<std::string> parents_list;

  auto it = entries_.begin();
  for (it; it != entries_.end(); it++) {
    if (entries_[it->first].find(key) != entries_[it->first].end()) {
      parents_list.push_back(it->first);
    }
  }
  return parents_list;
}

}  // namespace blackboard
