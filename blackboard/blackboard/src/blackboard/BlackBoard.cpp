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

}  // namespace blackboard