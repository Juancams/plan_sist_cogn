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

#ifndef BLACKBOARD__BLACKBOARDCLIENT_HPP_
#define BLACKBOARD__BLACKBOARDCLIENT_HPP_

#include <memory>
#include <vector>
#include <string>

#include "blackboard_msgs/srv/add_entry.hpp"
#include "blackboard_msgs/srv/get_entry.hpp"
#include "blackboard_msgs/srv/exist_parent.hpp"
#include "blackboard_msgs/srv/exist_entry.hpp"
#include "blackboard_msgs/srv/remove_parent.hpp"
#include "blackboard_msgs/srv/remove_entry.hpp"
#include "blackboard_msgs/srv/get_key_parents.hpp"

#include "blackboard/BlackBoardInterface.hpp"

#include "rclcpp/rclcpp.hpp"

namespace blackboard
{

class BlackBoardClient : public BlackBoardInterface
{
public:
  using Ptr = std::shared_ptr<BlackBoardClient>;
  static Ptr make_shared()
  {
    return std::make_shared<BlackBoardClient>();
  }

  BlackBoardClient();

  void add_entry(const std::string & parent_key, const std::string & key, EntryBase::Ptr entry);
  EntryBase::Ptr get_entry(const std::string & parent_key, const std::string & key);
  bool exist_parent(const std::string & parent_key);
  bool exist_entry(const std::string & parent_key, const std::string & key);
  void remove_entry(const std::string & parent_key, const std::string & key);
  void remove_parent(const std::string & parent_key);
  std::vector<std::string> get_key_parents(const std::string & key);

private:
  rclcpp::Node::SharedPtr client_node_;

  rclcpp::Client<blackboard_msgs::srv::AddEntry>::SharedPtr
    add_entry_client_;
  rclcpp::Client<blackboard_msgs::srv::GetEntry>::SharedPtr
    get_entry_client_;
  rclcpp::Client<blackboard_msgs::srv::ExistParent>::SharedPtr
    exist_parent_client_;
  rclcpp::Client<blackboard_msgs::srv::ExistEntry>::SharedPtr
    exist_entry_client_;
  rclcpp::Client<blackboard_msgs::srv::RemoveParent>::SharedPtr
    remove_parent_client_;
  rclcpp::Client<blackboard_msgs::srv::RemoveEntry>::SharedPtr
    remove_entry_client_;
  rclcpp::Client<blackboard_msgs::srv::GetKeyParents>::SharedPtr
    get_key_parents_client_;
};

}  // namespace blackboard

#endif  // BLACKBOARD__BLACKBOARDCLIENT_HPP_
