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

#include <type_traits>
#include <string>
#include <memory>
#include <vector>

#include "blackboard_msgs/msg/entry.hpp"

#include "blackboard/BlackBoardNode.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace blackboard
{

BlackBoardNode::BlackBoardNode()
: Node("blackboard")
{
  add_entry_service_ = create_service<blackboard_msgs::srv::AddEntry>(
    "blackboard/add_entry",
    std::bind(
      &BlackBoardNode::add_entry_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_entry_service_ = create_service<blackboard_msgs::srv::GetEntry>(
    "blackboard/get_entry",
    std::bind(
      &BlackBoardNode::get_entry_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  exist_parent_service_ = create_service<blackboard_msgs::srv::ExistParent>(
    "blackboard/exist_parent",
    std::bind(
      &BlackBoardNode::exist_parent_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  exist_entry_service_ = create_service<blackboard_msgs::srv::ExistEntry>(
    "blackboard/exist_entry",
    std::bind(
      &BlackBoardNode::exist_entry_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  remove_parent_service_ = create_service<blackboard_msgs::srv::RemoveParent>(
    "blackboard/remove_parent",
    std::bind(
      &BlackBoardNode::remove_parent_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  remove_entry_service_ = create_service<blackboard_msgs::srv::RemoveEntry>(
    "blackboard/remove_entry",
    std::bind(
      &BlackBoardNode::remove_entry_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_key_parents_service_ = create_service<blackboard_msgs::srv::GetKeyParents>(
    "blackboard/get_key_parents",
    std::bind(
      &BlackBoardNode::get_key_parents_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  new_entry_pub_ = create_publisher<blackboard_msgs::msg::Entry>(
    "blackboard", rclcpp::QoS(100).transient_local());
}

void
BlackBoardNode::add_entry_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<blackboard_msgs::srv::AddEntry::Request> request,
  const std::shared_ptr<blackboard_msgs::srv::AddEntry::Response> response)
{
  (void)request_header;

  blackboard_msgs::msg::Entry new_entry;
  new_entry.parent_key = request->entry.parent_key;
  new_entry.key = request->entry.key;
  new_entry.type = request->entry.type;

  switch (request->entry.type) {
    case blackboard_msgs::msg::Entry::BOOL_TYPE:
      {
        auto entry = blackboard::Entry<bool>::make_shared(request->entry.bool_entry);
        blackboard_.add_entry(request->entry.parent_key, request->entry.key, entry->to_base());
        new_entry.bool_entry = request->entry.bool_entry;
      }
      break;
    case blackboard_msgs::msg::Entry::STRING_TYPE:
      {
        auto entry = blackboard::Entry<std::string>::make_shared(request->entry.string_entry);
        blackboard_.add_entry(request->entry.parent_key, request->entry.key, entry->to_base());
        new_entry.string_entry = request->entry.string_entry;
      }
      break;
    case blackboard_msgs::msg::Entry::FLOAT_TYPE:
      {
        auto entry = blackboard::Entry<float>::make_shared(request->entry.float_entry);
        blackboard_.add_entry(request->entry.parent_key, request->entry.key, entry->to_base());
        new_entry.float_entry = request->entry.float_entry;
      }
      break;
    case blackboard_msgs::msg::Entry::POSESTAMPED_TYPE:
      {
        auto entry = blackboard::Entry<geometry_msgs::msg::PoseStamped>::make_shared(
          request->entry.posestamped_entry);
        blackboard_.add_entry(request->entry.parent_key, request->entry.key, entry->to_base());
        new_entry.posestamped_entry = request->entry.posestamped_entry;
      }
      break;
    case blackboard_msgs::msg::Entry::OCTOMAP_TYPE:
      {
        auto entry = blackboard::Entry<octomap_msgs::msg::Octomap>::make_shared(
          request->entry.octomap_entry);
        blackboard_.add_entry(request->entry.parent_key, request->entry.key, entry->to_base());
        new_entry.octomap_entry = request->entry.octomap_entry;
      }
      break;
    case blackboard_msgs::msg::Entry::VECTOR_DOUBLE_TYPE:
      {
        auto entry = blackboard::Entry<std::vector<double>>::make_shared(
          request->entry.vector_double_entry);
        blackboard_.add_entry(request->entry.parent_key, request->entry.key, entry->to_base());
        new_entry.vector_double_entry = request->entry.vector_double_entry;
      }
      break;
    default:
      break;
  }
  new_entry_pub_->publish(new_entry);
  response->success = true;
}

void
BlackBoardNode::get_entry_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<blackboard_msgs::srv::GetEntry::Request> request,
  const std::shared_ptr<blackboard_msgs::srv::GetEntry::Response> response)
{
  (void)request_header;
  auto entry = blackboard_.get_entry(request->parent_key, request->key);

  if (entry->get_type() == EntryBase::BOOL) {
    response->entry.type = blackboard_msgs::msg::Entry::BOOL_TYPE;
    response->entry.key = request->key;
    response->entry.bool_entry = blackboard::as<bool>(entry)->data_;
  }

  if (entry->get_type() == EntryBase::STRING) {
    response->entry.type = blackboard_msgs::msg::Entry::STRING_TYPE;
    response->entry.key = request->key;
    response->entry.string_entry = blackboard::as<std::string>(entry)->data_;
  }

  if (entry->get_type() == EntryBase::FLOAT) {
    response->entry.type = blackboard_msgs::msg::Entry::FLOAT_TYPE;
    response->entry.key = request->key;
    response->entry.float_entry = blackboard::as<float>(entry)->data_;
  }

  if (entry->get_type() == EntryBase::POSESTAMPED) {
    response->entry.type = blackboard_msgs::msg::Entry::POSESTAMPED_TYPE;
    response->entry.key = request->key;
    response->entry.posestamped_entry =
      blackboard::as<geometry_msgs::msg::PoseStamped>(entry)->data_;
  }

  if (entry->get_type() == EntryBase::OCTOMAP) {
    response->entry.type = blackboard_msgs::msg::Entry::OCTOMAP_TYPE;
    response->entry.key = request->key;
    response->entry.octomap_entry = blackboard::as<octomap_msgs::msg::Octomap>(entry)->data_;
  }

  if (entry->get_type() == EntryBase::VECTOR_DOUBLE) {
    response->entry.type = blackboard_msgs::msg::Entry::VECTOR_DOUBLE_TYPE;
    response->entry.key = request->key;
    response->entry.vector_double_entry = blackboard::as<std::vector<double>>(entry)->data_;
  }

  response->success = true;
}

void
BlackBoardNode::exist_parent_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<blackboard_msgs::srv::ExistParent::Request> request,
  const std::shared_ptr<blackboard_msgs::srv::ExistParent::Response> response)
{
  (void)request_header;

  response->exist = blackboard_.exist_parent(request->parent_key);
}

void
BlackBoardNode::exist_entry_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<blackboard_msgs::srv::ExistEntry::Request> request,
  const std::shared_ptr<blackboard_msgs::srv::ExistEntry::Response> response)
{
  (void)request_header;

  response->exist = blackboard_.exist_entry(request->parent_key, request->key);
}

void
BlackBoardNode::remove_parent_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<blackboard_msgs::srv::RemoveParent::Request> request,
  const std::shared_ptr<blackboard_msgs::srv::RemoveParent::Response> response)
{
  (void)request_header;

  blackboard_.remove_parent(request->parent_key);
  response->success = true;
}

void
BlackBoardNode::remove_entry_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<blackboard_msgs::srv::RemoveEntry::Request> request,
  const std::shared_ptr<blackboard_msgs::srv::RemoveEntry::Response> response)
{
  (void)request_header;

  blackboard_.remove_entry(request->parent_key, request->key);
  response->success = true;
}

void
BlackBoardNode::get_key_parents_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<blackboard_msgs::srv::GetKeyParents::Request> request,
  const std::shared_ptr<blackboard_msgs::srv::GetKeyParents::Response> response)
{
  (void)request_header;

  response->parents = blackboard_.get_key_parents(request->key);
  response->success = true;
}

}  // namespace blackboard
