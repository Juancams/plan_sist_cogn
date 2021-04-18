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
#include <memory>

#include "blackboard_msgs/srv/add_entry.hpp"
#include "blackboard_msgs/srv/get_entry.hpp"
#include "blackboard_msgs/srv/exist_parent.hpp"
#include "blackboard_msgs/srv/exist_entry.hpp"
#include "blackboard_msgs/srv/remove_parent.hpp"
#include "blackboard_msgs/srv/remove_entry.hpp"

#include "blackboard/BlackBoardClient.hpp"

#include "rclcpp/rclcpp.hpp"

namespace blackboard
{

BlackBoardClient::BlackBoardClient()
{
  client_node_ = rclcpp::Node::make_shared("blackboard_client");

  add_entry_client_ = client_node_->create_client<blackboard_msgs::srv::AddEntry>(
    "blackboard/add_entry");
  get_entry_client_ = client_node_->create_client<blackboard_msgs::srv::GetEntry>(
    "blackboard/get_entry");
  exist_parent_client_ = client_node_->create_client<blackboard_msgs::srv::ExistParent>(
    "blackboard/exist_parent");
  exist_entry_client_ = client_node_->create_client<blackboard_msgs::srv::ExistEntry>(
    "blackboard/exist_entry");
  remove_entry_client_ = client_node_->create_client<blackboard_msgs::srv::RemoveEntry>(
    "blackboard/remove_entry");
  remove_parent_client_ = client_node_->create_client<blackboard_msgs::srv::RemoveParent>(
    "blackboard/remove_parent");
}

void
BlackBoardClient::add_entry(
  const std::string & parent_key, const std::string & key, EntryBase::Ptr entry)
{
  while (!add_entry_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return;
    }
    RCLCPP_ERROR_STREAM(
      client_node_->get_logger(),
      add_entry_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<blackboard_msgs::srv::AddEntry::Request>();

  if (entry->get_type() == EntryBase::BOOL) {
    request->entry.type = blackboard_msgs::msg::Entry::BOOL_TYPE;
    request->entry.parent_key = parent_key;
    request->entry.key = key;
    request->entry.bool_entry = blackboard::as<bool>(entry)->data_;
  }

  if (entry->get_type() == EntryBase::STRING) {
    request->entry.type = blackboard_msgs::msg::Entry::STRING_TYPE;
    request->entry.parent_key = parent_key;
    request->entry.key = key;
    request->entry.string_entry = blackboard::as<std::string>(entry)->data_;
  }

  auto future_result = add_entry_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(client_node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return;
  }

  if (!future_result.get()->success) {
    RCLCPP_ERROR_STREAM(
      client_node_->get_logger(),
      add_entry_client_->get_service_name() << ": Error");
  }
}

EntryBase::Ptr
BlackBoardClient::get_entry(const std::string & parent_key, const std::string & key)
{
  while (!get_entry_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return nullptr;
    }
    RCLCPP_ERROR_STREAM(
      client_node_->get_logger(),
      get_entry_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<blackboard_msgs::srv::GetEntry::Request>();

  request->parent_key = parent_key;
  request->key = key;

  auto future_result = get_entry_client_->async_send_request(request);

  EntryBase::Ptr ret = nullptr;
  if (rclcpp::spin_until_future_complete(client_node_, future_result, std::chrono::seconds(1)) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    switch (future_result.get()->entry.type) {
      case blackboard_msgs::msg::Entry::BOOL_TYPE:
        {
          ret = blackboard::Entry<bool>::make_shared(future_result.get()->entry.bool_entry);
          return ret;
        }
        break;
      case blackboard_msgs::msg::Entry::STRING_TYPE:
        {
          ret = blackboard::Entry<std::string>::make_shared(
            future_result.get()->entry.string_entry);
          return ret;
        }
        break;
      default:
        break;
    }
  } else {
    RCLCPP_ERROR_STREAM(
      client_node_->get_logger(),
      add_entry_client_->get_service_name() << ": Error calling service");
  }

  if (!future_result.get()->success) {
    RCLCPP_ERROR_STREAM(
      client_node_->get_logger(),
      add_entry_client_->get_service_name() << ": Error");
  }

  return ret;
}

bool
BlackBoardClient::exist_parent(const std::string & parent_key)
{
  while (!exist_parent_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      client_node_->get_logger(),
      exist_parent_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<blackboard_msgs::srv::ExistParent::Request>();
  request->parent_key = parent_key;

  auto future_result = exist_parent_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(client_node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  return future_result.get()->exist;
}

bool
BlackBoardClient::exist_entry(const std::string & parent_key, const std::string & key)
{
  while (!exist_entry_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      client_node_->get_logger(),
      exist_entry_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<blackboard_msgs::srv::ExistEntry::Request>();
  request->parent_key = parent_key;
  request->key = key;

  auto future_result = exist_entry_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(client_node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  return future_result.get()->exist;
}

void
BlackBoardClient::remove_entry(const std::string & parent_key, const std::string & key)
{
  while (!remove_entry_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return;
    }
    RCLCPP_ERROR_STREAM(
      client_node_->get_logger(),
      remove_entry_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<blackboard_msgs::srv::RemoveEntry::Request>();
  request->parent_key = parent_key;
  request->key = key;

  auto future_result = remove_entry_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(client_node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return;
  }

  if (!future_result.get()->success) {
    RCLCPP_ERROR_STREAM(
      client_node_->get_logger(),
      add_entry_client_->get_service_name() << ": Error");
  }
}

void
BlackBoardClient::remove_parent(const std::string & parent_key)
{
  while (!remove_parent_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return;
    }
    RCLCPP_ERROR_STREAM(
      client_node_->get_logger(),
      remove_parent_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<blackboard_msgs::srv::RemoveParent::Request>();
  request->parent_key = parent_key;

  auto future_result = remove_parent_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(client_node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return;
  }

  if (!future_result.get()->success) {
    RCLCPP_ERROR_STREAM(
      client_node_->get_logger(),
      add_entry_client_->get_service_name() << ": Error");
  }
}

}  // namespace blackboard
