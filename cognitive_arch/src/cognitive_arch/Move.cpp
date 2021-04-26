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
// limitations under the License.

#include "cognitive_arch/Move.hpp"
#include <algorithm>
#include <vector>
#include <string>

namespace cognitive_arch
{

Move::Move(const std::string & name, const std::chrono::nanoseconds & rate)
: plansys2::ActionExecutorClient(name, rate)
{
  using namespace std::placeholders;
  pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose",
    10,
    std::bind(&Move::current_pos_callback, this, _1));
}

void Move::current_pos_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pos_ = msg->pose.pose;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Move::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  send_feedback(0.0, "Move starting");
  navigation_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    shared_from_this(),
    "navigate_to_pose");
  bool is_action_server_ready = false;

  do {
    RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");
    is_action_server_ready =
      navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
  } while (!is_action_server_ready);

  RCLCPP_INFO(get_logger(), "Navigation action server ready");
  auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action

  RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

  auto blackboard = blackboard::BlackBoardNode::make_shared();
  auto client = blackboard::BlackBoardClient::make_shared();

  auto wp =
    blackboard::as<geometry_msgs::msg::PoseStamped>(client->get_entry(wp_to_navigate, "location"));

  goal_pos_ = wp->data_;

  navigation_goal_.pose = goal_pos_;
  dist_to_move = getDistance(goal_pos_.pose, current_pos_);
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.feedback_callback = [this](
    NavigationGoalHandle::SharedPtr,
    NavigationFeedback feedback) {
      send_feedback(
        std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
        "Move running");
    };

  send_goal_options.result_callback = [this](auto) {
      finish(true, 1.0, "Move completed");
    };

  future_navigation_goal_handle_ =
    navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);

  return ActionExecutorClient::on_activate(previous_state);
}
double Move::getDistance(
  const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
{
  return sqrt(
    (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
    (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
}

}  // namespace cognitive_arch
