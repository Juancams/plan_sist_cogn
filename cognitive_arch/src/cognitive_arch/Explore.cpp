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

#include <string>
#include "cognitive_arch/Explore.hpp"


namespace cognitive_arch
{
Explore::Explore(const std::string & name, const std::chrono::nanoseconds & rate)
: plansys2::ActionExecutorClient(name, rate)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Explore::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  progress_ = 0.0;

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  cmd_vel_pub_->on_activate();

  return ActionExecutorClient::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Explore::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  cmd_vel_pub_->on_deactivate();

  return ActionExecutorClient::on_deactivate(previous_state);
}

void Explore::do_work()
{
  if (progress_ < 3.0) {
    progress_ += 0.1;

    send_feedback(progress_, "Patrol running");

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.5;

    cmd_vel_pub_->publish(cmd);
  } else {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    cmd_vel_pub_->publish(cmd);

    finish(true, 1.0, "Patrol completed");
  }
}

}  // namespace cognitive_arch
