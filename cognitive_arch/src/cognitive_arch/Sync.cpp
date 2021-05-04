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

#include <memory>

#include "cognitive_arch/Sync.hpp"

#include "rclcpp/rclcpp.hpp"
#include "blackboard_msgs/msg/entry.hpp"

#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

using std::placeholders::_1;

namespace cognitive_arch
{

Sync::Sync()
: Node("sync")
{
  node_aux = rclcpp::Node::make_shared("node_aux_sync");
  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(node_aux);
  blackboard_sub_ = create_subscription<blackboard_msgs::msg::Entry>(
    "blackboard", rclcpp::QoS(100).transient_local(),
    std::bind(&Sync::entriesCallback, this, _1));
}

void
Sync::entriesCallback(const blackboard_msgs::msg::Entry::SharedPtr msg)
{
  if (strcmp(msg->key.c_str(), "location") == 0) {
    RCLCPP_INFO(get_logger(), "%s->%s", msg->parent_key.c_str(), msg->key.c_str());
    problem_expert_->addInstance(plansys2::Instance{msg->parent_key.c_str(), msg->key.c_str()});
  }

  if (strcmp(msg->key.c_str(), "at") == 0) {
    RCLCPP_INFO(get_logger(), "%s->%s", msg->parent_key.c_str(), "location");
    problem_expert_->addInstance(plansys2::Instance{msg->parent_key.c_str(), "location"});
  }
}

}  // namespace cognitive_arch
