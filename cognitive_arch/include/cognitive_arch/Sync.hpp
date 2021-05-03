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

#ifndef COGNITIVE_ARCH__SYNC_HPP_
#define COGNITIVE_ARCH__SYNC_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "blackboard_msgs/msg/entry.hpp"

#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"


namespace cognitive_arch
{

class Sync : public rclcpp::Node
{

public:
  explicit Sync();

protected:
  void entriesCallback(const blackboard_msgs::msg::Entry::SharedPtr msg);

  rclcpp::Node::SharedPtr node_aux;
  rclcpp::Subscription<blackboard_msgs::msg::Entry>::SharedPtr blackboard_sub_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;;
};

}  // namespace cognitive_arch

#endif  // COGNITIVE_ARCH__SYNC_HPP_