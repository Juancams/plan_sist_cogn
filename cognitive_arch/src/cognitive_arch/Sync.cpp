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
#include <vector>
#include <memory>

#include "cognitive_arch/Sync.hpp"
#include "blackboard/BlackBoardNode.hpp"
#include "blackboard/BlackBoardClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "blackboard_msgs/msg/entry.hpp"

#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "octomap_msgs/msg/octomap.hpp"
#include "octomap/octomap.h"
#include "octomap/OcTree.h"
#include "octomap/OcTreeKey.h"
#include "octomap/ColorOcTree.h"
#include "octomap/octomap_utils.h"
#include "octomap_msgs/conversions.h"

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

    auto client_ = blackboard::BlackBoardClient::make_shared();
    auto rooms = client_->get_key_parents("octomap");
    
    for (int i = 0; i < rooms.size(); i++) {
      const auto entry =
      blackboard::as<octomap_msgs::msg::Octomap>(client_->get_entry(rooms[i], "octomap"));

      auto entry_point =
      blackboard::as<geometry_msgs::msg::PoseStamped>(client_->get_entry(msg->parent_key, "point"));

      octomap::ColorOcTree* octomap_ = dynamic_cast<octomap::ColorOcTree*>(octomap_msgs::msgToMap(entry->data_));

      octomap::OcTreeKey key = octomap_->coordToKey(octomap::point3d(entry_point->data_.pose.position.x,entry_point->data_.pose.position.y,entry_point->data_.pose.position.z));
      auto node = octomap_->search(key);

       if (node != nullptr) {
         auto entry_loc = blackboard::Entry<std::string>::make_shared(rooms[i]);
         client_->add_entry(msg->key, "at", entry_loc->to_base());
         break;
       }
    }
  }

}

}  // namespace cognitive_arch
