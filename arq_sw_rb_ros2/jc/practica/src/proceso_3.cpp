// Copyright 2020 Juan Carlos Manzanares Serrano
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
#include <future>
#include <string>
#include <list>
#include "rclcpp/rclcpp.hpp"
#include "practica_msgs/srv/get_string.hpp"
#include "practica_msgs/srv/get_pose_stamped.hpp"

using namespace std::chrono_literals;

using SharedResponse1 = practica_msgs::srv::GetString::Response::SharedPtr;
using SharedFuture1 = std::shared_future<SharedResponse1>;

using SharedResponse2 = practica_msgs::srv::GetPoseStamped::Response::SharedPtr;
using SharedFuture2 = std::shared_future<SharedResponse2>;

class NodeClient : public rclcpp::Node
{
public:
  explicit NodeClient(const std::string & name)
  : Node(name)
  {
    clientString = create_client<practica_msgs::srv::GetString>("get_string");
    clientPoseStamped = create_client<practica_msgs::srv::GetPoseStamped>("get_pose_stamped");

    while ((!clientString->wait_for_service(1s)) && (!clientPoseStamped->wait_for_service(1s))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }

    timerString = create_wall_timer(1s, std::bind(
          &NodeClient::timerString_callback, this));
    timerPoseStamped = create_wall_timer(1s, std::bind(
          &NodeClient::timerPoseStamped_callback, this));
  }

  void timerString_callback()
  {
    auto requestString = std::make_shared<practica_msgs::srv::GetString::Request>();

    pending_responses_string.push_back(clientString->async_send_request(requestString));

    auto it = pending_responses_string.begin();
    while (it != pending_responses_string.end()) {
      if (it->valid() && it->wait_for(100ms) == std::future_status::ready) {
        auto resp = it->get();

        RCLCPP_INFO(get_logger(), "Response by nodo_B: [%s] ",
          resp->number.c_str());

        it = pending_responses_string.erase(it);
      } else {
        ++it;
      }
    }
  }

  void timerPoseStamped_callback()
  {
    auto requestPoseStamped = std::make_shared<practica_msgs::srv::GetPoseStamped::Request>();

    pending_responses_pose_stamped.push_back(
      clientPoseStamped->async_send_request(requestPoseStamped));

    auto it = pending_responses_pose_stamped.begin();
    while (it != pending_responses_pose_stamped.end()) {
      if (it->valid() && it->wait_for(100ms) == std::future_status::ready) {
        auto resp = it->get();

        RCLCPP_INFO(get_logger(), "Response by nodo_D: [%f | %f | %f] ",
          resp->x, resp->y, resp->z);

        it = pending_responses_pose_stamped.erase(it);
      } else {
        ++it;
      }
    }
  }

private:
  rclcpp::Client<practica_msgs::srv::GetString>::SharedPtr clientString;
  rclcpp::TimerBase::SharedPtr timerString;
  std::list<SharedFuture1> pending_responses_string;

  rclcpp::Client<practica_msgs::srv::GetPoseStamped>::SharedPtr clientPoseStamped;
  rclcpp::TimerBase::SharedPtr timerPoseStamped;
  std::list<SharedFuture2> pending_responses_pose_stamped;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto nodo_E = std::make_shared<NodeClient>("nodo_E");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nodo_E);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
