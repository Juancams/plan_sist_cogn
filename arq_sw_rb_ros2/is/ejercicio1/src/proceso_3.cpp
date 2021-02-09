// Copyright 2020 Isabel Cebollada
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
#include <chrono>
#include <string>
#include <list>

#include "rclcpp/rclcpp.hpp"
#include "practica_msgs/srv/get_string.hpp"
#include "practica_msgs/srv/get_pose_stamped.hpp"

using namespace std::chrono_literals;

using SharedResponse_BE = practica_msgs::srv::GetString::Response::SharedPtr;
using SharedFuture_BE = std::shared_future<SharedResponse_BE>;
using SharedResponse_DE = practica_msgs::srv::GetPoseStamped::Response::SharedPtr;
using SharedFuture_DE = std::shared_future<SharedResponse_DE>;

class MyNodeClient : public rclcpp::Node
{
public:
  explicit MyNodeClient(const std::string & name)
  : Node(name)
  {
    client_BE = create_client<practica_msgs::srv::GetString>("get_string");
    client_DE = create_client<practica_msgs::srv::GetPoseStamped>("get_pose_stamped");

    while ((!client_BE->wait_for_service(1s)) || (!client_DE->wait_for_service(1s))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }
  }

  void timer_callback()
  {
    auto request_BE = std::make_shared<practica_msgs::srv::GetString::Request>();

    pending_responses_BE.push_back(client_BE->async_send_request(request_BE));

    auto result_BE = client_BE->async_send_request(request_BE);
    rclcpp::spin_until_future_complete(shared_from_this(), result_BE);

    auto it_BE = pending_responses_BE.begin();
    while (it_BE != pending_responses_BE.end()) {
      if (it_BE->valid() && it_BE->wait_for(100ms) == std::future_status::ready) {
        auto resp_BE = it_BE->get();

        RCLCPP_INFO(get_logger(), "Last string: [%s]",
          resp_BE->final_sentence.c_str());

        it_BE = pending_responses_BE.erase(it_BE);
      } else {
        ++it_BE;
      }
    }

    auto request_DE = std::make_shared<practica_msgs::srv::GetPoseStamped::Request>();

    pending_responses_DE.push_back(client_DE->async_send_request(request_DE));

    auto result_DE = client_DE->async_send_request(request_DE);
    rclcpp::spin_until_future_complete(shared_from_this(), result_DE);

    auto it_DE = pending_responses_DE.begin();
    while (it_DE != pending_responses_DE.end()) {
      if (it_DE->valid() && it_DE->wait_for(100ms) == std::future_status::ready) {
        auto resp_DE = it_DE->get();

        RCLCPP_INFO(get_logger(), "x, y, z: [%f, %f, %f]",
          resp_DE->x_final, resp_DE->y_final, resp_DE->z_final);

        it_DE = pending_responses_DE.erase(it_DE);
      } else {
        ++it_DE;
      }
    }
  }

private:
  rclcpp::Client<practica_msgs::srv::GetString>::SharedPtr client_BE;
  rclcpp::Client<practica_msgs::srv::GetPoseStamped>::SharedPtr client_DE;
  std::list<SharedFuture_BE> pending_responses_BE;
  std::list<SharedFuture_DE> pending_responses_DE;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto nodo_E = std::make_shared<MyNodeClient>("nodo_E");

  rclcpp::Rate loop_rate(1000ms);

  while (rclcpp::ok()) {
    nodo_E->timer_callback();
    rclcpp::spin_some(nodo_E->get_node_base_interface());
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
