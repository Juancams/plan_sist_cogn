// Copyright 2020 Noel Jiménez García
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

using SharedResponseMsg = practica_msgs::srv::GetString::Response::SharedPtr;
using SharedFutureMsg = std::shared_future<SharedResponseMsg>;
using SharedResponsePoint = practica_msgs::srv::GetPoseStamped::Response::SharedPtr;
using SharedFuturePoint = std::shared_future<SharedResponsePoint>;

class ServiceClient : public rclcpp::Node
{
public:
  explicit ServiceClient(const std::string & name)
  : Node(name)
  {
    msg_client = create_client<practica_msgs::srv::GetString>("get_string");
    point_client = create_client<practica_msgs::srv::GetPoseStamped>("get_point");

    while (!msg_client->wait_for_service(1s) && !point_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
    }

    timer = create_wall_timer(1s, std::bind(&ServiceClient::get_info, this));
  }

  void get_info()
  {
    auto request_msg = std::make_shared<practica_msgs::srv::GetString::Request>();
    auto request_point = std::make_shared<practica_msgs::srv::GetPoseStamped::Request>();

    pending_msg.push_back(msg_client->async_send_request(request_msg));
    pending_point.push_back(point_client->async_send_request(request_point));

    auto it = pending_msg.begin();
    auto it2 = pending_point.begin();
    while (it != pending_msg.end() && it2 != pending_point.end()) {
      if (it->valid() && it->wait_for(10ms) == std::future_status::ready) {
        auto resp_str = it->get();

        RCLCPP_INFO(get_logger(), "Last msg received: [%s]", resp_str->last_msg.c_str());
        it = pending_msg.erase(it);
      } else {
        it++;
      }

      if (it2->valid() && it2->wait_for(10ms) == std::future_status::ready) {
        auto resp_pose = it2->get();

        RCLCPP_INFO(get_logger(), "Last point received: [x = %g; y = %g; z = %g]",
          resp_pose->last_pose.position.x,
          resp_pose->last_pose.position.y,
          resp_pose->last_pose.position.z);

        it2 = pending_point.erase(it2);
      } else {
        it2++;
      }
    }
  }

private:
  rclcpp::Client<practica_msgs::srv::GetString>::SharedPtr msg_client;
  rclcpp::Client<practica_msgs::srv::GetPoseStamped>::SharedPtr point_client;
  rclcpp::TimerBase::SharedPtr timer;

  std::list<SharedFutureMsg> pending_msg;
  std::list<SharedFuturePoint> pending_point;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto nodo_E = std::make_shared<ServiceClient>("nodo_E");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nodo_E);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
