// Copyright 2020 Irene Bandera Moreno
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

#include <unistd.h>
#include <memory>
#include <future>
#include <list>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "practica_msgs/srv/get_string.hpp"
#include "practica_msgs/srv/get_pose_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

using SharedResponseAB = practica_msgs::srv::GetString::Response::SharedPtr;
using SharedFutureAB = std::shared_future<SharedResponseAB>;

using SharedResponseCD = practica_msgs::srv::GetPoseStamped::Response::SharedPtr;
using SharedFutureCD = std::shared_future<SharedResponseCD>;

class MyNode : public rclcpp::Node
{
public:
  explicit MyNode(const std::string & name)
  : Node(name)
  {
    clientAB_ = create_client<practica_msgs::srv::GetString>("get_string");

    clientCD_ = create_client<practica_msgs::srv::GetPoseStamped>("get_pose_stamped");

    while (!clientAB_->wait_for_service(1s) && !clientCD_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }

    timerAB_ = create_wall_timer(1s, std::bind(&MyNode::timer_callbackAB, this));

    timerCD_ = create_wall_timer(1s, std::bind(&MyNode::timer_callbackCD, this));
  }

  void timer_callbackAB()
  {
    auto request = std::make_shared<practica_msgs::srv::GetString::Request>();
    request->request_sentence = "Give me the string!";

    pending_responsesAB_.push_back(clientAB_->async_send_request(request));

    auto it = pending_responsesAB_.begin();
    while (it != pending_responsesAB_.end()) {
      if (it->valid() && it->wait_for(100ms) == std::future_status::ready) {
        auto resp = it->get();

        RCLCPP_INFO(get_logger(), "String: [%s]",
          resp->answer_sentence.c_str());

        it = pending_responsesAB_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void timer_callbackCD()
  {
    auto request = std::make_shared<practica_msgs::srv::GetPoseStamped::Request>();
    request->request_sentence = "Give me the pose stamped!";

    pending_responsesCD_.push_back(clientCD_->async_send_request(request));

    auto it = pending_responsesCD_.begin();
    while (it != pending_responsesCD_.end()) {
      if (it->valid() && it->wait_for(100ms) == std::future_status::ready) {
        auto resp = it->get();

        RCLCPP_INFO(get_logger(), "Pose stamped: x: [%lf] y: [%lf] z: [%lf]",
          resp->x, resp->y, resp->z);

        it = pending_responsesCD_.erase(it);
      } else {
        ++it;
      }
    }
  }

private:
  rclcpp::Client<practica_msgs::srv::GetString>::SharedPtr clientAB_;
  rclcpp::TimerBase::SharedPtr timerAB_;
  std::list<SharedFutureAB> pending_responsesAB_;
  rclcpp::Client<practica_msgs::srv::GetPoseStamped>::SharedPtr clientCD_;
  rclcpp::TimerBase::SharedPtr timerCD_;
  std::list<SharedFutureCD> pending_responsesCD_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto nodo_E = std::make_shared<MyNode>("nodo_E");

  rclcpp::spin(nodo_E);

  rclcpp::shutdown();

  return 0;
}
