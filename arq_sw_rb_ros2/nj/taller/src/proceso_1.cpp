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
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "practica_msgs/srv/get_string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class PublisherNode : public rclcpp::Node
{
public:
  explicit PublisherNode(const std::string & name)
  : Node(name), n(0)
  {
    pub = create_publisher<std_msgs::msg::String>(
      "topic_AB", rclcpp::QoS(100).best_effort());
    timer = create_wall_timer(
      50ms, std::bind(&PublisherNode::publish_msg, this));
  }

private:
  void publish_msg()
  {
    std_msgs::msg::String msg;
    msg.data = "message: " + std::to_string(n++);

    RCLCPP_INFO(get_logger(), "Publishing [%s]", msg.data.c_str());
    pub->publish(msg);
  }

private:
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
  int n;
};

class SubscriberNode : public rclcpp::Node
{
public:
  explicit SubscriberNode(const std::string & name)
  : Node(name)
  {
    service = create_service<practica_msgs::srv::GetString>(
      "get_string", std::bind(&SubscriberNode::send_string, this, _1, _2));

    sub = create_subscription<std_msgs::msg::String>("topic_AB",
        rclcpp::QoS(100).best_effort(), std::bind(&SubscriberNode::callback,
        this, _1));
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
    lastMsg = msg->data;
  }

  void send_string(
    const std::shared_ptr<practica_msgs::srv::GetString::Request> request,
    std::shared_ptr<practica_msgs::srv::GetString::Response> response)
  {
    response->last_msg = lastMsg;
    RCLCPP_INFO(get_logger(), "Sending last message: [%s]", response->last_msg.c_str());
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
  rclcpp::Service<practica_msgs::srv::GetString>::SharedPtr service;
  std::string lastMsg;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto nodo_A = std::make_shared<PublisherNode>("nodo_A");
  auto nodo_B = std::make_shared<SubscriberNode>("nodo_B");

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(nodo_A);
  exec.add_node(nodo_B);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
