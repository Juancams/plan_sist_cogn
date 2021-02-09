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

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "practica_msgs/srv/get_string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class MyNodePublisher
  : public rclcpp::Node
{
public:
  explicit MyNodePublisher(const std::string & name)
  : Node(name), N(0)
  {
    pub = create_publisher<std_msgs::msg::String>(
      "topic_AB", rclcpp::QoS(10).best_effort());
  }

  void doWork()
  {
    std_msgs::msg::String message;
    message.data = "message: " + std::to_string(N++);
    RCLCPP_INFO(get_logger(), "Publishing [%s]", message.data.c_str());
    pub->publish(message);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
  int N;
};

class MyNodeSubscriber
  : public rclcpp::Node
{
public:
  explicit MyNodeSubscriber(const std::string & name)
  : Node(name)
  {
    sub = create_subscription<std_msgs::msg::String>(
      "topic_AB", rclcpp::QoS(10).best_effort(),
      std::bind(&MyNodeSubscriber::messageCallback, this, _1));

    service = create_service<practica_msgs::srv::GetString>(
      "get_string", std::bind(&MyNodeSubscriber::response_sentence, this, _1, _2));

    RCLCPP_INFO(get_logger(), "Ready.");
  }

private:
  void messageCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    my_msg.data = msg->data;
    RCLCPP_INFO(get_logger(), "I heard: [%s]", msg->data.c_str());
  }

  void response_sentence(
    std::shared_ptr<practica_msgs::srv::GetString::Request> request,
    std::shared_ptr<practica_msgs::srv::GetString::Response> response)
  {
    response->number = my_msg.data;

    RCLCPP_INFO(get_logger(), "Sending response: [%s]",
      response->number.c_str());
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
  std_msgs::msg::String my_msg;
  rclcpp::Service<practica_msgs::srv::GetString>::SharedPtr service;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto nodo_A = std::make_shared<MyNodePublisher>("nodo_A");
  auto nodo_B = std::make_shared<MyNodeSubscriber>("nodo_B");

  rclcpp::Rate loop_rate(20);
  while (rclcpp::ok()) {
    nodo_A->doWork();
    rclcpp::spin_some(nodo_A);
    rclcpp::spin_some(nodo_B);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
