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

#include <unistd.h>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "practica_msgs/srv/get_string.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

class MyNodeSubscriber : public rclcpp::Node
{
public:
  explicit MyNodeSubscriber(const std::string & name)
  : Node(name)
  {
    sub_ = create_subscription<std_msgs::msg::String>(
      "topic_AB", rclcpp::QoS(100).best_effort(), std::bind(&MyNodeSubscriber::callback, this, _1));

    service = create_service<practica_msgs::srv::GetString>(
      "get_string", std::bind(&MyNodeSubscriber::send_info, this, _1, _2));
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg)
  {
    mensaje = msg->data.c_str();
    RCLCPP_INFO(this->get_logger(), "I heard: [%s] in %s",
      msg->data.c_str(), get_name());
  }

  void send_info(
    const
    std::shared_ptr<practica_msgs::srv::GetString::Request> request,
    std::shared_ptr<practica_msgs::srv::GetString::Response> response)
  {
    response->final_sentence = mensaje;
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Service<practica_msgs::srv::GetString>::SharedPtr service;
  std::string mensaje;
};


class MyNodePublisher : public rclcpp::Node
{
public:
  MyNodePublisher(const std::string & name, const std::chrono::nanoseconds & rate)
  : Node(name),
    counter_(0)
  {
    pub_ = create_publisher<std_msgs::msg::String>("topic_AB", rclcpp::QoS(100).best_effort());
    timer_ = create_wall_timer(
      rate, std::bind(&MyNodePublisher::timer_callback, this));
  }

  void timer_callback()
  {
    std_msgs::msg::String message;
    message.data = "message: " + std::to_string(counter_++);

    RCLCPP_INFO(get_logger(), "Publishing [%s]", message.data.c_str());

    pub_->publish(message);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  int counter_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto nodo_A = std::make_shared<MyNodePublisher>("nodo_A", 50ms);
  auto nodo_B = std::make_shared<MyNodeSubscriber>("nodo_B");

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(nodo_A);
  executor.add_node(nodo_B);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
