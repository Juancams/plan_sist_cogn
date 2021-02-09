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

#include <time.h>
#include <iostream>
#include <cstdlib>

#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "practica_msgs/srv/get_pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using std::placeholders::_2;
//

class MyLifeCycleNodePublisher : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifeCycleNodePublisher(const std::string & name, const std::chrono::nanoseconds & rate)
  : rclcpp_lifecycle::LifecycleNode(name)
  {
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/topic_CD", 100);

    timer_ = create_wall_timer(
      rate, std::bind(&MyLifeCycleNodePublisher::timer_callback, this));
  }

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(),
      "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(),
      "[%s] Activating from [%s] state...", get_name(), state.label().c_str());

    pub_->on_activate();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(),
      "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());

    pub_->on_deactivate();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(),
      "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());

    pub_.reset();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(),
      "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());

    pub_.reset();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_error(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(),
      "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  void timer_callback()
  {
    if (pub_->is_activated()) {
      geometry_msgs::msg::PoseStamped msg;
      msg.pose.position.x = rand_r(&aux);
      msg.pose.position.y = rand_r(&aux);
      msg.pose.position.z = rand_r(&aux);

      RCLCPP_INFO(get_logger(), "Publishing [x: %f, y: %f, z: %f]",
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

      pub_->publish(msg);
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};


class MyLifeCycleNodeSubscriber : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MyLifeCycleNodeSubscriber(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name), x(0), y(0),
    z(0)
  {
  }

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...",
      get_name(), state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...",
      get_name(), state.label().c_str());

    service = create_service<practica_msgs::srv::GetPoseStamped>(
      "get_pose_stamped",
      std::bind(&MyLifeCycleNodeSubscriber::send_info, this, _1, _2));

    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/topic_CD", 100,
      std::bind(&MyLifeCycleNodeSubscriber::callback, this, _1));

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...",
      get_name(), state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...",
      get_name(), state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...",
      get_name(), state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_error(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...",
      get_name(), state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }

  void callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    z = msg->pose.position.z;

    RCLCPP_INFO(this->get_logger(), "I heard [x: %f, y: %f, z: %f] ",
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }

  void send_info(
    const
    std::shared_ptr<practica_msgs::srv::GetPoseStamped::Request> request,
    std::shared_ptr<practica_msgs::srv::GetPoseStamped::Response> response)
  {
    response->x_final = x;
    response->y_final = y;
    response->z_final = z;
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Service<practica_msgs::srv::GetPoseStamped>::SharedPtr service;
  float x;
  float y;
  float z;
};


int main(int argc, char * argv[])
{
  unsigned int aux = time(NULL);
  aux = time(NULL);
  rclcpp::init(argc, argv);

  auto nodo_C = std::make_shared<MyLifeCycleNodePublisher>("nodo_C", 100ms);
  auto nodo_D = std::make_shared<MyLifeCycleNodeSubscriber>("nodo_D");

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(nodo_C->get_node_base_interface());
  executor.add_node(nodo_D->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
