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

#include <stdlib.h>
#include <time.h>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "practica_msgs/srv/get_pose_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class LifeCyclePublisher : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifeCyclePublisher(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name), seed(time(NULL))
  {
    pub = create_publisher<geometry_msgs::msg::PoseStamped>("topic_CD", 100);
    timer = create_wall_timer(100ms,
        std::bind(&LifeCyclePublisher::publish_point, this));
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
    pub->on_activate();
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...",
      get_name(), state.label().c_str());
    pub->on_deactivate();
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...",
      get_name(), state.label().c_str());
    pub.reset();
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...",
      get_name(), state.label().c_str());
    pub.reset();
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_error(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...",
      get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

private:
  void publish_point()
  {
    if (pub->is_activated()) {
      geometry_msgs::msg::PoseStamped msg;
      msg.pose.position.x = rand_r(&seed);
      msg.pose.position.y = rand_r(&seed);
      msg.pose.position.z = rand_r(&seed);

      RCLCPP_INFO(get_logger(), "Publishing [x = %g; y = %g; z = %g]",
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
      pub->publish(msg);
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
  unsigned int seed;
};

class LifeCycleSubscriber : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifeCycleSubscriber(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name)
  {}

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
    sub = create_subscription<geometry_msgs::msg::PoseStamped>("topic_CD",
        100, std::bind(&LifeCycleSubscriber::callback, this, _1));
    service = create_service<practica_msgs::srv::GetPoseStamped>(
      "get_point", std::bind(&LifeCycleSubscriber::send_point, this, _1, _2));
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...",
      get_name(), state.label().c_str());
    sub = nullptr;
    service = nullptr;
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...",
      get_name(), state.label().c_str());
    sub = nullptr;
    service = nullptr;
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...",
      get_name(), state.label().c_str());
    sub = nullptr;
    service = nullptr;
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_error(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...",
      get_name(), state.label().c_str());
    sub = nullptr;
    service = nullptr;
    return CallbackReturnT::SUCCESS;
  }

private:
  void callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "I heard: [x = %g; y = %g; z = %g]",
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    lastPoint = msg->pose;
  }

  void send_point(
    const std::shared_ptr<practica_msgs::srv::GetPoseStamped::Request> request,
    std::shared_ptr<practica_msgs::srv::GetPoseStamped::Response> response)
  {
    response->last_pose = lastPoint;
    RCLCPP_INFO(get_logger(), "Sending last point: [x = %g; y = %g; z = %g]",
      response->last_pose.position.x,
      response->last_pose.position.y,
      response->last_pose.position.z);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;
  rclcpp::Service<practica_msgs::srv::GetPoseStamped>::SharedPtr service;
  geometry_msgs::msg::Pose lastPoint;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto nodo_C = std::make_shared<LifeCyclePublisher>("nodo_C");
  auto nodo_D = std::make_shared<LifeCycleSubscriber>("nodo_D");

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(nodo_C->get_node_base_interface());
  exec.add_node(nodo_D->get_node_base_interface());

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
