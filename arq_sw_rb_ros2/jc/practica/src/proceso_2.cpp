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

#include <time.h>
#include <memory>
#include <string>
#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "practica_msgs/srv/get_pose_stamped.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using std::placeholders::_2;

class LifeCycleNodePub : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifeCycleNodePub(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name)
  {
    pub = create_publisher<geometry_msgs::msg::PoseStamped>("topic_CD", 100);
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

  void do_work()
  {
    if (pub->is_activated()) {
      geometry_msgs::msg::PoseStamped msg;

      msg.header.frame_id = "coordenada";
      msg.pose.position.x = 10 * (rand_r(&seed) / static_cast<float>(RAND_MAX));
      msg.pose.position.y = 10 * (rand_r(&seed) / static_cast<float>(RAND_MAX));
      msg.pose.position.z = 10 * (rand_r(&seed) / static_cast<float>(RAND_MAX));
      msg.pose.orientation.x = 0;
      msg.pose.orientation.y = 0;
      msg.pose.orientation.z = 0;
      msg.pose.orientation.w = 1;
      pub->publish(msg);

      RCLCPP_INFO(this->get_logger(), "Publishing: X = %f | Y = %f | Z = %f",
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    }
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
  unsigned int seed;
};

class LifeCycleNodeSub : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifeCycleNodeSub(const std::string & name)
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

    sub = create_subscription<geometry_msgs::msg::PoseStamped>(
      "topic_CD", 10, std::bind(&LifeCycleNodeSub::messageCallback, this, _1));

    service = create_service<practica_msgs::srv::GetPoseStamped>(
      "get_pose_stamped", std::bind(&LifeCycleNodeSub::response_sentence, this, _1, _2));

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...",
      get_name(), state.label().c_str());

    sub = nullptr;

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...",
      get_name(), state.label().c_str());

    sub.reset();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...",
      get_name(), state.label().c_str());

    sub.reset();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_error(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...",
      get_name(), state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }

private:
  void messageCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    my_msg.pose.position.x = msg->pose.position.x;
    my_msg.pose.position.y = msg->pose.position.y;
    my_msg.pose.position.z = msg->pose.position.z;
    RCLCPP_INFO(this->get_logger(), "I heard: X = %f | Y = %f | Z = %f",
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }

  void response_sentence(
    std::shared_ptr<practica_msgs::srv::GetPoseStamped::Request> request,
    std::shared_ptr<practica_msgs::srv::GetPoseStamped::Response> response)
  {
    response->x = my_msg.pose.position.x;
    response->y = my_msg.pose.position.y;
    response->z = my_msg.pose.position.z;

    RCLCPP_INFO(get_logger(), "Sending back response: [%f | %f | %f]",
      response->x, response->y, response->z);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;
  geometry_msgs::msg::PoseStamped my_msg;
  rclcpp::Service<practica_msgs::srv::GetPoseStamped>::SharedPtr service;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  srand(time(NULL));

  auto nodo_C = std::make_shared<LifeCycleNodePub>("nodo_C");
  auto nodo_D = std::make_shared<LifeCycleNodeSub>("nodo_D");

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    nodo_C->do_work();

    rclcpp::spin_some(nodo_C->get_node_base_interface());
    rclcpp::spin_some(nodo_D->get_node_base_interface());
    rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
