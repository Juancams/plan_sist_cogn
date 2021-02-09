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

#include <chrono>
#include <memory>

#include <cstdlib>
#include <ctime>

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

class LifeCycleNodePublisher : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifeCycleNodePublisher(const std::string & name, const std::chrono::nanoseconds & rate)
  : rclcpp_lifecycle::LifecycleNode(name)
  {
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/topic_CD", 10);

    timer_ = create_wall_timer(
      rate, std::bind(&LifeCycleNodePublisher::do_work, this));

    seed = time(NULL);
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

    pub_->on_activate();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...",
      get_name(), state.label().c_str());

    pub_->on_deactivate();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...",
      get_name(), state.label().c_str());

    pub_.reset();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...",
      get_name(), state.label().c_str());

    pub_.reset();

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
    if (pub_->is_activated()) {
      geometry_msgs::msg::PoseStamped msg;
      // numeros aleatorios entre 0-100
      msg.pose.position.x = static_cast<float>(rand_r(&seed) % (100));
      msg.pose.position.y = static_cast<float>(rand_r(&seed) % (100));
      msg.pose.position.z = static_cast<float>(rand_r(&seed) % (100));

      RCLCPP_INFO(get_logger(), "Publishing");

      pub_->publish(msg);
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  unsigned int seed;
};

class LifeCycleNodeSubscriber : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifeCycleNodeSubscriber(const std::string & name)
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

    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/topic_CD", 10, std::bind(&LifeCycleNodeSubscriber::callback, this, _1));

    service_ = create_service<practica_msgs::srv::GetPoseStamped>(
      "get_pose_stamped", std::bind(&LifeCycleNodeSubscriber::get_pose_stampedCB,
      this, _1, _2));

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...",
      get_name(), state.label().c_str());

    sub_ = nullptr;

    service_ = nullptr;

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

  void get_pose_stampedCB(
    const std::shared_ptr<practica_msgs::srv::GetPoseStamped::Request>
    request, std::shared_ptr<practica_msgs::srv::GetPoseStamped::Response> response)
  {
    response->x = p_stamped.pose.position.x;
    response->y = p_stamped.pose.position.y;
    response->z = p_stamped.pose.position.z;

    RCLCPP_INFO(get_logger(), "Incoming request [%s]", request->request_sentence.c_str());
    RCLCPP_INFO(get_logger(), "sending back response: x: [%lf] y: [%lf] z: [%lf]",
      response->x, response->y, response->z);
  }

private:
  void callback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: ");
    RCLCPP_INFO(this->get_logger(), "X:  [%lf]",
      msg->pose.position.x);
    p_stamped.pose.position.x = msg->pose.position.x;
    RCLCPP_INFO(this->get_logger(), "Y:  [%lf]",
      msg->pose.position.y);
    p_stamped.pose.position.y = msg->pose.position.y;
    RCLCPP_INFO(this->get_logger(), "Z:  [%lf]",
      msg->pose.position.z);
    p_stamped.pose.position.z = msg->pose.position.z;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

  rclcpp::Service<practica_msgs::srv::GetPoseStamped>::SharedPtr service_;

  geometry_msgs::msg::PoseStamped p_stamped;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto nodo_C = std::make_shared<LifeCycleNodePublisher>("nodo_C", 100ms);  // 10Hz
  auto nodo_D = std::make_shared<LifeCycleNodeSubscriber>("nodo_D");

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(nodo_C->get_node_base_interface());
  executor.add_node(nodo_D->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
