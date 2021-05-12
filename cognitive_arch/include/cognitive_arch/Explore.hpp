// Copyright 2021 The Rebooters
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

#ifndef COGNITIVE_ARCH__EXPLORE_HPP_
#define COGNITIVE_ARCH__EXPLORE_HPP_

#include <memory>
#include <string>
#include <algorithm>
#include <iostream>
#include <vector>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_listener.h"
#include "pcl/impl/point_types.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "blackboard/BlackBoardNode.hpp"
#include "blackboard/BlackBoardClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

namespace cognitive_arch
{

class Explore : public plansys2::ActionExecutorClient
{
public:
  Explore(const std::string & name, const std::chrono::nanoseconds & rate);

protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

private:
  void do_work();
  void cameraCB(const sensor_msgs::msg::Image::SharedPtr image_in);
  void depthCB(const sensor_msgs::msg::Image::SharedPtr image_in);
  void cloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_in);

  float progress_;
  int row_;
  int col_;
  int distance_;
  bool first_;
  bool found_;
  bool found_pc;
  bool pose_saved_;
  int counter_;
  geometry_msgs::msg::PoseStamped ob_pose;
  geometry_msgs::msg::PoseStamped obj_pc;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tfListener;
  std::string object;
  int counters_[3];
  int index_;

  blackboard::Entry<std::string>::Ptr room_color_;
  blackboard::Entry<std::string>::Ptr room_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Node::SharedPtr node_;
  blackboard::BlackBoardClient::Ptr client_;

  cv::Scalar min_;
  cv::Scalar max_;
};

}  // namespace cognitive_arch

#endif  // COGNITIVE_ARCH__EXPLORE_HPP_
