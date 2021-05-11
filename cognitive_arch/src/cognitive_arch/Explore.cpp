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

#include <stdio.h>
#include <string>
#include <vector>
#include "cognitive_arch/Explore.hpp"


namespace cognitive_arch
{
Explore::Explore(const std::string & name, const std::chrono::nanoseconds & rate)
: plansys2::ActionExecutorClient(name, rate)
{
  memset(&counters_, 0, sizeof(counters_));
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Explore::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  progress_ = 0.0;
  first_ = true;
  found_ = false;
  pose_saved_ = false;
  row_ = 0;
  col_ = 0;
  distance_ = 0;

  client_ = blackboard::BlackBoardClient::make_shared();

  room_ = blackboard::as<std::string>(client_->get_entry("r2d2", "place"));
  room_color_ = blackboard::as<std::string>(client_->get_entry(room_->data_, "color"));

  if (room_color_ == nullptr) {
    RCLCPP_ERROR(
      get_logger(), "No color defined for %s",
      room_->data_.c_str());
  }

  auto node = rclcpp::Node::make_shared("move_param_node");

  node->declare_parameter("colors_values");

  if (node->has_parameter("colors_values")) {
    node->declare_parameter("colors_values." + room_color_->data_);
    std::vector<double> values;
    if (node->get_parameter_or("colors_values." + room_color_->data_, values, {})) {
      min_ = cv::Scalar(values[0], values[1], values[2]);
      max_ = cv::Scalar(values[3], values[4], values[5]);
    }
  }

  node->declare_parameter("objects");
  if (node->has_parameter("objects")) {
    node->declare_parameter("objects." + room_color_->data_);
    if (node->get_parameter_or("objects." + room_color_->data_, object, {})) {
      if (object.compare("cereals") == 0) {
        index_ = 0;
      } else if (object.compare("ball") == 0) {
        index_ = 1;
      } else if (object.compare("apple") == 0) {
        index_ = 2;
      }
    }
  }

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  cmd_vel_pub_->on_activate();

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/kinect_color/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&Explore::cameraCB, this, std::placeholders::_1));

  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/kinect_range/image_depth", rclcpp::SensorDataQoS(),
    std::bind(&Explore::depthCB, this, std::placeholders::_1));

  pos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 1, std::bind(&Explore::positionCB, this, std::placeholders::_1));

  return ActionExecutorClient::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Explore::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  cmd_vel_pub_->on_deactivate();
  image_sub_ = nullptr;
  depth_sub_ = nullptr;

  return ActionExecutorClient::on_deactivate(previous_state);
}

void Explore::do_work()
{
  if (progress_ < 5.0 && !found_) {
    progress_ += 0.1;

    send_feedback(progress_, "Patrol running");

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 1;

    cmd_vel_pub_->publish(cmd);
  } else {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    cmd_vel_pub_->publish(cmd);

    if (found_) {
      std::stringstream obj;

      obj << object << counters_[index_]++;

      auto entry = blackboard::Entry<geometry_msgs::msg::PoseStamped>::make_shared(ob_pose);
      auto entry_loc = blackboard::Entry<std::string>::make_shared(room_->data_.c_str());

      client_->add_entry(obj.str(), "location", entry->to_base());
      client_->add_entry(obj.str(), "at", entry_loc->to_base());
  
    }
    finish(true, 1.0, "Patrol completed");
  }
}


void Explore::cameraCB(const sensor_msgs::msg::Image::SharedPtr image_in)
{
  cv_bridge::CvImagePtr cv_ptr, cv_imageout;
  cv_ptr = cv_bridge::toCvCopy(image_in, sensor_msgs::image_encodings::BGR8);
  cv::Mat hsv, binary;

  cv::cvtColor(cv_ptr->image, hsv, CV_BGR2HSV);
  cv::inRange(hsv, min_, max_, binary);

  cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(binary, binary, cv::MORPH_OPEN, element);
  cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, element);

  int counter = 0;

  for (int i = 0; i < binary.rows; i++) {
    for (int j = 0; j < binary.cols; j++) {
      if (binary.at<uchar>(i, j) == 255) {
        row_ += i;
        col_ += j;
        counter++;
      }
    }
  }

  if (counter != 0) {
    row_ /= counter;
    col_ /= counter;
  }

  if (counter > 50 && first_ && 280 < col_ && col_ < 320) {
    RCLCPP_INFO(get_logger(), "ENCONTRADO");
    found_ = true;
    first_ = false;
  }

  circle(cv_ptr->image, cv::Point(col_, row_), 4, cv::Scalar(255, 0, 0), -1);
  cv::imshow("Image", cv_ptr->image);
  cv::waitKey(3);
}

void Explore::depthCB(const sensor_msgs::msg::Image::SharedPtr image_in)
{
  if (found_) {
    distance_ = image_in->data[image_in->step * row_ + col_];
    RCLCPP_INFO(get_logger(), "Distance %d", distance_);
  }
}

void Explore::positionCB(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
  if (found_ && !pose_saved_) {
    ob_pose.pose = odometry->pose.pose;
    pose_saved_ = true;
  }
}

}  // namespace cognitive_arch
