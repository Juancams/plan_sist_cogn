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
#include <cmath>
#include "tf2/LinearMath/Quaternion.h"
#include "cognitive_arch/Explore.hpp"


namespace cognitive_arch
{
Explore::Explore(const std::string & name, const std::chrono::nanoseconds & rate)
: plansys2::ActionExecutorClient(name, rate),  buffer_(this->get_clock()), tfListener(buffer_), found_pc(false)
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

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/depth_registered/points", rclcpp::SensorDataQoS(), std::bind(&Explore::cloudCB, this, std::placeholders::_1));
 
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
    found_pc = false;
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

void Explore::cloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_in)
{
  if (!found_pc && found_) { 
    pcl::PCLPointCloud2 pc2;
    pcl_conversions::toPCL(*cloud_in, pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pc2, *point_cloud);
    //while(!buffer_.canTransform("map", point_cloud->header.frame_id, pcl_conversions::fromPCL(point_cloud->header.stamp))){
    for (int i = 0; i < 20; i++) {
      try
      {
        
        pcl_ros::transformPointCloud("map", *point_cloud, *map_point_cloud, buffer_);
      }
      catch(tf2::TransformException & ex)
      {
        RCLCPP_ERROR_STREAM(get_logger(),"Transform error of sensor data: quitting callback");
        return;
      }
    }
    if(map_point_cloud->height != 1){
      pcl::PointXYZ point = map_point_cloud->at(row_,col_);
      
      obj_pc.pose.position.x = point.x;
      obj_pc.pose.position.y = point.y;
      obj_pc.pose.position.z = point.z;

      std::stringstream obj;
      obj << object << counters_[index_]++;

      auto entry_pc = blackboard::Entry<geometry_msgs::msg::PoseStamped>::make_shared(obj_pc);
      auto loc_room = blackboard::as<geometry_msgs::msg::PoseStamped>(client_->get_entry(room_->data_, "location"));

      ob_pose.pose.position.x = loc_room->data_.pose.position.x;
      ob_pose.pose.position.y = loc_room->data_.pose.position.y;
      ob_pose.pose.position.z = loc_room->data_.pose.position.z;

      double angle = atan((ob_pose.pose.position.x - obj_pc.pose.position.x) / (ob_pose.pose.position.y - obj_pc.pose.position.y));
      RCLCPP_INFO(get_logger(), "Angle %f", angle);
      RCLCPP_INFO(get_logger(), "pos x %f", ob_pose.pose.position.x);
      RCLCPP_INFO(get_logger(), "pos x' %f", obj_pc.pose.position.x);
      RCLCPP_INFO(get_logger(), "pos y %f", ob_pose.pose.position.y);
      RCLCPP_INFO(get_logger(), "pos y' %f", obj_pc.pose.position.y);
      if(ob_pose.pose.position.x > obj_pc.pose.position.x) {
        if(ob_pose.pose.position.y > obj_pc.pose.position.y) {
          angle = angle + M_PI;
          RCLCPP_INFO(get_logger(), "Angle2 %f", angle);
        }
        else {
          angle = angle + (3*M_PI)/2;
          RCLCPP_INFO(get_logger(), "Angle3 %f", angle);
        }
      }
      else {
        if(ob_pose.pose.position.y > obj_pc.pose.position.y) {
          angle = angle + M_PI/2;
          RCLCPP_INFO(get_logger(), "Angle4 %f", angle);
        }
      }

      tf2::Quaternion q;
      q.setRPY(0,0,angle);
      q=q.normalize();

      

      ob_pose.pose.orientation.x = q[0];
      ob_pose.pose.orientation.y = q[1];
      ob_pose.pose.orientation.z = q[2];
      ob_pose.pose.orientation.w = q[3];
      
      auto entry = blackboard::Entry<geometry_msgs::msg::PoseStamped>::make_shared(ob_pose);
      auto entry_loc = blackboard::Entry<std::string>::make_shared(room_->data_.c_str());

      client_->add_entry(obj.str(), "location", entry->to_base());
      client_->add_entry(obj.str(), "at", entry_loc->to_base());
      client_->add_entry(obj.str(), "point", entry_pc->to_base());

      RCLCPP_INFO(get_logger(), "(%f, %f, %f)",point.x,point.y,point.z);
      found_pc = true;
    }
    
  }
}

}  // namespace cognitive_arch
