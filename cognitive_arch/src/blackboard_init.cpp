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
// limitations under the License.#include <memory>

#include <string>
#include <vector>

#include "blackboard/BlackBoardNode.hpp"
#include "blackboard/BlackBoardClient.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap/octomap.h"
#include "octomap/OcTree.h"
#include "octomap/OcTreeKey.h"
#include "octomap/ColorOcTree.h"
#include "octomap/octomap_utils.h"
#include "octomap_msgs/conversions.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto client = blackboard::BlackBoardClient::make_shared();

  auto node = rclcpp::Node::make_shared("move_param_node");
  auto octomap_pub_ = node->create_publisher<octomap_msgs::msg::Octomap>(
    "/octomap", rclcpp::QoS(1).transient_local());

  node->declare_parameter("waypoints");
  node->declare_parameter("waypoint_coords");
  node->declare_parameter("rooms_colors");
  node->declare_parameter("rooms_octomaps");
  node->declare_parameter("voxel_resolution", 0.1);

  if (node->has_parameter("waypoints")) {
    std::vector<std::string> wp_names;

    node->get_parameter_or("waypoints", wp_names, {});

    for (auto & wp : wp_names) {
      node->declare_parameter("waypoint_coords." + wp);

      std::vector<double> coords;
      if (node->get_parameter_or("waypoint_coords." + wp, coords, {})) {
        geometry_msgs::msg::PoseStamped wp_pose;
        wp_pose.header.frame_id = "/map";
        // wp_pose.header.stamp = now();
        wp_pose.pose.position.x = coords[0];
        wp_pose.pose.position.y = coords[1];
        wp_pose.pose.position.z = coords[2];
        wp_pose.pose.orientation.x = 0.0;
        wp_pose.pose.orientation.y = 0.0;
        wp_pose.pose.orientation.z = 0.0;
        wp_pose.pose.orientation.w = 1.0;

        auto entry = blackboard::Entry<geometry_msgs::msg::PoseStamped>::make_shared(wp_pose);
        client->add_entry(wp, "location", entry->to_base());
      }

      node->declare_parameter("rooms_colors." + wp);

      std::string color;
      if (node->get_parameter_or("rooms_colors." + wp, color, {})) {
        auto entry_color = blackboard::Entry<std::string>::make_shared(color);
        client->add_entry(wp, "color", entry_color->to_base());
      }

      // node->declare_parameter("rooms_octomaps." + wp);

      // std::vector<double> room_octomap;
      // if (node->get_parameter_or("rooms_octomaps." + wp, room_octomap, {})) {
      //   octomap::ColorOcTree * octomap_;

      //   double voxel_res_;
      //   node->get_parameter("voxel_resolution", voxel_res_);

      //   double probHit, probMiss, thresMin, thresMax;
      //   probHit = 0.7;
      //   probMiss = 0.4;
      //   thresMin = 0.12;
      //   thresMax = 0.97;

      //   octomap_ = new octomap::ColorOcTree(voxel_res_);
      //   octomap_->setProbHit(probHit);
      //   octomap_->setProbMiss(probMiss);
      //   octomap_->setClampingThresMin(thresMin);
      //   octomap_->setClampingThresMax(thresMax);

      //   for (int x = room_octomap[0]; x < room_octomap[3]; x++) {
      //     for (int y = room_octomap[1]; y < room_octomap[4]; y++) {
      //       for (int z = room_octomap[2]; z < room_octomap[5]; z++) {
      //         octomap::point3d endpoint((float) x, (float)y, (float)z);
      //         octomap_->updateNode(endpoint, true);
      //       }
      //     }
      //   }

      // octomap_msgs::msg::Octomap octomap_msg;
      // octomap_msg.header.frame_id = "map";
      // // octomap_msg.header.stamp = now;
      // octomap_msg.binary = false;
      // octomap_msg.resolution = voxel_res_;

      // octomap_msgs::fullMapToMsg(*octomap_, octomap_msg);
      // auto entry_octomap =
      //   blackboard::Entry<octomap_msgs::msg::Octomap>::make_shared(octomap_msg);
      // client->add_entry(wp, "octomap", entry_octomap->to_base());
      // octomap_pub_->publish(octomap_msg);
      // rclcpp::spin_some(node);
      // }
    }
  }

  return 0;
}
