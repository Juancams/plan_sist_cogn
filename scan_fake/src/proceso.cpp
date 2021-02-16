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

#include <memory>
#include "scan_fake/ScanFakePublisher.hpp"
#include "scan_fake/ScanFakeSubscriber.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_A = std::make_shared<scan_fake::ScanFakePublisher>("node_A");
  auto node_B = std::make_shared<scan_fake::ScanFakeSubscriber>("node_B");

  rclcpp::executors::MultiThreadedExecutor exec;

  exec.add_node(node_A);
  exec.add_node(node_B);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
