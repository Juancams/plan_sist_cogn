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

#include "scan_fake/ScanFakePublisher.hpp"
#include <memory>
#include <algorithm>
#include <iostream>
#include <string>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace scan_fake 
{
using namespace std::chrono_literals;

ScanFakePublisher::ScanFakePublisher(const std::string & name)
: Node (name)
{
    // Max angle, min angle, increment of the scan and resize ranges
    scan.angle_min = angle_min;
    scan.angle_max = angle_max;
    scan.angle_increment = (angle_max - angle_min) / n_readings;
    scan.ranges.resize(n_readings);

    // Set min and max values of the scan
    scan.range_min = min_value;
    scan.range_max = max_value;

    // Publisher and timer
    pub = create_publisher<sensor_msgs::msg::LaserScan>(
      "scan_fake", rclcpp::SensorDataQoS());
    timer = create_wall_timer(
      1s, std::bind(&ScanFakePublisher::publish_scan_fake, this));
}

void ScanFakePublisher::generate_scan_fake()
{
    // Normal distribution and generator seed
    generator.seed(time(NULL));
    std::normal_distribution<float> distribution(mean, standard_deviation);

    // Generate 100 random readings between the minimum and maximum values
    // and include them in the scan
    for (int n = 0; n < n_readings; n++) {
      float value = std::max(scan.range_min, std::min(scan.range_max, distribution(generator)));
      scan.ranges[n] = value;
    }
}

void ScanFakePublisher::publish_scan_fake()
{
    // Generate scan
    generate_scan_fake();

    // Publish scan
    RCLCPP_INFO(get_logger(), "Publishing Scan");
    pub->publish(scan);
}

} // namesàce scan_fake