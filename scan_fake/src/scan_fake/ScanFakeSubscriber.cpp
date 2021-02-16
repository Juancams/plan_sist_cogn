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

#include "scan_fake/ScanFakeSubscriber.hpp"
#include <memory>
#include <algorithm>
#include <iostream>
#include <string>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace scan_fake 
{
using std::placeholders::_1;

ScanFakeSubscriber::ScanFakeSubscriber(const std::string & name)
: Node (name)
{
    // Subscriber
    sub = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_fake", rclcpp::SensorDataQoS(),
      std::bind(&ScanFakeSubscriber::scan_callback, this, _1));
}

void ScanFakeSubscriber::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    // Initialize min and max values
    min_value = scan->range_max;
    max_value = scan->range_min;
    int n_readings = (scan->angle_max - scan->angle_min) / scan->angle_increment;
    float value_sum = 0.0;

    // Loop for checking every value in the scan ranges
    for (int n = 0; n < n_readings; n++) {
      if (scan->ranges[n] > max_value) {  // Compare if it is the maximum value
        max_value = scan->ranges[n];
      } else if (scan->ranges[n] < min_value) {  // Compare if it is the minimum value
        min_value = scan->ranges[n];
      }
      // Sum of all values
      value_sum += scan->ranges[n];
    }
    // Result of the mean
    average = value_sum / n_readings;

    // Show minimum, maximum and mean
    RCLCPP_INFO(
      get_logger(),
      "The min value is: [%f] | The max value is: [%f] | The mean is: [%f] ",
      min_value, max_value, average);
}

} // namespace scan_fake
