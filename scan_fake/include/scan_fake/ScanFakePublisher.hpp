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

#ifndef SCAN_FAKE__SCANFAKEPUBLISHER_HPP_
#define SCAN_FAKE__SCANFAKEPUBLISHER_HPP_

#include <memory>
#include <algorithm>
#include <iostream>
#include <string>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace scan_fake
{
class ScanFakePublisher : public rclcpp::Node
{
public:
    explicit ScanFakePublisher(const std::string & name);

private:
    void generate_scan_fake();
    void publish_scan_fake();

private:
    // Timer and publisher
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;

    // Scan msg and parameters
    sensor_msgs::msg::LaserScan scan;
    const float angle_min = 0.0;
    const float angle_max = 2.0 * M_PI;
    const int n_readings = 100;
    const float max_value = 8.0;
    const float min_value = 0.0;

    // Random number generator
    std::default_random_engine generator;

    // Normal distribution parameters
    const float mean = 4.0;
    const float standard_deviation = 1.0;
};
} // namespace scan_fake

#endif // SCAN_FAKE__SCANFAKEPUBLISHER_HPP_