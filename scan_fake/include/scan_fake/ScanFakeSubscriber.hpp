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

#ifndef SCAN_FAKE__SCANFAKESUBSCRIBER_HPP_
#define SCAN_FAKE__SCANFAKESUBSCRIBER_HPP_

#include <memory>
#include <algorithm>
#include <iostream>
#include <string>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace scan_fake
{
class ScanFakeSubscriber : public rclcpp::Node
{
public:
    explicit ScanFakeSubscriber(const std::string & name);

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
    float min_value = 0.0;
    float max_value = 0.0;
    float average = 0.0;
};
} // namespace scan_fake

#endif // SCAN_FAKE__SCANFAKESUBSCRIBER_HPP_