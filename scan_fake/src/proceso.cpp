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
#include <algorithm>
#include <iostream>
#include <string>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ScanFakePublisher : public rclcpp::Node
{
public:
  explicit ScanFakePublisher(const std::string & name)
  : Node(name)
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

private:
  void generate_scan_fake()
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

  void publish_scan_fake()
  {
    // Generate scan
    generate_scan_fake();

    // Publish scan
    RCLCPP_INFO(get_logger(), "Publishing Scan");
    pub->publish(scan);
  }

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

class ScanFakeSubscriber : public rclcpp::Node
{
public:
  explicit ScanFakeSubscriber(const std::string & name)
  : Node(name)
  {
    // Subscriber
    sub = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_fake", rclcpp::SensorDataQoS(),
      std::bind(&ScanFakeSubscriber::scan_callback, this, _1));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    // Initialize min and max values 
    min_value = scan->range_max;
    max_value = scan->range_min;
    
    float value_sum = 0.0;

    // Loop for checking every value in the scan ranges
    for(int n = 0; n < n_readings; n++)
    {
      // Compare if it is the maximum value
      if(scan->ranges[n] > max_value)
      {
        max_value = scan->ranges[n];
      }
      // Compare if it is the minimum value
      else if(scan->ranges[n] < min_value)
      {
        min_value = scan->ranges[n];
      }
      // Sum of all values
      value_sum += scan->ranges[n];
    }
    // Result of the mean
    mean = value_sum / n_readings;

    // Show minimum, maximum and mean
    RCLCPP_INFO(get_logger(), 
    "The min value is: [%f] | The max value is: [%f] | The mean is: [%f] ", 
    min_value, max_value, mean);
  }
private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
  float min_value = 0.0;
  float max_value = 0.0;
  float mean = 0.0;
  const int n_readings = 100;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_A = std::make_shared<ScanFakePublisher>("node_A");
  auto node_B = std::make_shared<ScanFakeSubscriber>("node_B");

  rclcpp::executors::MultiThreadedExecutor exec;

  exec.add_node(node_A);
  exec.add_node(node_B);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
