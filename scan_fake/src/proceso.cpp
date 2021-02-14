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
#include <iostream>
#include <string>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class ScanFakePublisher : public rclcpp::Node
{
public:
  explicit ScanFakePublisher(const std::string & name)
  : Node(name)
  {
    // Max angle, min angle, increment of the scan and resize ranges
    scan.angle_min = angle_min;
    scan.angle_max = angle_max;
    scan.angle_increment = (angle_max - angle_min) * (M_PI / 180);
    scan.ranges.resize(n_readings);

    // Publisher and timer
    pub = create_publisher<sensor_msgs::msg::LaserScan>(
      "scan_fake", rclcpp::QoS(10).best_effort());
    timer = create_wall_timer(
      1s, std::bind(&ScanFakePublisher::publish_scan_fake, this));
  }

private:
  void generate_scan_fake()
  {
    // Normal distribution and random number generator
    std::default_random_engine generator;
    generator.seed(time(NULL));
    std::normal_distribution<float> distribution(mean, standard_deviation);

    float max_value = mean;
    float min_value = mean;

    // Generate 100 random readings avoiding negative values
    int reads = 0;
    while (reads < n_readings) {
      // Random reading
      float value = distribution(generator);
      if (value > 0) {
        // Include reading in the scan
        scan.ranges[reads] = value;
        reads++;

        // Update min and max value
        if (value > max_value) {
          max_value = value;
        }

        if (value < min_value) {
          min_value = value;
        }
      }
    }

    // Set min and max values of the scan
    scan.range_min = min_value;
    scan.range_max = max_value;
  }

  void publish_scan_fake()
  {
    // Generate scan
    generate_scan_fake();

    // Publish scan
    pub->publish(scan);
  }

private:
  // Timer and publisher
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;

  // Scan msg and parameters
  sensor_msgs::msg::LaserScan scan;
  const int angle_min = 0;
  const int angle_max = 2 * M_PI;
  const int n_readings = 100;

  // Normal distribution parameters
  const float mean = 4.0;
  const float standard_deviation = 1.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_A = std::make_shared<ScanFakePublisher>("nodo_A");

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node_A);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
