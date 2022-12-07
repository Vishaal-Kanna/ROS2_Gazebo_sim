// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using SCAN = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

typedef enum {
  FORWARD = 0,
  STOP,
  TURN,
} StateType;

/**
 * @brief Class to implement walker algorithm for the waffle with depth camera
 *
 */
class turtlebot_sim : public rclcpp::Node {
 public:
  turtlebot_sim();

 private:
  /**
   * @brief Callback function to determine the state of the system and decide on
   * the new input.
   *
   */
  void timer_callback();

  /**
   * @brief Callback function to receive the incoming laser scan data
   *
   */
  void scan_callback(const SCAN&);

  /**
   * @brief Method to find obstacle distance is within range.
   *
   * @return true If obstacle is detected
   * @return false If obstacle is not detected
   */
  bool detect_obstacle();

  rclcpp::Publisher<TWIST>::SharedPtr publisher_;
  rclcpp::Subscription<SCAN>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  SCAN scan_;          // To store incoming laser scan data
  float left_dist_;    // Distance measured by left ray of Laser
  float center_dist_;  // Distance measured by center ray of Laser
  float right_dist_;   // Distance measured by right ray of Laser
  StateType state_;
};