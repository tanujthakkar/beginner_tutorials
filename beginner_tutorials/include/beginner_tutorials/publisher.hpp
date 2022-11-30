/******************************************************************************
 * MIT License
Copyright (c) 2022 Tanuj Thakkar

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
* *******************************************************************************
*/

/**
 * @copyright Copyright (c) 2022
 * @file publisher.cpp
 * @author Tanuj Thakkar (tanuj@umd.edu)
 * @version 0.1
 *
 * @brief ROS2 Publisher header file
 *
 */

#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <beginner_tutorials/srv/count.hpp>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;
using Count = beginner_tutorials::srv::Count;

/**
 * @Brief Class to represent publisher and service
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   *
   */
  MinimalPublisher();

 private:
  /**
   * @brief Callback from timer
   *
   */
  void timer_callback();

  /**
   * @brief Switch to reproduce logging levels
   *
   * @param msg
   */
  void logger(const std_msgs::msg::String &msg);

  /**
   * @brief Get the count callback object
   *
   * @param request
   * @param response
   */
  void get_count_callback(const std::shared_ptr<Count::Request> request,
                          std::shared_ptr<Count::Response> response);

  /**
   * @Brief  Callback for the tf broadcaster timer
   */
  void tf_broadcast_timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<Count>::SharedPtr get_count_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_broadcaster_timer_;
  int count_;
};
