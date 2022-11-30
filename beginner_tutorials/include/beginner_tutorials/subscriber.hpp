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
 * @file subscriber.cpp
 * @author Tanuj Thakkar (tanuj@umd.edu)
 * @version 0.1
 *
 * @brief ROS2 Subscriber header file
 *
 */

#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

/**
 * @brief Class to represent subscriber and service client
 *
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Subscriber object
   *
   */
  MinimalSubscriber();

 private:
  /**
   * @brief Callback to get count
   *
   * @param msg
   */
  void topic_callback(const std_msgs::msg::String &msg) const;

  /**
   * @brief Switch to display received count logging levels
   *
   * @param msg
   */
  void logger(const std_msgs::msg::String &msg) const;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
