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
 * @author Tanuj Thakkar (tanuj@phantomauto.com)
 * @version 0.1
 * @date 2022-11-06
 *
 * @brief ROS2 Subscriber
 *
 */

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
  MinimalSubscriber() : Node("minimal_subscriber") {
    if (rcutils_logging_set_logger_level(
            this->get_logger().get_name(),
            RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG) ==
        RCUTILS_RET_OK) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Set logger level DEBUG success.");
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Set logger level DEBUG fails.");
    }

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "talker", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  /**
   * @brief Callback to get count
   * 
   * @param msg 
   */
  void topic_callback(const std_msgs::msg::String &msg) const {
    this->logger(msg);
  }

  /**
   * @brief Switch to display received count logging levels
   * 
   * @param msg 
   */
  void logger(const std_msgs::msg::String &msg) const {
    int count = stoi(msg.data);
    switch (count % 5) {
      case 0:
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Count Received: " << msg.data);
        break;
      case 1:
        RCLCPP_INFO_STREAM(this->get_logger(), "Count Received: " << msg.data);
        break;
      case 2:
        RCLCPP_WARN_STREAM(this->get_logger(), "Count Received: " << msg.data);
        break;
      case 3:
        RCLCPP_ERROR_STREAM(this->get_logger(), "Count Received: " << msg.data);
        break;
      case 4:
        RCLCPP_FATAL_STREAM(this->get_logger(), "Count Received: " << msg.data);
        break;
      default:
        break;

        return;
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                     "The logger level will depends on the "
                     "remainder of received count divided by 5.");

  RCLCPP_INFO_STREAM(
      rclcpp::get_logger("rclcpp"),
      "\nThe logger level of each remainder value:"
      "\n 0) Debug \n 1) INFO \n 2) WARN \n 3) ERROR \n 4) FATAL \n");

  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
