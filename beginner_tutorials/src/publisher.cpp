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
 * @brief ROS2 Publisher source file
 *
 */

#include <beginner_tutorials/publisher.hpp>

/**
 * @brief Construct a new Minimal Publisher object
 *
 */
MinimalPublisher::MinimalPublisher() : Node("minimal_publisher"), count_(0) {
  if (rcutils_logging_set_logger_level(
          this->get_logger().get_name(),
          RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG) == RCUTILS_RET_OK) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Set logger level DEBUG success.");
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Set logger level DEBUG fails.");
  }

  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description =
      "\nThis parameter is the count value passed between "
      "the minimal publisher and the minimal subscriber."
      "\n\n It also determines the logger level of both nodes.";

  this->declare_parameter("count", count_, param_desc);

  count_ = this->get_parameter("count").get_parameter_value().get<int>();
  RCLCPP_INFO_STREAM(this->get_logger(), "Starting counter from: " << count_);

  publisher_ = this->create_publisher<std_msgs::msg::String>("talker", 10);
  timer_ = this->create_wall_timer(
      1s, std::bind(&MinimalPublisher::timer_callback, this));

  // Create a service for modifying count
  std::string get_count_service_name =
      "/" + std::string(this->get_name()) + "/" + "Count";
  get_count_service_ = this->create_service<Count>(
      get_count_service_name,
      std::bind(&MinimalPublisher::get_count_callback, this, _1, _2));

  // Creating a tf broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_broadcaster_timer_ = this->create_wall_timer(
      300ms, std::bind(&MinimalPublisher::tf_broadcast_timer_callback, this));
}

/**
 * @brief Get the tf world talk object
 *
 * @return geometry_msgs::msg::Transform
 */
geometry_msgs::msg::TransformStamped MinimalPublisher::get_tf_world_talk()
    const {
  return tf_world_talk_;
}

/**
 * @brief Set the tf world talk object
 *
 * @param tf_world_talk
 */
void MinimalPublisher::set_tf_world_talk(
    const geometry_msgs::msg::TransformStamped &tf_world_talk) {
  tf_world_talk_ = tf_world_talk;
}

/**
 * @brief Callback from timer
 *
 */
void MinimalPublisher::timer_callback() {
  auto message = std_msgs::msg::String();
  count_ = this->get_parameter("count").get_parameter_value().get<int>();
  message.data = std::to_string(count_);
  this->logger(message);
  publisher_->publish(message);
  count_++;
  this->set_parameter(rclcpp::Parameter("count", count_));
}

/**
 * @brief Switch to reproduce logging levels
 *
 * @param msg
 */
void MinimalPublisher::logger(const std_msgs::msg::String &msg) {
  int count = stoi(msg.data);
  switch (count % 5) {
    case 0:
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Count: " << msg.data);
      break;
    case 1:
      RCLCPP_INFO_STREAM(this->get_logger(), "Count: " << msg.data);
      break;
    case 2:
      RCLCPP_WARN_STREAM(this->get_logger(), "Count: " << msg.data);
      break;
    case 3:
      RCLCPP_ERROR_STREAM(this->get_logger(), "Count: " << msg.data);
      break;
    case 4:
      RCLCPP_FATAL_STREAM(this->get_logger(), "Count: " << msg.data);
      break;
    default:
      break;
  }

  return;
}

/**
 * @brief Get the count callback object
 *
 * @param request
 * @param response
 */
void MinimalPublisher::get_count_callback(
    const std::shared_ptr<Count::Request> request,
    std::shared_ptr<Count::Response> response) {
  (void)request;
  response->count = count_;
}

void MinimalPublisher::tf_broadcast_timer_callback() {
  tf2::Quaternion quat;
  quat.setRPY(0, 0.735, -0.735);

  geometry_msgs::msg::TransformStamped transform_stamped;

  transform_stamped.header.stamp = this->get_clock()->now();
  transform_stamped.header.frame_id = "world";
  transform_stamped.child_frame_id = "talk";
  transform_stamped.transform.translation.x = 5.0;
  transform_stamped.transform.translation.y = 5.0;
  transform_stamped.transform.translation.z = 5.0;
  transform_stamped.transform.rotation = tf2::toMsg(quat);
  set_tf_world_talk(transform_stamped);

  tf_broadcaster_->sendTransform(transform_stamped);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                     "The logger level will depends on the "
                     "remainder of current count divided by 5.");

  RCLCPP_INFO_STREAM(
      rclcpp::get_logger("rclcpp"),
      "\nThe logger level of each remainder value:"
      "\n 0) Debug \n 1) INFO \n 2) WARN \n 3) ERROR \n 4) FATAL \n");

  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
