/**
 * @copyright Copyright (c) 2022
 * @file publisher.cpp
 * @author Tanuj Thakkar (tanuj@phantomauto.com)
 * @version 0.1
 * @date 2022-11-06
 *
 * @brief ROS2 Publisher
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>
#include "beginner_tutorials/srv/count.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using Count = beginner_tutorials::srv::Count;

/**
 * @Brief  A publisher node
 */
class MinimalPublisher : public rclcpp::Node {
public:
  /**
   * @Brief The constructor
   */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    if (rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG) == RCUTILS_RET_OK) {
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
    RCLCPP_INFO_STREAM(this->get_logger(), "Count starts from " << count_);

    publisher_ = this->create_publisher<std_msgs::msg::String>("talker", 10);
    timer_ = this->create_wall_timer(
        1s, std::bind(&MinimalPublisher::timer_callback, this));

    // Create a service for modifying count
    std::string get_count_service_name =
        "/" + std::string(this->get_name()) + "/" + "Count";
    get_count_service_ = this->create_service<Count>(
        get_count_service_name,
        std::bind(&MinimalPublisher::get_count_callback, this, _1, _2));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    count_ = this->get_parameter("count").get_parameter_value().get<int>();
    message.data = std::to_string(count_);
    this->logger(message);
    publisher_->publish(message);
    count_++;
    this->set_parameter(rclcpp::Parameter("count", count_));
  }

  void logger(const std_msgs::msg::String &msg) {
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

  void get_count_callback(const std::shared_ptr<Count::Request> request,
                          std::shared_ptr<Count::Response> response) {
    (void)request;
    response->count = count_;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<Count>::SharedPtr get_count_service_;
  int count_;
};

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
