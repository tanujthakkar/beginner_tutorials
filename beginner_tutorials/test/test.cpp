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
 * @file test.cpp
 * @author Tanuj Thakkar (tanuj@umd.edu)
 * @version 0.1
 *
 * @brief Testing Talker Node
 *
 */

#include <gtest/gtest.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <beginner_tutorials/publisher.hpp>
#include <cstdint>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

TEST(DummyTests, Dummy1) { EXPECT_TRUE(true); }

class TestPub : public ::testing::Test {
 public:
  TestPub() {}

  void SetUp() override {
    rclcpp::init(0, nullptr);
    minimal_pub_ = std::make_shared<MinimalPublisher>();
    clock_ = std::make_unique<rclcpp::Clock>(RCL_ROS_TIME);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void TearDown() override { rclcpp::shutdown(); }

 protected:
  std::shared_ptr<MinimalPublisher> minimal_pub_;
  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<std::thread> pub_thread_;
};

TEST_F(TestPub, TestTF) {
  auto start = clock_->now();
  double duration_sec = 0;
  while (duration_sec < 3) {
    rclcpp::spin_some(minimal_pub_);
    duration_sec = (clock_->now() - start).seconds();
  }

  geometry_msgs::msg::TransformStamped t;

  // lookupTransform
  try {
    t = tf_buffer_->lookupTransform("world", "talk", tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "Could not find transform from world to talk");
    FAIL();
  }

  // Received transform
  auto rx = t.transform.translation.x;
  auto ry = t.transform.translation.y;
  auto rz = t.transform.translation.z;
  auto rqx = t.transform.rotation.x;
  auto rqy = t.transform.rotation.y;
  auto rqz = t.transform.rotation.z;
  auto rqw = t.transform.rotation.w;

  // Correct transform
  auto cx = minimal_pub_->get_tf_world_talk().transform.translation.x;
  auto cy = minimal_pub_->get_tf_world_talk().transform.translation.y;
  auto cz = minimal_pub_->get_tf_world_talk().transform.translation.z;
  auto cqx = minimal_pub_->get_tf_world_talk().transform.rotation.x;
  auto cqy = minimal_pub_->get_tf_world_talk().transform.rotation.y;
  auto cqz = minimal_pub_->get_tf_world_talk().transform.rotation.z;
  auto cqw = minimal_pub_->get_tf_world_talk().transform.rotation.w;

  EXPECT_FLOAT_EQ(rx, cx);
  EXPECT_FLOAT_EQ(ry, cy);
  EXPECT_FLOAT_EQ(rz, cz);
  EXPECT_FLOAT_EQ(rqx, cqx);
  EXPECT_FLOAT_EQ(rqy, cqy);
  EXPECT_FLOAT_EQ(rqz, cqz);
  EXPECT_FLOAT_EQ(rqw, cqw);
}
