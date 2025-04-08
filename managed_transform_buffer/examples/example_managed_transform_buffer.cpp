// Copyright 2024 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc.

#include "managed_transform_buffer/managed_transform_buffer.hpp"

#include <eigen3/Eigen/Core>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>

class ExampleNode : public rclcpp::Node
{
public:
  ExampleNode() : Node("managed_transform_buffer_example")
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "dummy_target_frame");
    source_frame_ = declare_parameter<std::string>("source_frame", "dummy_source_frame");
    managed_tf_buffer_ = std::make_unique<managed_transform_buffer::ManagedTransformBuffer>();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input/cloud", rclcpp::SensorDataQoS(),
      std::bind(&ExampleNode::cloudCb, this, std::placeholders::_1));
    cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("output/cloud", rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&ExampleNode::timerCb, this));
  }

private:
  std::unique_ptr<managed_transform_buffer::ManagedTransformBuffer> managed_tf_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string target_frame_;
  std::string source_frame_;

  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    if (managed_tf_buffer_->transformPointcloud(
          target_frame_, *msg, transformed_cloud, this->now(), rclcpp::Duration::from_seconds(1))) {
      RCLCPP_INFO(get_logger(), "Pointcloud transformed");
      cloud_pub_->publish(transformed_cloud);
    }
  }

  void timerCb()
  {
    auto tf = managed_tf_buffer_->getTransform<Eigen::Matrix4f>(
      target_frame_, source_frame_, this->now(), rclcpp::Duration::from_seconds(1));

    if (tf.has_value()) {
      RCLCPP_INFO(
        get_logger(), "Got transform with x: %f, y: %f, z: %f", tf.value()(0, 3), tf.value()(1, 3),
        tf.value()(2, 3));
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
