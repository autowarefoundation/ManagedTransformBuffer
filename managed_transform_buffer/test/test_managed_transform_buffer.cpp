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
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cstdint>
#include <memory>
#include <string>

class TestManagedTransformBuffer : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::unique_ptr<managed_transform_buffer::ManagedTransformBuffer> managed_tf_buffer_{nullptr};
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
  geometry_msgs::msg::TransformStamped tf_map_to_base_;
  geometry_msgs::msg::TransformStamped tf_base_to_lidar_top_;
  geometry_msgs::msg::TransformStamped tf_base_to_lidar_right_;
  Eigen::Matrix4f eigen_map_to_base_;
  Eigen::Matrix4f eigen_base_to_lidar_top_;
  Eigen::Matrix4f eigen_base_to_lidar_right_;
  std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud_in_{nullptr};
  double precision_;
  rclcpp::Time time_;
  rclcpp::Duration timeout_ = rclcpp::Duration::from_seconds(1);

  geometry_msgs::msg::TransformStamped generateTransformMsg(
    const int32_t seconds, const uint32_t nanoseconds, const std::string & parent_frame,
    const std::string & child_frame, double x, double y, double z, double qx, double qy, double qz,
    double qw)
  {
    rclcpp::Time timestamp(seconds, nanoseconds, node_->get_clock()->get_clock_type());
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = timestamp;
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = child_frame;
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = z;
    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;
    return tf_msg;
  }

  void broadcastDynamicTf(geometry_msgs::msg::TransformStamped transform, uint32_t seconds = 1)
  {
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this, transform]() -> void {
      tf_broadcaster_->sendTransform(transform);
    });

    rclcpp::Rate r(10);
    rclcpp::spin_some(node_);
    for (uint32_t i = 0; i < 10u * seconds; ++i) {
      r.sleep();
      rclcpp::spin_some(node_);
    }

    timer_->cancel();
    timer_->reset();
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_managed_transform_buffer");
    managed_tf_buffer_ = std::make_unique<managed_transform_buffer::ManagedTransformBuffer>(
      node_->get_clock()->get_clock_type());
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    tf_map_to_base_ = generateTransformMsg(
      10, 100'000'000, "map", "base_link", 120.0, 240.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    tf_base_to_lidar_top_ = generateTransformMsg(
      10, 100'000'000, "base_link", "lidar_top", 0.690, 0.000, 2.100, -0.007, -0.007, 0.692, 0.722);
    tf_base_to_lidar_right_ = generateTransformMsg(
      10, 100'000'000, "base_link", "lidar_right", 0.0, -0.56362, -0.30555, 0.244, 0.248, 0.665,
      0.661);
    eigen_map_to_base_ = tf2::transformToEigen(tf_map_to_base_).matrix().cast<float>();
    eigen_base_to_lidar_top_ = tf2::transformToEigen(tf_base_to_lidar_top_).matrix().cast<float>();
    eigen_base_to_lidar_right_ =
      tf2::transformToEigen(tf_base_to_lidar_right_).matrix().cast<float>();
    cloud_in_ = std::make_unique<sensor_msgs::msg::PointCloud2>();
    precision_ = 0.01;
    time_ = rclcpp::Time(10, 100'000'000);

    // Set up the fields for x, y, and z coordinates
    cloud_in_->fields.resize(3);
    sensor_msgs::PointCloud2Modifier modifier(*cloud_in_);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    // Resize the cloud to hold points_per_pointcloud_ points
    modifier.resize(10);

    // Create an iterator for the x, y, z fields
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_in_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_in_, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_in_, "z");

    // Populate the point cloud
    for (size_t i = 0; i < modifier.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
      *iter_x = static_cast<float>(i);
      *iter_y = static_cast<float>(i);
      *iter_z = static_cast<float>(i);
    }

    // Set up cloud header
    cloud_in_->header.frame_id = "lidar_top";
    cloud_in_->header.stamp = time_;

    ASSERT_TRUE(rclcpp::ok());
  }

  void TearDown() override {}
};

TEST_F(TestManagedTransformBuffer, TestReturn)
{
  static_tf_broadcaster_->sendTransform(tf_base_to_lidar_top_);

  auto eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "lidar_top", time_, timeout_);
  EXPECT_TRUE(eigen_transform.has_value());

  auto tf2_transform =
    managed_tf_buffer_->getTransform<tf2::Transform>("base_link", "lidar_top", time_, timeout_);
  EXPECT_TRUE(tf2_transform.has_value());

  auto tf_msg_transform = managed_tf_buffer_->getTransform<geometry_msgs::msg::TransformStamped>(
    "base_link", "lidar_top", time_, timeout_);
  EXPECT_TRUE(tf_msg_transform.has_value());
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformNoExist)
{
  static_tf_broadcaster_->sendTransform(tf_base_to_lidar_top_);

  auto eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "fake_link", time_, timeout_);
  EXPECT_FALSE(eigen_transform.has_value());
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformBase)
{
  static_tf_broadcaster_->sendTransform(tf_base_to_lidar_top_);
  auto eigen_base_to_lidar_top =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "lidar_top", time_, timeout_);
  ASSERT_TRUE(eigen_base_to_lidar_top.has_value());
  EXPECT_TRUE(eigen_base_to_lidar_top.value().isApprox(eigen_base_to_lidar_top_, precision_));
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformSameFrame)
{
  static_tf_broadcaster_->sendTransform(tf_base_to_lidar_top_);

  auto eigen_base_to_base =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "base_link", time_, timeout_);
  ASSERT_TRUE(eigen_base_to_base.has_value());
  EXPECT_TRUE(eigen_base_to_base.value().isApprox(Eigen::Matrix4f::Identity(), precision_));
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformInverse)
{
  static_tf_broadcaster_->sendTransform(tf_base_to_lidar_top_);

  auto eigen_lidar_top_tobase =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("lidar_top", "base_link", time_, timeout_);
  ASSERT_TRUE(eigen_lidar_top_tobase.has_value());
  EXPECT_TRUE(
    eigen_lidar_top_tobase.value().isApprox(eigen_base_to_lidar_top_.inverse(), precision_));
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformNonDirect)
{
  static_tf_broadcaster_->sendTransform(tf_base_to_lidar_top_);
  static_tf_broadcaster_->sendTransform(tf_base_to_lidar_right_);

  auto eigen_lidar_top_to_lidar_right =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("lidar_top", "lidar_right", time_, timeout_);
  ASSERT_TRUE(eigen_lidar_top_to_lidar_right.has_value());
  EXPECT_TRUE(eigen_lidar_top_to_lidar_right.value().isApprox(
    eigen_base_to_lidar_top_.inverse() * eigen_base_to_lidar_right_, precision_));
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformDynamic)
{
  static_tf_broadcaster_->sendTransform(tf_base_to_lidar_top_);
  static_tf_broadcaster_->sendTransform(tf_base_to_lidar_right_);

  std::future<void> future =
    std::async(std::launch::async, [this]() { broadcastDynamicTf(tf_map_to_base_); });
  auto eigen_map_to_base =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("map", "base_link", time_, timeout_);
  future.wait();

  ASSERT_TRUE(eigen_map_to_base.has_value());
  EXPECT_TRUE(eigen_map_to_base.value().isApprox(eigen_map_to_base_, precision_));
  EXPECT_FALSE(managed_tf_buffer_->isStatic());

  auto eigen_lidar_top_to_lidar_right =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("lidar_top", "lidar_right", time_, timeout_);
  ASSERT_TRUE(eigen_lidar_top_to_lidar_right.has_value());
  EXPECT_TRUE(eigen_lidar_top_to_lidar_right.value().isApprox(
    eigen_base_to_lidar_top_.inverse() * eigen_base_to_lidar_right_, precision_));
  EXPECT_FALSE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformMultipleCall)
{
  EXPECT_FALSE(managed_tf_buffer_->isStatic());

  static_tf_broadcaster_->sendTransform(tf_base_to_lidar_top_);

  std::optional<Eigen::Matrix4f> eigen_transform;
  eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "fake_link", time_, timeout_);
  EXPECT_FALSE(eigen_transform.has_value());

  eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("lidar_top", "base_link", time_, timeout_);
  ASSERT_TRUE(eigen_transform.has_value());
  EXPECT_TRUE(eigen_transform.value().isApprox(eigen_base_to_lidar_top_.inverse(), precision_));

  eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("fake_link", "fake_link", time_, timeout_);
  ASSERT_TRUE(eigen_transform.has_value());
  EXPECT_TRUE(eigen_transform.value().isApprox(Eigen::Matrix4f::Identity(), precision_));

  eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "lidar_top", time_, timeout_);
  ASSERT_TRUE(eigen_transform.has_value());
  EXPECT_TRUE(eigen_transform.value().isApprox(eigen_base_to_lidar_top_, precision_));

  eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("fake_link", "lidar_top", time_, timeout_);
  EXPECT_FALSE(eigen_transform.has_value());

  eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("base_link", "lidar_top", time_, timeout_);
  ASSERT_TRUE(eigen_transform.has_value());
  EXPECT_TRUE(eigen_transform.value().isApprox(eigen_base_to_lidar_top_, precision_));

  std::future<void> future =
    std::async(std::launch::async, [this]() { broadcastDynamicTf(tf_map_to_base_); });
  auto eigen_map_to_base =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("map", "base_link", time_, timeout_);
  future.wait();
  ASSERT_TRUE(eigen_map_to_base.has_value());
  EXPECT_TRUE(eigen_map_to_base.value().isApprox(eigen_map_to_base_, precision_));

  eigen_transform =
    managed_tf_buffer_->getTransform<Eigen::Matrix4f>("fake_link", "fake_link", time_, timeout_);
  ASSERT_TRUE(eigen_transform.has_value());
  EXPECT_TRUE(eigen_transform.value().isApprox(Eigen::Matrix4f::Identity(), precision_));
  EXPECT_FALSE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformEmptyPointCloud)
{
  static_tf_broadcaster_->sendTransform(tf_base_to_lidar_top_);

  auto cloud_in = std::make_unique<sensor_msgs::msg::PointCloud2>();
  cloud_in->header.frame_id = "lidar_top";
  cloud_in->header.stamp = rclcpp::Time(10, 100'000'000);
  auto cloud_out = std::make_unique<sensor_msgs::msg::PointCloud2>();

  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("lidar_top", *cloud_in, *cloud_out, time_, timeout_));
  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("base_link", *cloud_in, *cloud_out, time_, timeout_));
  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("fake_link", *cloud_in, *cloud_out, time_, timeout_));
}

TEST_F(TestManagedTransformBuffer, TestTransformEmptyPointCloudNoHeader)
{
  static_tf_broadcaster_->sendTransform(tf_base_to_lidar_top_);
  auto cloud_in = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto cloud_out = std::make_unique<sensor_msgs::msg::PointCloud2>();

  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("lidar_top", *cloud_in, *cloud_out, time_, timeout_));
  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("base_link", *cloud_in, *cloud_out, time_, timeout_));
  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("fake_link", *cloud_in, *cloud_out, time_, timeout_));
}

TEST_F(TestManagedTransformBuffer, TestTransformPointCloud)
{
  static_tf_broadcaster_->sendTransform(tf_base_to_lidar_top_);
  auto cloud_out = std::make_unique<sensor_msgs::msg::PointCloud2>();

  // Transform cloud with header
  EXPECT_TRUE(
    managed_tf_buffer_->transformPointcloud("lidar_top", *cloud_in_, *cloud_out, time_, timeout_));
  EXPECT_TRUE(
    managed_tf_buffer_->transformPointcloud("base_link", *cloud_in_, *cloud_out, time_, timeout_));
  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("fake_link", *cloud_in_, *cloud_out, time_, timeout_));
}

TEST_F(TestManagedTransformBuffer, TestTransformPointCloudNoHeader)
{
  static_tf_broadcaster_->sendTransform(tf_base_to_lidar_top_);
  auto cloud_out = std::make_unique<sensor_msgs::msg::PointCloud2>();

  // Transform cloud without header
  auto cloud_in = std::make_unique<sensor_msgs::msg::PointCloud2>(*cloud_in_);
  cloud_in->header.frame_id = "";
  cloud_in->header.stamp = rclcpp::Time(0, 0);
  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("lidar_top", *cloud_in, *cloud_out, time_, timeout_));
  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("base_link", *cloud_in, *cloud_out, time_, timeout_));
  EXPECT_FALSE(
    managed_tf_buffer_->transformPointcloud("fake_link", *cloud_in, *cloud_out, time_, timeout_));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
