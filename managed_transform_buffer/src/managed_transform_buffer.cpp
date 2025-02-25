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

#include <pcl_ros/transforms.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace managed_transform_buffer
{

ManagedTransformBuffer::ManagedTransformBuffer(
  rclcpp::Clock::SharedPtr clock, tf2::Duration cache_time)
{
  provider_ = &ManagedTransformBufferProvider::getInstance(clock, cache_time);
}

ManagedTransformBuffer::~ManagedTransformBuffer() = default;

template <>
std::optional<TransformStamped> ManagedTransformBuffer::getTransform<TransformStamped>(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout)
{
  return provider_->getTransform(target_frame, source_frame, time, timeout);
}

template <>
std::optional<Eigen::Matrix4f> ManagedTransformBuffer::getTransform<Eigen::Matrix4f>(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout)
{
  auto tf = provider_->getTransform(target_frame, source_frame, time, timeout);
  if (!tf.has_value()) {
    return std::nullopt;
  }
  Eigen::Matrix4f eigen_transform;
  pcl_ros::transformAsMatrix(tf.value(), eigen_transform);
  return std::make_optional<Eigen::Matrix4f>(eigen_transform);
}

template <>
std::optional<tf2::Transform> ManagedTransformBuffer::getTransform<tf2::Transform>(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout)
{
  auto tf = provider_->getTransform(target_frame, source_frame, time, timeout);
  if (!tf.has_value()) {
    return std::nullopt;
  }
  tf2::Transform tf2_transform;
  tf2::fromMsg(tf.value().transform, tf2_transform);
  return std::make_optional<tf2::Transform>(tf2_transform);
}

template <>
std::optional<TransformStamped> ManagedTransformBuffer::getTransform<TransformStamped>(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
  const rclcpp::Duration & timeout)
{
  return getTransform<TransformStamped>(
    target_frame, source_frame, tf2_ros::fromRclcpp(time), tf2_ros::fromRclcpp(timeout));
}

template <>
std::optional<Eigen::Matrix4f> ManagedTransformBuffer::getTransform<Eigen::Matrix4f>(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
  const rclcpp::Duration & timeout)
{
  return getTransform<Eigen::Matrix4f>(
    target_frame, source_frame, tf2_ros::fromRclcpp(time), tf2_ros::fromRclcpp(timeout));
}

template <>
std::optional<tf2::Transform> ManagedTransformBuffer::getTransform<tf2::Transform>(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
  const rclcpp::Duration & timeout)
{
  return getTransform<tf2::Transform>(
    target_frame, source_frame, tf2_ros::fromRclcpp(time), tf2_ros::fromRclcpp(timeout));
}

bool ManagedTransformBuffer::transformPointcloud(
  const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & cloud_in,
  sensor_msgs::msg::PointCloud2 & cloud_out, const tf2::TimePoint & time,
  const tf2::Duration & timeout)
{
  if (
    pcl::getFieldIndex(cloud_in, "x") == -1 || pcl::getFieldIndex(cloud_in, "y") == -1 ||
    pcl::getFieldIndex(cloud_in, "z") == -1) {
    return false;
  }
  if (target_frame == cloud_in.header.frame_id) {
    cloud_out = cloud_in;
    return true;
  }
  auto eigen_transform =
    getTransform<Eigen::Matrix4f>(target_frame, cloud_in.header.frame_id, time, timeout);
  if (!eigen_transform.has_value()) {
    return false;
  }
  pcl_ros::transformPointCloud(eigen_transform.value(), cloud_in, cloud_out);
  cloud_out.header.frame_id = target_frame;
  return true;
}

bool ManagedTransformBuffer::transformPointcloud(
  const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & cloud_in,
  sensor_msgs::msg::PointCloud2 & cloud_out, const rclcpp::Time & time,
  const rclcpp::Duration & timeout)
{
  return transformPointcloud(
    target_frame, cloud_in, cloud_out, tf2_ros::fromRclcpp(time), tf2_ros::fromRclcpp(timeout));
}

bool ManagedTransformBuffer::isStatic() const
{
  return provider_->isStatic();
}

}  // namespace managed_transform_buffer
