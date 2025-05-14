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

#include <Eigen/src/Geometry/Transform.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>

namespace managed_transform_buffer
{

ManagedTransformBuffer::ManagedTransformBuffer(
  rcl_clock_type_t clock_type, const bool force_dynamic, tf2::Duration discovery_timeout,
  tf2::Duration cache_time)
{
  provider_ = &ManagedTransformBufferProvider::getInstance(
    clock_type, force_dynamic, discovery_timeout, cache_time);
}

template <>
std::optional<TransformStamped> ManagedTransformBuffer::getTransform<TransformStamped>(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger)
{
  return provider_->getTransform(target_frame, source_frame, time, timeout, logger);
}

template <>
std::optional<tf2::Transform> ManagedTransformBuffer::getTransform<tf2::Transform>(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger)
{
  auto tf = provider_->getTransform(target_frame, source_frame, time, timeout, logger);
  if (!tf.has_value()) {
    return std::nullopt;
  }

  tf2::Transform tf2_transform;
  tf2::fromMsg(tf.value().transform, tf2_transform);

  return std::make_optional<tf2::Transform>(tf2_transform);
}

template <>
std::optional<Eigen::Affine3f> ManagedTransformBuffer::getTransform<Eigen::Affine3f>(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger)
{
  auto tf = provider_->getTransform(target_frame, source_frame, time, timeout, logger);
  if (!tf.has_value()) {
    return std::nullopt;
  }

  Eigen::Vector3f translation(
    static_cast<float>(tf->transform.translation.x),
    static_cast<float>(tf->transform.translation.y),
    static_cast<float>(tf->transform.translation.z));
  Eigen::Quaternionf rotation(
    static_cast<float>(tf->transform.rotation.w), static_cast<float>(tf->transform.rotation.x),
    static_cast<float>(tf->transform.rotation.y), static_cast<float>(tf->transform.rotation.z));
  Eigen::Affine3f eigen_transform = Eigen::Translation3f(translation) * rotation;

  return std::make_optional<Eigen::Affine3f>(eigen_transform);
}

template <>
std::optional<Eigen::Affine3d> ManagedTransformBuffer::getTransform<Eigen::Affine3d>(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger)
{
  auto tf = provider_->getTransform(target_frame, source_frame, time, timeout, logger);
  if (!tf.has_value()) {
    return std::nullopt;
  }

  Eigen::Vector3d translation(
    tf->transform.translation.x, tf->transform.translation.y, tf->transform.translation.z);
  Eigen::Quaterniond rotation(
    tf->transform.rotation.w, tf->transform.rotation.x, tf->transform.rotation.y,
    tf->transform.rotation.z);
  Eigen::Affine3d eigen_transform = Eigen::Translation3d(translation) * rotation;

  return std::make_optional<Eigen::Affine3d>(eigen_transform);
}

template <>
std::optional<Eigen::Matrix4f> ManagedTransformBuffer::getTransform<Eigen::Matrix4f>(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger)
{
  auto tf = provider_->getTransform(target_frame, source_frame, time, timeout, logger);
  if (!tf.has_value()) {
    return std::nullopt;
  }

  Eigen::Vector3f translation(
    static_cast<float>(tf->transform.translation.x),
    static_cast<float>(tf->transform.translation.y),
    static_cast<float>(tf->transform.translation.z));
  Eigen::Quaternionf rotation(
    static_cast<float>(tf->transform.rotation.w), static_cast<float>(tf->transform.rotation.x),
    static_cast<float>(tf->transform.rotation.y), static_cast<float>(tf->transform.rotation.z));
  Eigen::Matrix3f rotation_matrix = rotation.toRotationMatrix();
  Eigen::Matrix4f eigen_transform = Eigen::Matrix4f::Identity();
  static_assert(!Eigen::Matrix4f::IsRowMajor, "Matrices should be column major.");
  eigen_transform.block<3, 3>(0, 0) = rotation_matrix;
  eigen_transform.block<3, 1>(0, 3) = translation;

  return std::make_optional<Eigen::Matrix4f>(eigen_transform);
}

template <>
std::optional<Eigen::Matrix4d> ManagedTransformBuffer::getTransform<Eigen::Matrix4d>(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger)
{
  auto tf = provider_->getTransform(target_frame, source_frame, time, timeout, logger);
  if (!tf.has_value()) {
    return std::nullopt;
  }

  Eigen::Vector3d translation(
    tf->transform.translation.x, tf->transform.translation.y, tf->transform.translation.z);
  Eigen::Quaterniond rotation(
    tf->transform.rotation.w, tf->transform.rotation.x, tf->transform.rotation.y,
    tf->transform.rotation.z);
  Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
  Eigen::Matrix4d eigen_transform = Eigen::Matrix4d::Identity();
  static_assert(!Eigen::Matrix4d::IsRowMajor, "Matrices should be column major.");
  eigen_transform.block<3, 3>(0, 0) = rotation_matrix;
  eigen_transform.block<3, 1>(0, 3) = translation;

  return std::make_optional<Eigen::Matrix4d>(eigen_transform);
}

template <>
std::optional<TransformStamped> ManagedTransformBuffer::getTransform<TransformStamped>(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
  const rclcpp::Duration & timeout, const rclcpp::Logger & logger)
{
  return getTransform<TransformStamped>(
    target_frame, source_frame, tf2_ros::fromRclcpp(time), tf2_ros::fromRclcpp(timeout), logger);
}

template <>
std::optional<tf2::Transform> ManagedTransformBuffer::getTransform<tf2::Transform>(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
  const rclcpp::Duration & timeout, const rclcpp::Logger & logger)
{
  return getTransform<tf2::Transform>(
    target_frame, source_frame, tf2_ros::fromRclcpp(time), tf2_ros::fromRclcpp(timeout), logger);
}

template <>
std::optional<Eigen::Affine3f> ManagedTransformBuffer::getTransform<Eigen::Affine3f>(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
  const rclcpp::Duration & timeout, const rclcpp::Logger & logger)
{
  return getTransform<Eigen::Affine3f>(
    target_frame, source_frame, tf2_ros::fromRclcpp(time), tf2_ros::fromRclcpp(timeout), logger);
}

template <>
std::optional<Eigen::Affine3d> ManagedTransformBuffer::getTransform<Eigen::Affine3d>(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
  const rclcpp::Duration & timeout, const rclcpp::Logger & logger)
{
  return getTransform<Eigen::Affine3d>(
    target_frame, source_frame, tf2_ros::fromRclcpp(time), tf2_ros::fromRclcpp(timeout), logger);
}

template <>
std::optional<Eigen::Matrix4f> ManagedTransformBuffer::getTransform<Eigen::Matrix4f>(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
  const rclcpp::Duration & timeout, const rclcpp::Logger & logger)
{
  return getTransform<Eigen::Matrix4f>(
    target_frame, source_frame, tf2_ros::fromRclcpp(time), tf2_ros::fromRclcpp(timeout), logger);
}

template <>
std::optional<Eigen::Matrix4d> ManagedTransformBuffer::getTransform<Eigen::Matrix4d>(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
  const rclcpp::Duration & timeout, const rclcpp::Logger & logger)
{
  return getTransform<Eigen::Matrix4d>(
    target_frame, source_frame, tf2_ros::fromRclcpp(time), tf2_ros::fromRclcpp(timeout), logger);
}

template <>
std::optional<TransformStamped> ManagedTransformBuffer::getLatestTransform<TransformStamped>(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Logger & logger)
{
  return getTransform<TransformStamped>(
    target_frame, source_frame, tf2::TimePointZero, tf2::Duration::zero(), logger);
}

template <>
std::optional<tf2::Transform> ManagedTransformBuffer::getLatestTransform<tf2::Transform>(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Logger & logger)
{
  return getTransform<tf2::Transform>(
    target_frame, source_frame, tf2::TimePointZero, tf2::Duration::zero(), logger);
}

template <>
std::optional<Eigen::Affine3f> ManagedTransformBuffer::getLatestTransform<Eigen::Affine3f>(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Logger & logger)
{
  return getTransform<Eigen::Affine3f>(
    target_frame, source_frame, tf2::TimePointZero, tf2::Duration::zero(), logger);
}

template <>
std::optional<Eigen::Affine3d> ManagedTransformBuffer::getLatestTransform<Eigen::Affine3d>(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Logger & logger)
{
  return getTransform<Eigen::Affine3d>(
    target_frame, source_frame, tf2::TimePointZero, tf2::Duration::zero(), logger);
}

template <>
std::optional<Eigen::Matrix4f> ManagedTransformBuffer::getLatestTransform<Eigen::Matrix4f>(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Logger & logger)
{
  return getTransform<Eigen::Matrix4f>(
    target_frame, source_frame, tf2::TimePointZero, tf2::Duration::zero(), logger);
}

template <>
std::optional<Eigen::Matrix4d> ManagedTransformBuffer::getLatestTransform<Eigen::Matrix4d>(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Logger & logger)
{
  return getTransform<Eigen::Matrix4d>(
    target_frame, source_frame, tf2::TimePointZero, tf2::Duration::zero(), logger);
}

bool ManagedTransformBuffer::transformPointcloud(
  const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & cloud_in,
  sensor_msgs::msg::PointCloud2 & cloud_out, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger)
{
  if (
    pcl::getFieldIndex(cloud_in, "x") == -1 || pcl::getFieldIndex(cloud_in, "y") == -1 ||
    pcl::getFieldIndex(cloud_in, "z") == -1) {
    RCLCPP_ERROR_THROTTLE(
      logger, *provider_->getClock(), 3000,
      "Cloud does not contain x, y, z fields. Cannot transform.");
    return false;
  }
  if (target_frame.empty() || cloud_in.header.frame_id.empty()) {
    RCLCPP_ERROR_THROTTLE(
      logger, *provider_->getClock(), 3000,
      "Target frame (%s) or source frame (%s) is empty. Cannot transform.", target_frame.c_str(),
      cloud_in.header.frame_id.c_str());
    return false;
  }
  if (target_frame == cloud_in.header.frame_id || cloud_in.data.empty()) {
    cloud_out = cloud_in;
    cloud_out.header.frame_id = target_frame;
    return true;
  }
  auto eigen_transform =
    getTransform<Eigen::Matrix4f>(target_frame, cloud_in.header.frame_id, time, timeout, logger);
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
  const rclcpp::Duration & timeout, const rclcpp::Logger & logger)
{
  return transformPointcloud(
    target_frame, cloud_in, cloud_out, tf2_ros::fromRclcpp(time), tf2_ros::fromRclcpp(timeout),
    logger);
}

bool ManagedTransformBuffer::isStatic() const
{
  return provider_->isStatic();
}

rclcpp::Logger ManagedTransformBuffer::defaultLogger()
{
  return rclcpp::get_logger("managed_transform_buffer");
}

}  // namespace managed_transform_buffer
