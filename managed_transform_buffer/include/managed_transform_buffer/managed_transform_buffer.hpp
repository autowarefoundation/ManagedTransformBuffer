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

#ifndef MANAGED_TRANSFORM_BUFFER__MANAGED_TRANSFORM_BUFFER_HPP_
#define MANAGED_TRANSFORM_BUFFER__MANAGED_TRANSFORM_BUFFER_HPP_

#include "managed_transform_buffer/managed_transform_buffer_provider.hpp"

#include <eigen3/Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <optional>
#include <string>
#include <type_traits>

namespace managed_transform_buffer
{
using geometry_msgs::msg::TransformStamped;

/**
 * @brief A managed TF buffer that handles listener node lifetime. This buffer triggers listener
 * only for first occurrence of frames pair. After that, the local buffer is used for storing
 * static transforms. If a dynamic transform is detected, the listener is switched to dynamic mode
 * and acts as a regular TF buffer.
 */
class ManagedTransformBuffer
{
public:
  /**
   * @brief Construct a new Managed Transform Buffer object
   *
   * @param[in] clock A clock to use for time and sleeping
   * @param[in] cache_time How long to keep a history of transforms
   */
  explicit ManagedTransformBuffer(
    rclcpp::Clock::SharedPtr clock,
    tf2::Duration cache_time = tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME));

  /** @brief Destroy the Managed Transform Buffer object */
  ~ManagedTransformBuffer();

  /**
   * @brief Get the transform between two frames by frame ID.
   *
   * @tparam T the type of the transformation to retrieve
   * @param[in] target_frame the frame to which data should be transformed
   * @param[in] source_frame the frame where the data originated
   * @param[in] time the time at which the value of the transform is desired (0 will get the latest)
   * @param[in] timeout how long to block before failing
   * @return an optional containing the transform if successful, or empty if not
   *
   * @overload getTransform<geometry_msgs::msg::TransformStamped>
   * @return An optional containing the TransformStamped if successful, or empty if not
   *
   * @overload getTransform<Eigen::Matrix4f>
   * @return An optional containing the Eigen::Matrix4f if successful, or empty if not
   *
   * @overload getTransform<tf2::Transform>
   * @return An optional containing the tf2::Transform if successful, or empty if not
   */
  template <typename T = TransformStamped>
  std::enable_if_t<std::is_same_v<T, TransformStamped>, std::optional<TransformStamped>>
  getTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout);

  template <typename T = Eigen::Matrix4f>
  std::enable_if_t<std::is_same_v<T, Eigen::Matrix4f>, std::optional<Eigen::Matrix4f>> getTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout);

  template <typename T = tf2::Transform>
  std::enable_if_t<std::is_same_v<T, tf2::Transform>, std::optional<tf2::Transform>> getTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout);

  /**
   * @brief Get the transform between two frames by frame ID.
   *
   * @sa getTransform(const std::string &, const std::string &, const tf2::TimePoint &, const
   * tf2::Duration)
   */
  template <typename T = TransformStamped>
  std::enable_if_t<std::is_same_v<T, TransformStamped>, std::optional<TransformStamped>>
  getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    const rclcpp::Duration & timeout);

  template <typename T = Eigen::Matrix4f>
  std::enable_if_t<std::is_same_v<T, Eigen::Matrix4f>, std::optional<Eigen::Matrix4f>> getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    const rclcpp::Duration & timeout);

  template <typename T = tf2::Transform>
  std::enable_if_t<std::is_same_v<T, tf2::Transform>, std::optional<tf2::Transform>> getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    const rclcpp::Duration & timeout);

  /**
   * @brief Transforms a point cloud from one frame to another.
   *
   * @param[in] target_frame the target TF frame
   * @param[in] cloud_in the input point cloud
   * @param[out] cloud_out the resultant output point cloud
   * @param[in] time the time at which the value of the transform is desired
   * @param[in] timeout how long to block before failing
   * @return true if the transformation is successful, false otherwise
   */
  bool transformPointcloud(
    const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & cloud_in,
    sensor_msgs::msg::PointCloud2 & cloud_out, const tf2::TimePoint & time,
    const tf2::Duration & timeout);

  /**
   * @brief Transforms a point cloud from one frame to another.
   *
   * @sa transformPointcloud(const std::string &, const sensor_msgs::msg::PointCloud2 &,
   * sensor_msgs::msg::PointCloud2 &, const tf2::TimePoint &, const tf2::Duration)
   */
  bool transformPointcloud(
    const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & cloud_in,
    sensor_msgs::msg::PointCloud2 & cloud_out, const rclcpp::Time & time,
    const rclcpp::Duration & timeout);

  /** @brief Check if all TFs requests have been for static TF so far.
   * @return true if only static TFs have been requested
   */
  bool isStatic() const;

private:
  ManagedTransformBufferProvider * provider_;
};

}  // namespace managed_transform_buffer

#endif  // MANAGED_TRANSFORM_BUFFER__MANAGED_TRANSFORM_BUFFER_HPP_
