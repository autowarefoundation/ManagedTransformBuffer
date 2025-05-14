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
#include <eigen3/Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

#include <chrono>
#include <optional>
#include <string>
#include <type_traits>

namespace managed_transform_buffer
{
using geometry_msgs::msg::TransformStamped;
constexpr tf2::Duration DISCOVERY_TIMEOUT = std::chrono::milliseconds(20);

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
   * @param[in] clock_type type of the clock
   * @param[in] force_dynamic if true, TF listener will be activated during initialization
   * @param[in] discovery_timeout how long to wait for first TF discovery
   * @param[in] cache_time how long to keep a history of transforms
   */
  explicit ManagedTransformBuffer(
    rcl_clock_type_t clock_type = RCL_ROS_TIME, const bool force_dynamic = false,
    tf2::Duration discovery_timeout = tf2::Duration(managed_transform_buffer::DISCOVERY_TIMEOUT),
    tf2::Duration cache_time = tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME));

  /** @brief Destroy the Managed Transform Buffer object */
  ~ManagedTransformBuffer() = default;

  /**
   * @brief Get the transform between two frames by frame ID.
   *
   * @tparam T the type of the transformation to retrieve
   * @param[in] target_frame the frame to which data should be transformed
   * @param[in] source_frame the frame where the data originated
   * @param[in] time the time at which the value of the transform is desired (0 will get the latest)
   * @param[in] timeout how long to block before failing
   * @param[in] logger logger, if not specified, default logger will be used
   * @return an optional containing the transform if successful, or empty if not
   *
   * @overload getTransform<geometry_msgs::msg::TransformStamped>
   * @return An optional containing the TransformStamped if successful, or empty if not
   *
   * @overload getTransform<tf2::Transform>
   * @return An optional containing the tf2::Transform if successful, or empty if not
   *
   * @overload getTransform<Eigen::Affine3f>
   * @return An optional containing the Eigen::Affine3f if successful, or empty if not
   *
   * @overload getTransform<Eigen::Affine3d>
   * @return An optional containing the Eigen::Affine3d if successful, or empty if not
   *
   * @overload getTransform<Eigen::Matrix4f>
   * @return An optional containing the Eigen::Matrix4f if successful, or empty if not
   *
   * @overload getTransform<Eigen::Matrix4d>
   * @return An optional containing the Eigen::Matrix4d if successful, or empty if not
   */
  template <typename T = TransformStamped>
  std::enable_if_t<std::is_same_v<T, TransformStamped>, std::optional<TransformStamped>>
  getTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout, const rclcpp::Logger & logger = defaultLogger());

  template <typename T = tf2::Transform>
  std::enable_if_t<std::is_same_v<T, tf2::Transform>, std::optional<tf2::Transform>> getTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout, const rclcpp::Logger & logger = defaultLogger());

  template <typename T = Eigen::Affine3f>
  std::enable_if_t<std::is_same_v<T, Eigen::Affine3f>, std::optional<Eigen::Affine3f>> getTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout, const rclcpp::Logger & logger = defaultLogger());

  template <typename T = Eigen::Affine3d>
  std::enable_if_t<std::is_same_v<T, Eigen::Affine3d>, std::optional<Eigen::Affine3d>> getTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout, const rclcpp::Logger & logger = defaultLogger());

  template <typename T = Eigen::Matrix4f>
  std::enable_if_t<std::is_same_v<T, Eigen::Matrix4f>, std::optional<Eigen::Matrix4f>> getTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout, const rclcpp::Logger & logger = defaultLogger());

  template <typename T = Eigen::Matrix4d>
  std::enable_if_t<std::is_same_v<T, Eigen::Matrix4d>, std::optional<Eigen::Matrix4d>> getTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout, const rclcpp::Logger & logger = defaultLogger());

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
    const rclcpp::Duration & timeout, const rclcpp::Logger & logger = defaultLogger());

  template <typename T = tf2::Transform>
  std::enable_if_t<std::is_same_v<T, tf2::Transform>, std::optional<tf2::Transform>> getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    const rclcpp::Duration & timeout, const rclcpp::Logger & logger = defaultLogger());

  template <typename T = Eigen::Affine3f>
  std::enable_if_t<std::is_same_v<T, Eigen::Affine3f>, std::optional<Eigen::Affine3f>> getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    const rclcpp::Duration & timeout, const rclcpp::Logger & logger = defaultLogger());

  template <typename T = Eigen::Affine3d>
  std::enable_if_t<std::is_same_v<T, Eigen::Affine3d>, std::optional<Eigen::Affine3d>> getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    const rclcpp::Duration & timeout, const rclcpp::Logger & logger = defaultLogger());

  template <typename T = Eigen::Matrix4f>
  std::enable_if_t<std::is_same_v<T, Eigen::Matrix4f>, std::optional<Eigen::Matrix4f>> getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    const rclcpp::Duration & timeout, const rclcpp::Logger & logger = defaultLogger());

  template <typename T = Eigen::Matrix4d>
  std::enable_if_t<std::is_same_v<T, Eigen::Matrix4d>, std::optional<Eigen::Matrix4d>> getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    const rclcpp::Duration & timeout, const rclcpp::Logger & logger = defaultLogger());

  /**
   * @brief Get the latest transform between two frames by frame ID.
   *
   * @tparam T the type of the transformation to retrieve
   * @param[in] target_frame the frame to which data should be transformed
   * @param[in] source_frame the frame where the data originated
   * @param[in] logger logger, if not specified, default logger will be used
   * @return an optional containing the transform if successful, or empty if not
   *
   * @overload getTransform<geometry_msgs::msg::TransformStamped>
   * @return An optional containing the TransformStamped if successful, or empty if not
   *
   * @overload getTransform<tf2::Transform>
   * @return An optional containing the tf2::Transform if successful, or empty if not
   *
   * @overload getTransform<Eigen::Affine3f>
   * @return An optional containing the Eigen::Affine3f if successful, or empty if not
   *
   * @overload getTransform<Eigen::Affine3d>
   * @return An optional containing the Eigen::Affine3d if successful, or empty if not
   *
   * @overload getTransform<Eigen::Matrix4f>
   * @return An optional containing the Eigen::Matrix4f if successful, or empty if not
   *
   * @overload getTransform<Eigen::Matrix4d>
   * @return An optional containing the Eigen::Matrix4d if successful, or empty if not
   */
  template <typename T = TransformStamped>
  std::enable_if_t<std::is_same_v<T, TransformStamped>, std::optional<TransformStamped>>
  getLatestTransform(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Logger & logger = defaultLogger());

  template <typename T = tf2::Transform>
  std::enable_if_t<std::is_same_v<T, tf2::Transform>, std::optional<tf2::Transform>>
  getLatestTransform(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Logger & logger = defaultLogger());

  template <typename T = Eigen::Affine3f>
  std::enable_if_t<std::is_same_v<T, Eigen::Affine3f>, std::optional<Eigen::Affine3f>>
  getLatestTransform(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Logger & logger = defaultLogger());

  template <typename T = Eigen::Affine3d>
  std::enable_if_t<std::is_same_v<T, Eigen::Affine3d>, std::optional<Eigen::Affine3d>>
  getLatestTransform(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Logger & logger = defaultLogger());

  template <typename T = Eigen::Matrix4f>
  std::enable_if_t<std::is_same_v<T, Eigen::Matrix4f>, std::optional<Eigen::Matrix4f>>
  getLatestTransform(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Logger & logger = defaultLogger());

  template <typename T = Eigen::Matrix4d>
  std::enable_if_t<std::is_same_v<T, Eigen::Matrix4d>, std::optional<Eigen::Matrix4d>>
  getLatestTransform(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Logger & logger = defaultLogger());

  /**
   * @brief Transforms a point cloud from one frame to another.
   *
   * @param[in] target_frame the target TF frame
   * @param[in] cloud_in the input point cloud
   * @param[out] cloud_out the resultant output point cloud
   * @param[in] time the time at which the value of the transform is desired
   * @param[in] timeout how long to block before failing
   * @param[in] logger logger, if not specified, default logger will be used
   * @return true if the transformation is successful, false otherwise
   */
  bool transformPointcloud(
    const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & cloud_in,
    sensor_msgs::msg::PointCloud2 & cloud_out, const tf2::TimePoint & time,
    const tf2::Duration & timeout, const rclcpp::Logger & logger = defaultLogger());

  /**
   * @brief Transforms a point cloud from one frame to another.
   *
   * @sa transformPointcloud(const std::string &, const sensor_msgs::msg::PointCloud2 &,
   * sensor_msgs::msg::PointCloud2 &, const tf2::TimePoint &, const tf2::Duration)
   */
  template <typename PointT>
  bool transformPointcloud(
    const std::string & target_frame, const pcl::PointCloud<PointT> & cloud_in,
    pcl::PointCloud<PointT> & cloud_out, const tf2::TimePoint & time, const tf2::Duration & timeout,
    const rclcpp::Logger & logger = defaultLogger())
  {
    if (target_frame.empty() || cloud_in.header.frame_id.empty()) {
      RCLCPP_ERROR_THROTTLE(
        logger, *provider_->getClock(), 3000,
        "Target frame (%s) or source frame (%s) is empty. Cannot transform.", target_frame.c_str(),
        cloud_in.header.frame_id.c_str());
      return false;
    }
    if (target_frame == cloud_in.header.frame_id || cloud_in.points.empty()) {
      cloud_out = cloud_in;
      cloud_out.header.frame_id = target_frame;
      return true;
    }
    auto eigen_transform =
      getTransform<Eigen::Matrix4f>(target_frame, cloud_in.header.frame_id, time, timeout, logger);
    if (!eigen_transform.has_value()) {
      return false;
    }
    pcl::transformPointCloud(cloud_in, cloud_out, eigen_transform.value());
    cloud_out.header.frame_id = target_frame;
    return true;
  }

  /**
   * @brief Transforms a point cloud from one frame to another.
   *
   * @sa transformPointcloud(const std::string &, const sensor_msgs::msg::PointCloud2 &,
   * sensor_msgs::msg::PointCloud2 &, const tf2::TimePoint &, const tf2::Duration)
   */
  bool transformPointcloud(
    const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & cloud_in,
    sensor_msgs::msg::PointCloud2 & cloud_out, const rclcpp::Time & time,
    const rclcpp::Duration & timeout, const rclcpp::Logger & logger = defaultLogger());

  /**
   * @brief Transforms a point cloud from one frame to another.
   *
   * @sa transformPointcloud(const std::string &, const sensor_msgs::msg::PointCloud2 &,
   * sensor_msgs::msg::PointCloud2 &, const rclcpp::Time &, const rclcpp::Duration)
   */
  template <typename PointT>
  bool transformPointcloud(
    const std::string & target_frame, const pcl::PointCloud<PointT> & cloud_in,
    pcl::PointCloud<PointT> & cloud_out, const rclcpp::Time & time,
    const rclcpp::Duration & timeout, const rclcpp::Logger & logger = defaultLogger())
  {
    return transformPointcloud<PointT>(
      target_frame, cloud_in, cloud_out, tf2_ros::fromRclcpp(time), tf2_ros::fromRclcpp(timeout),
      logger);
  }

  /** @brief Check if all TFs requests have been for static TF so far.
   *
   * @return true if only static TFs have been requested
   */
  bool isStatic() const;

private:
  /** @brief Get the default logger.
   *
   * @return the default logger
   */
  static rclcpp::Logger defaultLogger();

  ManagedTransformBufferProvider * provider_;
};

}  // namespace managed_transform_buffer

#endif  // MANAGED_TRANSFORM_BUFFER__MANAGED_TRANSFORM_BUFFER_HPP_
