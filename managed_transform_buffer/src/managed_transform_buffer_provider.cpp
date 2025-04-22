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

#include "managed_transform_buffer/managed_transform_buffer_provider.hpp"

#include <tf2/LinearMath/Transform.hpp>
#include <tf2/convert.hpp>
#include <tf2_ros/qos.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace managed_transform_buffer
{

std::optional<TransformStamped> ManagedTransformBufferProvider::getTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger)
{
  return getDynamicTransform(target_frame, source_frame, time, timeout, logger);
}

bool ManagedTransformBufferProvider::isStatic() const
{
  return false;
}

rclcpp::Clock::SharedPtr ManagedTransformBufferProvider::getClock() const
{
  return clock_;
}

ManagedTransformBufferProvider::ManagedTransformBufferProvider(
  rcl_clock_type_t clock_type, tf2::Duration cache_time)
: clock_(std::make_shared<rclcpp::Clock>(clock_type)),
  logger_(rclcpp::get_logger("managed_transform_buffer"))
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_, cache_time);
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

std::optional<TransformStamped> ManagedTransformBufferProvider::lookupTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger) const
{
  try {
    auto tf = tf_buffer_->lookupTransform(target_frame, source_frame, time, timeout);
    return std::make_optional<TransformStamped>(tf);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_THROTTLE(
      logger, *clock_, 3000, "Failure to get transform from %s to %s with error: %s",
      target_frame.c_str(), source_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

std::optional<TransformStamped> ManagedTransformBufferProvider::getDynamicTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger) const
{
  return lookupTransform(target_frame, source_frame, time, timeout, logger);
}

}  // namespace managed_transform_buffer
