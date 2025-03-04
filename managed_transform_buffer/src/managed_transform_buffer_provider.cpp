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

#include <algorithm>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace managed_transform_buffer
{

std::unique_ptr<ManagedTransformBufferProvider> ManagedTransformBufferProvider::instance = nullptr;

ManagedTransformBufferProvider & ManagedTransformBufferProvider::getInstance(
  rcl_clock_type_t clock_type, const bool force_dynamic, tf2::Duration discovery_timeout,
  tf2::Duration cache_time)
{
  static ManagedTransformBufferProvider instance(clock_type, discovery_timeout, cache_time);
  if (force_dynamic) {
    std::lock_guard<std::mutex> lock(instance.mutex_);
    if (instance.isStatic()) {
      instance.registerAsDynamic();
    }
    if (clock_type != instance.clock_->get_clock_type()) {
      RCLCPP_WARN_THROTTLE(
        instance.logger_, *instance.clock_, 3000,
        "Input clock type does not match (%d vs. %d). Input clock type will be ignored.",
        clock_type, instance.clock_->get_clock_type());
    }
  }
  return instance;
}

std::optional<TransformStamped> ManagedTransformBufferProvider::getTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return get_transform_(target_frame, source_frame, time, timeout, logger);
}

bool ManagedTransformBufferProvider::isStatic() const
{
  return is_static_.load();
}

ManagedTransformBufferProvider::ManagedTransformBufferProvider(
  rcl_clock_type_t clock_type, tf2::Duration discovery_timeout, tf2::Duration cache_time)
: clock_(std::make_shared<rclcpp::Clock>(clock_type)),
  discovery_timeout_(discovery_timeout),
  logger_(rclcpp::get_logger("ManagedTransformBuffer"))
{
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_thread_ = std::make_shared<std::thread>(
    std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, executor_));

  static_tf_buffer_ = std::make_unique<TFMap>();
  tf_tree_ = std::make_unique<TreeMap>();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_, cache_time);
  tf_buffer_->setUsingDedicatedThread(true);

  tf_options_ = tf2_ros::detail::get_default_transform_listener_sub_options();
  tf_static_options_ = tf2_ros::detail::get_default_transform_listener_static_sub_options();
  cb_ = std::bind(&ManagedTransformBufferProvider::tfCallback, this, std::placeholders::_1, false);
  cb_static_ =
    std::bind(&ManagedTransformBufferProvider::tfCallback, this, std::placeholders::_1, true);
  options_.start_parameter_event_publisher(false);
  options_.start_parameter_services(false);
  random_engine_ = std::mt19937(std::random_device{}());
  dis_ = std::uniform_int_distribution<>(0, 0xFFFFFF);
  registerAsUnknown();
}

ManagedTransformBufferProvider::~ManagedTransformBufferProvider()
{
  executor_->cancel();
  if (executor_thread_->joinable()) {
    executor_thread_->join();
  }
}

void ManagedTransformBufferProvider::activateListener()
{
  options_.arguments({"--ros-args", "-r", "__node:=" + generateUniqueNodeName()});
  node_ = rclcpp::Node::make_unique("_", options_);
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node_->get_node_base_interface(), node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  tf_options_.callback_group = callback_group_;
  tf_options_.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  tf_static_options_.callback_group = callback_group_;
  tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", tf2_ros::DynamicListenerQoS(), cb_, tf_options_);
  tf_static_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", tf2_ros::StaticListenerQoS(), cb_static_, tf_static_options_);
  executor_->add_callback_group(callback_group_, node_->get_node_base_interface());
}

void ManagedTransformBufferProvider::deactivateListener()
{
  auto cb_grps = executor_->get_all_callback_groups();
  for (auto & cb_grp : cb_grps) {
    executor_->remove_callback_group(cb_grp.lock());
  }
  tf_static_sub_.reset();
  tf_sub_.reset();
  node_.reset();
}

void ManagedTransformBufferProvider::registerAsUnknown()
{
  get_transform_ = [this](
                     const std::string & target_frame, const std::string & source_frame,
                     const tf2::TimePoint & time, const tf2::Duration & timeout,
                     const rclcpp::Logger & logger) -> std::optional<TransformStamped> {
    // Try to get transform from local static buffer
    auto static_tf = getStaticTransform(target_frame, source_frame);
    if (static_tf) return static_tf;

    // Try to discover transform
    if (!isStatic()) {  // Another thread could switch to dynamic listener
      return getDynamicTransform(target_frame, source_frame, time, timeout, logger);
    }
    return getUnknownTransform(target_frame, source_frame, time, timeout, logger);
  };
}

void ManagedTransformBufferProvider::registerAsDynamic()
{
  is_static_.store(false);
  get_transform_ = [this](
                     const std::string & target_frame, const std::string & source_frame,
                     const tf2::TimePoint & time, const tf2::Duration & timeout,
                     const rclcpp::Logger & logger) -> std::optional<TransformStamped> {
    return getDynamicTransform(target_frame, source_frame, time, timeout, logger);
  };
}

std::string ManagedTransformBufferProvider::generateUniqueNodeName()
{
  std::stringstream sstream;
  sstream << "managed_tf_listener_impl_" << std::hex << dis_(random_engine_)
          << dis_(random_engine_);
  return sstream.str();
}

void ManagedTransformBufferProvider::tfCallback(
  const tf2_msgs::msg::TFMessage::SharedPtr msg, const bool is_static)
{
  std::string authority = "Authority undetectable";
  for (const auto & tf : msg->transforms) {
    try {
      tf_buffer_->setTransform(tf, authority, is_static);
      tf_tree_->emplace(tf.child_frame_id, TreeNode{tf.header.frame_id, is_static});
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_THROTTLE(
        logger_, *clock_, 3000, "Failure to set received transform from %s to %s with error: %s\n",
        tf.child_frame_id.c_str(), tf.header.frame_id.c_str(), ex.what());
    }
  }
}

std::optional<TransformStamped> ManagedTransformBufferProvider::lookupTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger)
{
  try {
    auto x = node_->get_logger();
    auto tf = tf_buffer_->lookupTransform(target_frame, source_frame, time, timeout);
    return std::make_optional<TransformStamped>(tf);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_THROTTLE(
      logger, *clock_, 3000, "Failure to get transform from %s to %s with error: %s",
      target_frame.c_str(), source_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

TraverseResult ManagedTransformBufferProvider::traverseTree(
  const std::string & target_frame, const std::string & source_frame, const tf2::Duration & timeout,
  const rclcpp::Logger & logger)
{
  std::atomic<bool> timeout_reached{false};

  auto framesToRoot = [this](
                        const std::string & start_frame, const TreeMap & tf_tree,
                        std::vector<std::string> & frames_out,
                        const rclcpp::Logger & logger) -> bool {
    frames_out.push_back(start_frame);
    uint32_t depth = 0;
    auto current_frame = start_frame;
    auto frame_it = tf_tree.find(current_frame);
    while (frame_it != tf_tree.end()) {
      current_frame = frame_it->second.parent;
      frames_out.push_back(current_frame);
      frame_it = tf_tree.find(current_frame);
      depth++;
      if (depth > tf2::BufferCore::MAX_GRAPH_DEPTH) {
        RCLCPP_ERROR_THROTTLE(
          logger, *clock_, 3000, "Traverse depth exceeded for %s", start_frame.c_str());
        return false;
      }
    }
    return true;
  };

  auto traverse = [this, &timeout_reached, &framesToRoot](
                    const std::string & t1, const std::string & t2,
                    const rclcpp::Logger & logger) -> TraverseResult {
    while (!timeout_reached.load()) {
      std::vector<std::string> frames_from_t1;
      std::vector<std::string> frames_from_t2;
      auto last_tf_tree = *tf_tree_;

      // Collect all frames from bottom to the top of TF tree for both frames
      if (
        !framesToRoot(t1, last_tf_tree, frames_from_t1, logger) ||
        !framesToRoot(t2, last_tf_tree, frames_from_t2, logger)) {
        // Possibly TF loop occurred
        return {false, false};
      }

      // Find common ancestor
      bool only_static_requested_from_t1 = true;
      bool only_static_requested_from_t2 = true;
      for (auto & frame_from_t1 : frames_from_t1) {
        auto frame_from_t1_it = last_tf_tree.find(frame_from_t1);
        if (frame_from_t1_it != last_tf_tree.end()) {  // Otherwise frame is TF root (no parent)
          only_static_requested_from_t1 &= last_tf_tree.at(frame_from_t1).is_static;
        }
        for (auto & frame_from_t2 : frames_from_t2) {
          auto frame_from_t2_it = last_tf_tree.find(frame_from_t2);
          if (frame_from_t2_it != last_tf_tree.end()) {  // Otherwise frame is TF root (no parent)
            only_static_requested_from_t2 &= last_tf_tree.at(frame_from_t2).is_static;
          }
          if (frame_from_t1 == frame_from_t2) {
            return {true, only_static_requested_from_t1 && only_static_requested_from_t2};
          }
        }
        only_static_requested_from_t1 = true;
        only_static_requested_from_t2 = true;
      }
    }
    // Timeout reached
    return {false, false};
  };

  std::future<TraverseResult> future =
    std::async(std::launch::async, traverse, target_frame, source_frame, logger);
  if (future.wait_for(timeout) == std::future_status::ready) {
    return future.get();
  }
  timeout_reached.store(true);
  return {false, false};
}

std::optional<TransformStamped> ManagedTransformBufferProvider::getDynamicTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger)
{
  return lookupTransform(target_frame, source_frame, time, timeout, logger);
}

std::optional<TransformStamped> ManagedTransformBufferProvider::getStaticTransform(
  const std::string & target_frame, const std::string & source_frame)
{
  auto key = std::make_pair(target_frame, source_frame);
  auto key_inv = std::make_pair(source_frame, target_frame);

  // Check if the transform is already in the buffer
  auto it = static_tf_buffer_->find(key);
  if (it != static_tf_buffer_->end()) {
    auto tf_msg = it->second;
    tf_msg.header.stamp = clock_->now();
    return std::make_optional<TransformStamped>(tf_msg);
  }

  // Check if the inverse transform is already in the buffer
  auto it_inv = static_tf_buffer_->find(key_inv);
  if (it_inv != static_tf_buffer_->end()) {
    auto tf_msg = it_inv->second;
    tf2::Transform tf;
    tf2::fromMsg(tf_msg.transform, tf);
    tf2::Transform inv_tf = tf.inverse();
    TransformStamped inv_tf_msg;
    inv_tf_msg.transform = tf2::toMsg(inv_tf);
    inv_tf_msg.header.frame_id = tf_msg.child_frame_id;
    inv_tf_msg.child_frame_id = tf_msg.header.frame_id;
    inv_tf_msg.header.stamp = clock_->now();
    static_tf_buffer_->emplace(key, inv_tf_msg);
    return std::make_optional<TransformStamped>(inv_tf_msg);
  }

  // Check if transform is needed
  if (target_frame == source_frame) {
    auto tf_identity = tf2::Transform::getIdentity();
    TransformStamped tf_msg;
    tf_msg.transform = tf2::toMsg(tf_identity);
    tf_msg.header.frame_id = target_frame;
    tf_msg.child_frame_id = source_frame;
    tf_msg.header.stamp = clock_->now();
    static_tf_buffer_->emplace(key, tf_msg);
    return std::make_optional<TransformStamped>(tf_msg);
  }

  return std::nullopt;
}

std::optional<TransformStamped> ManagedTransformBufferProvider::getUnknownTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger)
{
  // Initialize local TF listener and base TF listener
  activateListener();

  // Check local TF tree and TF buffer asynchronously
  auto discovery_timeout = std::max(timeout, discovery_timeout_);
  std::future<TraverseResult> traverse_future = std::async(
    std::launch::async, &ManagedTransformBufferProvider::traverseTree, this, target_frame,
    source_frame, discovery_timeout, logger);
  std::future<std::optional<TransformStamped>> tf_future = std::async(
    std::launch::async, &ManagedTransformBufferProvider::lookupTransform, this, target_frame,
    source_frame, time, discovery_timeout, logger);
  auto traverse_result = traverse_future.get();
  auto tf = tf_future.get();

  // Mimic lookup transform error if TF not exists in tree or buffer
  if (!traverse_result.success || !tf.has_value()) {
    deactivateListener();
    return std::nullopt;
  }

  // If TF is static, add it to the static buffer. Otherwise, switch to dynamic listener
  if (traverse_result.is_static) {
    auto key = std::make_pair(target_frame, source_frame);
    static_tf_buffer_->emplace(key, tf.value());
    deactivateListener();
  } else {
    registerAsDynamic();
    RCLCPP_INFO_THROTTLE(
      logger, *clock_, 3000, "Transform %s -> %s is dynamic. Switching to dynamic listener.",
      target_frame.c_str(), source_frame.c_str());
  }

  return tf;
}

}  // namespace managed_transform_buffer
