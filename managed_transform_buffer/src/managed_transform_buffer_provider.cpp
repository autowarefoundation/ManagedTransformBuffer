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
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace managed_transform_buffer
{

ManagedTransformBufferProvider & ManagedTransformBufferProvider::getInstance(
  rcl_clock_type_t clock_type, const bool force_dynamic, tf2::Duration discovery_timeout,
  tf2::Duration cache_time)
{
  static ManagedTransformBufferProvider instance(clock_type, discovery_timeout, cache_time);
  if (force_dynamic) {
    if (instance.isStatic()) {
      std::lock_guard<std::mutex> listener_lock(instance.listener_mutex_);
      std::unique_lock<std::shared_mutex> unq_buffer_lock(instance.buffer_mutex_);
      std::unique_lock<std::shared_mutex> unq_tree_lock(instance.tree_mutex_);
      instance.is_static_.store(false);
    }
  }
  if (clock_type != instance.clock_->get_clock_type()) {
    RCLCPP_WARN_THROTTLE(
      instance.logger_, *instance.clock_, 3000,
      "Input clock type does not match (%d vs. %d). Input clock type will be ignored.", clock_type,
      instance.clock_->get_clock_type());
  }
  return instance;
}

std::optional<TransformStamped> ManagedTransformBufferProvider::getTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger)
{
  if (isStatic()) {
    auto static_tf = getStaticTransform(target_frame, source_frame, time);
    if (static_tf) return static_tf;
    return getUnknownTransform(target_frame, source_frame, time, timeout, logger);
  }
  return getDynamicTransform(target_frame, source_frame, time, timeout, logger);
}

bool ManagedTransformBufferProvider::isStatic() const
{
  return is_static_.load();
}

rclcpp::Clock::SharedPtr ManagedTransformBufferProvider::getClock() const
{
  return clock_;
}

ManagedTransformBufferProvider::ManagedTransformBufferProvider(
  rcl_clock_type_t clock_type, tf2::Duration discovery_timeout, tf2::Duration cache_time)
: clock_(std::make_shared<rclcpp::Clock>(clock_type)),
  discovery_timeout_(discovery_timeout),
  logger_(rclcpp::get_logger("managed_transform_buffer"))
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
  is_static_.store(true);
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
  std::lock_guard<std::mutex> listener_lock(listener_mutex_);
  operational_threads_.fetch_add(1);
  if (operational_threads_.load() != 1) return;
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
  std::lock_guard<std::mutex> listener_lock(listener_mutex_);
  operational_threads_.fetch_sub(1);
  if (operational_threads_.load() != 0) return;
  auto cb_grps = executor_->get_all_callback_groups();
  for (auto & cb_grp : cb_grps) {
    executor_->remove_callback_group(cb_grp.lock());
  }
  tf_static_sub_.reset();
  tf_sub_.reset();
  node_.reset();
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
      if (isStatic()) {  // Only static TF listener needs TF tree
        std::shared_lock<std::shared_mutex> sh_tree_lock(tree_mutex_);
        auto tree_it = tf_tree_->find(tf.child_frame_id);
        if (tree_it == tf_tree_->end()) {
          sh_tree_lock.unlock();
          std::unique_lock<std::shared_mutex> unq_tree_lock(tree_mutex_);
          tf_tree_->emplace(tf.child_frame_id, TreeNode{tf.header.frame_id, is_static});
        }
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_THROTTLE(
        logger_, *clock_, 3000, "Failure to set received transform from %s to %s with error: %s\n",
        tf.child_frame_id.c_str(), tf.header.frame_id.c_str(), ex.what());
    }
  }
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

TraverseResult ManagedTransformBufferProvider::traverseTree(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Logger & logger)
{
  auto frames_to_root = [this](
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

  std::vector<std::string> frames_from_t1;
  std::vector<std::string> frames_from_t2;
  std::shared_lock<std::shared_mutex> sh_tree_lock(tree_mutex_);
  auto last_tf_tree = *tf_tree_;
  sh_tree_lock.unlock();

  // Collect all frames from bottom to the top of TF tree for both frames
  if (
    !frames_to_root(target_frame, last_tf_tree, frames_from_t1, logger) ||
    !frames_to_root(source_frame, last_tf_tree, frames_from_t2, logger)) {
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

  // No common ancestor found
  return {false, false};
}

std::optional<TransformStamped> ManagedTransformBufferProvider::getDynamicTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger) const
{
  return lookupTransform(target_frame, source_frame, time, timeout, logger);
}

std::optional<TransformStamped> ManagedTransformBufferProvider::getStaticTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time)
{
  std::shared_lock<std::shared_mutex> sh_buffer_lock(buffer_mutex_);

  // Check if the transform is already in the buffer
  auto key = std::make_pair(target_frame, source_frame);
  auto it = static_tf_buffer_->find(key);
  if (it != static_tf_buffer_->end()) {
    auto tf_msg = it->second;
    tf_msg.header.stamp = tf2_ros::toRclcpp(time);
    return std::make_optional<TransformStamped>(tf_msg);
  }

  // Check if the inverse transform is already in the buffer
  auto key_inv = std::make_pair(source_frame, target_frame);
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
    inv_tf_msg.header.stamp = tf2_ros::toRclcpp(time);
    sh_buffer_lock.unlock();
    std::unique_lock<std::shared_mutex> unq_buffer_lock(buffer_mutex_);
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
    tf_msg.header.stamp = tf2_ros::toRclcpp(time);
    sh_buffer_lock.unlock();
    std::unique_lock<std::shared_mutex> unq_buffer_lock(buffer_mutex_);
    static_tf_buffer_->emplace(key, tf_msg);
    return std::make_optional<TransformStamped>(tf_msg);
  }

  return std::nullopt;
}

std::optional<TransformStamped> ManagedTransformBufferProvider::getUnknownTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, const rclcpp::Logger & logger)
{
  // Initialize TF listener
  activateListener();

  // Check TF buffer
  auto discovery_timeout = std::max(timeout, discovery_timeout_);
  auto tf = lookupTransform(target_frame, source_frame, time, discovery_timeout, logger);
  if (!tf.has_value()) {
    deactivateListener();
    return std::nullopt;
  }

  // Check whether TF is static
  auto traverse_result = traverseTree(target_frame, source_frame, logger);
  if (!traverse_result.success) {
    deactivateListener();
    return std::nullopt;
  }

  // If TF is static, add it to the static buffer. Otherwise, switch to dynamic listener
  if (traverse_result.is_static) {
    auto key = std::make_pair(target_frame, source_frame);
    std::unique_lock<std::shared_mutex> unq_buffer_lock(buffer_mutex_);
    static_tf_buffer_->emplace(key, tf.value());
    unq_buffer_lock.unlock();
    deactivateListener();
  } else {
    if (isStatic()) {
      is_static_.store(false);
      RCLCPP_INFO(
        logger, "Transform %s -> %s is dynamic. Switching to dynamic listener.",
        target_frame.c_str(), source_frame.c_str());
    }
  }

  return tf;
}

}  // namespace managed_transform_buffer
