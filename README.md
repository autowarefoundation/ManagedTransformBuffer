# managed_transform_buffer

## Purpose

This package contains a wrapper of ROS 2 TF buffer & listener. It offers better performance in large systems with multiple TF listeners.

![Managed Transform Buffer](https://github.com/user-attachments/assets/97f472c8-7797-413e-a78b-e86f900797a6)

Apart from the obvious benefits in cases of static-only transformations, it also boosts performance for scenarios with Composable Node Containers thanks to the use of the Singleton pattern.

## Installation

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-select managed_transform_buffer
```

## Usage

Library exposes a few handy function for handling TFs and PointCloud2 transformations.

```cpp
// Create a managed TF buffer
auto managed_tf_buffer = std::make_unique<managed_transform_buffer::ManagedTransformBuffer>();

// Get a transform from source_frame to target_frame
auto tf_msg_transform = managed_tf_buffer->getTransform<geometry_msgs::msg::TransformStamped>("my_target_frame", "my_source_frame", this->now(), rclcpp::Duration::from_seconds(1));
auto tf2_transform = managed_tf_buffer->getTransform<tf2::Transform>("my_target_frame", "my_source_frame", this->now(), rclcpp::Duration::from_seconds(1));
auto eigen_transform = managed_tf_buffer->getTransform<Eigen::Matrix4f>("my_target_frame", "my_source_frame", this->now(), rclcpp::Duration::from_seconds(1));

if (tf_msg_transform.has_value())
{
  // Do something with the transform
}

// Transform a PointCloud2 message
sensor_msgs::msg::PointCloud2 transformed_cloud;
auto success = managed_tf_buffer->transformPointcloud("my_target_frame", *in_cloud_msg, transformed_cloud, this->now(), rclcpp::Duration::from_seconds(1));
```

For full example, see [example_managed_transform_buffer.cpp](managed_transform_buffer/examples/example_managed_transform_buffer.cpp).
You can also build this example and run it:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_EXAMPLES=On --packages-select managed_transform_buffer
ros2 run managed_transform_buffer example_managed_transform_buffer --ros-args -p target_frame:=my_target_frame -p source_frame:=my_source_frame -r input/cloud:=/my_input_cloud -r output/cloud:=/my_output_cloud
```

## Limitations

- Requests for dynamic transforms with zero timeout might never succeed. This limitation is due to the fact that the listener is initialized for each transform request (till first occurrence of dynamic transform). If timeout is zero, the listener will not have enough time to fill the buffer. This can be controlled with `discovery_timeout` parameter for `ManagedTransformBuffer` class constructor. `discovery_timeout` (default: 20ms) is used to set timeout for the first occurrence of any transform:

```cpp
auto managed_tf_buffer = std::make_unique<managed_transform_buffer::ManagedTransformBuffer>(RCL_ROS_TIME, false, tf2::durationFromSec(0.3));
```
