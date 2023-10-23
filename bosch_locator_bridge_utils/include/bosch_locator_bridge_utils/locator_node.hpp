// Copyright (c) 2022 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/boschglobal/locator_ros_bridge.
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

#ifndef BOSCH_LOCATOR_BRIDGE_UTILS__LOCATOR_NODE_HPP_
#define BOSCH_LOCATOR_BRIDGE_UTILS__LOCATOR_NODE_HPP_

#include <memory>
#include <string>

#include "nav2_util/lifecycle_node.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bosch_locator_bridge_utils
{
class LocatorNode : public nav2_util::LifecycleNode
{
public:
  explicit LocatorNode(const std::string & node_name);

protected:
  nav2_util::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state) override;

  std::atomic<bool> active_ {false};

  // Callbacks
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose);

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  // Service clients
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr serviceClientStartLocalization_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr serviceClientStopLocalization_;

  // Transforms
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2::Transform odom_to_map_;

  std::string base_frame_id_;
  std::string global_frame_id_;
  std::string odom_frame_id_;
  std::string pose_topic_;
  tf2::Duration transform_tolerance_;
};
}  // namespace bosch_locator_bridge_utils

#endif  // BOSCH_LOCATOR_BRIDGE_UTILS__LOCATOR_NODE_HPP_
