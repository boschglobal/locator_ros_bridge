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

#include "bosch_locator_bridge_utils/locator_node.hpp"

#include <memory>
#include <string>

using namespace std::chrono_literals;

using std::placeholders::_1;

namespace bosch_locator_bridge_utils
{
LocatorNode::LocatorNode(const std::string & node_name)
: nav2_util::LifecycleNode(node_name, "")
{
  add_parameter(
    "base_frame_id", rclcpp::ParameterValue(std::string("base_footprint")),
    "Which frame to use for the robot base");

  add_parameter(
    "global_frame_id", rclcpp::ParameterValue(
      std::string(
        "")),
    "The name of the coordinate frame published by the localization system. "
    "If not set, take frame_id of localization pose for it.");

  add_parameter(
    "odom_frame_id", rclcpp::ParameterValue(std::string("odom")),
    "Which frame to use for odometry");

  add_parameter(
    "pose_topic", rclcpp::ParameterValue(std::string("/bridge_node/client_localization_pose/pose")),
    "Topic to subscribe to in order to receive the localization poses");

  add_parameter(
    "transform_tolerance", rclcpp::ParameterValue(0.1),
    "Time with which to post-date the transform that is published, to indicate that this transform "
    "is valid into the future");
}

nav2_util::CallbackReturn LocatorNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  // Set parameters

  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("pose_topic", pose_topic_);

  double transform_tolerance;
  get_parameter("transform_tolerance", transform_tolerance);
  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);

  // Initialize transform listener and broadcaster
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

  odom_to_map_ = tf2::Transform::getIdentity();

  // Initialize subscribers
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    pose_topic_, 10, std::bind(&LocatorNode::poseCallback, this, std::placeholders::_1));

  // Initialize service clients

  serviceClientStartLocalization_ = \
    create_client<std_srvs::srv::Empty>("/bridge_node/start_localization");

  while (!serviceClientStartLocalization_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        get_logger(),
        "Interrupted while waiting for start_localization service. Exiting.");
      return nav2_util::CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(
      get_logger(),
      "Service start_localization not available, waiting again...");
  }

  serviceClientStopLocalization_ = \
    create_client<std_srvs::srv::Empty>("/bridge_node/stop_localization");

  while (!serviceClientStopLocalization_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        get_logger(),
        "Interrupted while waiting for stop_localization service. Exiting.");
      return nav2_util::CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(
      get_logger(),
      "Service stop_localization not available, waiting again...");
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LocatorNode::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  serviceClientStartLocalization_->async_send_request(
    std::make_shared<std_srvs::srv::Empty::Request>());

  active_ = true;

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LocatorNode::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  serviceClientStopLocalization_->async_send_request(
    std::make_shared<std_srvs::srv::Empty::Request>());

  active_ = false;

  // destroy bond
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LocatorNode::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  pose_sub_.reset();

  tf_broadcaster_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LocatorNode::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  return nav2_util::CallbackReturn::SUCCESS;
}

void LocatorNode::poseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose)
{
  // calculate map to odom transform

  // subtracting base to odom from map to base and send map to odom instead
  try {
    tf2::Quaternion quat(
      pose->pose.pose.orientation.x,
      pose->pose.pose.orientation.y,
      pose->pose.pose.orientation.z,
      pose->pose.pose.orientation.w);
    tf2::Vector3 vec(
      pose->pose.pose.position.x,
      pose->pose.pose.position.y,
      pose->pose.pose.position.z);
    tf2::Transform pose_tf(quat, vec);

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = base_frame_id_;
    pose_stamped.header.stamp = pose->header.stamp;
    tf2::toMsg(pose_tf.inverse(), pose_stamped.pose);

    geometry_msgs::msg::PoseStamped odom_to_map_pose;
    tf_buffer_->transform(pose_stamped, odom_to_map_pose, odom_frame_id_);
    tf2::impl::Converter<true, false>::convert(
      odom_to_map_pose.pose, odom_to_map_);
  } catch (tf2::TransformException & e) {
    RCLCPP_DEBUG(
      get_logger(),
      "Failed to subtract base to odom transform: (%s)", e.what());
  }

  // send map to odom transform

  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.frame_id = global_frame_id_.empty() ? pose->header.frame_id : global_frame_id_;
  map_to_odom.header.stamp = tf2_ros::toMsg(
    tf2_ros::fromMsg(pose->header.stamp) + transform_tolerance_);
  map_to_odom.child_frame_id = odom_frame_id_;
  tf2::impl::Converter<false, true>::convert(
    odom_to_map_.inverse(), map_to_odom.transform);
  tf_broadcaster_->sendTransform(map_to_odom);
}

}  // namespace bosch_locator_bridge_utils
