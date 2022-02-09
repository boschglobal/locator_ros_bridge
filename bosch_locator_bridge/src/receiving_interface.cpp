// Copyright (c) 2021 - for information on the respective copyright owner
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

#include "receiving_interface.hpp"

#include <Poco/NObserver.h>

#include <string>
#include <vector>

#include "rosmsgs_datagram_converter.hpp"

#include "bosch_locator_bridge/msg/client_global_align_visualization.hpp"
#include "bosch_locator_bridge/msg/client_localization_pose.hpp"
#include "bosch_locator_bridge/msg/client_localization_visualization.hpp"
#include "bosch_locator_bridge/msg/client_map_visualization.hpp"
#include "bosch_locator_bridge/msg/client_recording_visualization.hpp"

ReceivingInterface::ReceivingInterface(
  const Poco::Net::IPAddress & hostadress, Poco::UInt16 port,
  rclcpp::Node::SharedPtr node)
: node_(node),
  tf_broadcaster_(node),
  ccm_socket_(Poco::Net::SocketAddress(hostadress, port))
{
  reactor_.addEventHandler(
    ccm_socket_, Poco::NObserver<ReceivingInterface, Poco::Net::ReadableNotification>(
      *this, &ReceivingInterface::onReadEvent));
}

ReceivingInterface::~ReceivingInterface()
{
  reactor_.stop();
  ccm_socket_.shutdown();
}

void ReceivingInterface::onReadEvent(
  const Poco::AutoPtr<Poco::Net::ReadableNotification> & /*notification*/)
{
  try {
    // Create buffer with size of available data
    const int bytes_available = ccm_socket_.available();
    std::vector<char> msg(bytes_available);
    int received_bytes = ccm_socket_.receiveBytes(&(msg[0]), bytes_available);
    if (received_bytes == 0) {
      std::cout << "received msg of length 0... Connection closed? \n";
    } else {
      datagram_buffer_.insert(datagram_buffer_.end(), msg.begin(), msg.end());
      const auto bytes_to_delete = tryToParseData(datagram_buffer_, node_);
      datagram_buffer_.erase(datagram_buffer_.begin(), datagram_buffer_.begin() + bytes_to_delete);
    }
  } catch (const std::ios_base::failure & io_failure) {
    // catching this exception is actually no error:
    // the datagram is just not yet completely transmitted could not be
    // parsed because of that. Will automatically retry after more data is available.
  } catch (...) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Caught exception in ReceivingInterface!");
  }
}

void ReceivingInterface::run()
{
  reactor_.run();
}

void ReceivingInterface::publishTransform(
  const geometry_msgs::msg::PoseStamped & pose, const std::string & parent_frame,
  const std::string child_frame)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = pose.header.stamp;
  transform.header.frame_id = parent_frame;

  transform.transform.translation.x = pose.pose.position.x;
  transform.transform.translation.y = pose.pose.position.y;
  transform.transform.translation.z = pose.pose.position.z;
  transform.transform.rotation = pose.pose.orientation;

  transform.child_frame_id = child_frame;

  tf_broadcaster_.sendTransform(transform);
}

ClientControlModeInterface::ClientControlModeInterface(
  const Poco::Net::IPAddress & hostadress,
  rclcpp::Node::SharedPtr node)
: ReceivingInterface(hostadress, BINARY_CLIENT_CONTROL_MODE_PORT, node)
{
  // Setup publisher (use QoS settings to emulate a latched topic (ROS 1))
  client_control_mode_pub_ = node->create_publisher<bosch_locator_bridge::msg::ClientControlMode>(
    "~/client_control_mode", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

size_t ClientControlModeInterface::tryToParseData(
  const std::vector<char> & datagram,
  rclcpp::Node::SharedPtr node)
{
  // convert datagram to ros message
  bosch_locator_bridge::msg::ClientControlMode client_control_mode;
  const auto parsed_bytes =
    RosMsgsDatagramConverter::convertClientControlMode2Message(
    datagram,
    node->now(), client_control_mode);
  if (parsed_bytes > 0) {
    // publish client control mode
    client_control_mode_pub_->publish(client_control_mode);
  }
  return parsed_bytes;
}

ClientMapMapInterface::ClientMapMapInterface(
  const Poco::Net::IPAddress & hostadress,
  rclcpp::Node::SharedPtr node)
: ReceivingInterface(hostadress, BINARY_CLIENT_MAP_MAP_PORT, node)
{
  // Setup publisher
  client_map_map_pub_ =
    node->create_publisher<sensor_msgs::msg::PointCloud2>("~/client_map_map", 5);
}

size_t ClientMapMapInterface::tryToParseData(
  const std::vector<char> & datagram,
  rclcpp::Node::SharedPtr node)
{
  // convert datagram to ros message
  sensor_msgs::msg::PointCloud2 map;
  const auto parsed_bytes = RosMsgsDatagramConverter::convertMapDatagram2Message(
    datagram,
    node->now(), map);
  if (parsed_bytes > 0) {
    // publish
    client_map_map_pub_->publish(map);
  }
  return parsed_bytes;
}

ClientMapVisualizationInterface::ClientMapVisualizationInterface(
  const Poco::Net::IPAddress & hostadress,
  rclcpp::Node::SharedPtr node)
: ReceivingInterface(hostadress, BINARY_CLIENT_MAP_VISUALIZATION_PORT, node)
{
  // Setup publisher
  client_map_visualization_pub_ =
    node->create_publisher<bosch_locator_bridge::msg::ClientMapVisualization>(
    "~/client_map_visualization", 5);
  client_map_visualization_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/client_map_visualization/pose", 5);
  client_map_visualization_scan_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/client_map_visualization/scan", 5);
  client_map_visualization_path_poses_pub_ = node->create_publisher<geometry_msgs::msg::PoseArray>(
    "~/client_map_visualization/path_poses", 5);
}

size_t ClientMapVisualizationInterface::tryToParseData(
  const std::vector<char> & datagram,
  rclcpp::Node::SharedPtr /*node*/)
{
  // convert datagram to ros messages
  bosch_locator_bridge::msg::ClientMapVisualization client_map_visualization;
  geometry_msgs::msg::PoseStamped pose;
  sensor_msgs::msg::PointCloud2 scan;
  geometry_msgs::msg::PoseArray path_poses;

  const auto bytes_parsed = RosMsgsDatagramConverter::convertClientMapVisualizationDatagram2Message(
    datagram, client_map_visualization, pose, scan, path_poses);

  if (bytes_parsed > 0) {
    // publish
    publishTransform(pose, MAP_FRAME_ID, LASER_FRAME_ID);
    client_map_visualization_pub_->publish(client_map_visualization);
    client_map_visualization_pose_pub_->publish(pose);
    client_map_visualization_scan_pub_->publish(scan);
    client_map_visualization_path_poses_pub_->publish(path_poses);
  }
  return bytes_parsed;
}

ClientRecordingMapInterface::ClientRecordingMapInterface(
  const Poco::Net::IPAddress & hostadress,
  rclcpp::Node::SharedPtr node)
: ReceivingInterface(hostadress, BINARY_CLIENT_RECORDING_MAP_PORT, node)
{
  // Setup publisher
  client_recording_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/client_recording_map", 5);
}

size_t ClientRecordingMapInterface::tryToParseData(
  const std::vector<char> & datagram,
  rclcpp::Node::SharedPtr node)
{
  // convert datagram to ros message
  sensor_msgs::msg::PointCloud2 map;
  const auto parsed_bytes = RosMsgsDatagramConverter::convertMapDatagram2Message(
    datagram,
    node->now(), map);
  if (parsed_bytes > 0) {
    // publish
    client_recording_map_pub_->publish(map);
  }
  return parsed_bytes;
}

ClientRecordingVisualizationInterface::ClientRecordingVisualizationInterface(
  const Poco::Net::IPAddress & hostadress,
  rclcpp::Node::SharedPtr node)
: ReceivingInterface(hostadress, BINARY_CLIENT_RECORDING_VISUALIZATION_PORT, node)
{
  // Setup publisher
  client_recording_visualization_pub_ =
    node->create_publisher<bosch_locator_bridge::msg::ClientRecordingVisualization>(
    "~/client_recording_visualization", 5);
  client_recording_visualization_pose_pub_ =
    node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/client_recording_visualization/pose",
    5);
  client_recording_visualization_scan_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/client_recording_visualization/scan", 5);
  client_recording_visualization_path_poses_pub_ =
    node->create_publisher<geometry_msgs::msg::PoseArray>(
    "~/client_recording_visualization/path_poses", 5);
}

size_t ClientRecordingVisualizationInterface::tryToParseData(
  const std::vector<char> & datagram,
  rclcpp::Node::SharedPtr /*node*/)
{
  // convert datagram to ros messages
  bosch_locator_bridge::msg::ClientRecordingVisualization client_recording_visualization;
  geometry_msgs::msg::PoseStamped pose;
  sensor_msgs::msg::PointCloud2 scan;
  geometry_msgs::msg::PoseArray path_poses;

  const auto parsed_bytes =
    RosMsgsDatagramConverter::convertClientRecordingVisualizationDatagram2Message(
    datagram, client_recording_visualization, pose, scan, path_poses);

  if (parsed_bytes > 0) {
    // publish
    publishTransform(pose, MAP_FRAME_ID, LASER_FRAME_ID);
    client_recording_visualization_pub_->publish(client_recording_visualization);
    client_recording_visualization_pose_pub_->publish(pose);
    client_recording_visualization_scan_pub_->publish(scan);
    client_recording_visualization_path_poses_pub_->publish(path_poses);
  }
  return parsed_bytes;
}

ClientLocalizationMapInterface::ClientLocalizationMapInterface(
  const Poco::Net::IPAddress & hostadress,
  rclcpp::Node::SharedPtr node)
: ReceivingInterface(hostadress, BINARY_CLIENT_LOCALIZATION_MAP_PORT, node)
{
  // Setup publisher (use QoS settings to emulate a latched topic (ROS 1))
  client_localization_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/client_localization_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

size_t ClientLocalizationMapInterface::tryToParseData(
  const std::vector<char> & datagram,
  rclcpp::Node::SharedPtr node)
{
  // convert datagram to ros message
  sensor_msgs::msg::PointCloud2 map;
  const auto bytes_parsed = RosMsgsDatagramConverter::convertMapDatagram2Message(
    datagram,
    node->now(), map);
  if (bytes_parsed > 0) {
    // publish
    client_localization_map_pub_->publish(map);
  }
  return bytes_parsed;
}

ClientLocalizationVisualizationInterface::ClientLocalizationVisualizationInterface(
  const Poco::Net::IPAddress & hostadress, rclcpp::Node::SharedPtr node)
: ReceivingInterface(hostadress, BINARY_CLIENT_LOCALIZATION_VISUALIZATION_PORT, node)
{
  // Setup publisher
  client_localization_visualization_pub_ =
    node->create_publisher<bosch_locator_bridge::msg::ClientLocalizationVisualization>(
    "~/client_localization_visualization", 5);
  client_localization_visualization_pose_pub_ =
    node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/client_localization_visualization/pose", 5);
  client_localization_visualization_scan_pub_ =
    node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/client_localization_visualization/scan", 5);
}

size_t ClientLocalizationVisualizationInterface::tryToParseData(
  const std::vector<char> & datagram,
  rclcpp::Node::SharedPtr /*node*/)
{
  // convert datagram to ros messages
  bosch_locator_bridge::msg::ClientLocalizationVisualization client_localization_visualization;
  geometry_msgs::msg::PoseStamped pose;
  sensor_msgs::msg::PointCloud2 scan;

  const auto bytes_parsed =
    RosMsgsDatagramConverter::convertClientLocalizationVisualizationDatagram2Message(
    datagram, client_localization_visualization, pose, scan);

  if (bytes_parsed > 0) {
    // publish
    client_localization_visualization_pub_->publish(client_localization_visualization);
    client_localization_visualization_pose_pub_->publish(pose);
    client_localization_visualization_scan_pub_->publish(scan);
  }
  return bytes_parsed;
}

ClientLocalizationPoseInterface::ClientLocalizationPoseInterface(
  const Poco::Net::IPAddress & hostadress,
  rclcpp::Node::SharedPtr node)
: ReceivingInterface(hostadress, BINARY_CLIENT_LOCALIZATION_POSE_PORT, node)
{
  // Setup publisher
  client_localization_pose_pub_ =
    node->create_publisher<bosch_locator_bridge::msg::ClientLocalizationPose>(
    "~/client_localization_pose", 5);
  client_localization_pose_pose_pub_ =
    node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "~/client_localization_pose/pose", 5);
  client_localization_pose_lidar_odo_pose_pub_ =
    node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/client_localization_pose/lidar_odo_pose", 5);
}

size_t ClientLocalizationPoseInterface::tryToParseData(
  const std::vector<char> & datagram,
  rclcpp::Node::SharedPtr /*node*/)
{
  // convert datagram to ros messages
  bosch_locator_bridge::msg::ClientLocalizationPose client_localization_pose;
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::PoseWithCovarianceStamped poseWithCov;
  geometry_msgs::msg::PoseStamped lidar_odo_pose;

  double covariance[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  const auto bytes_parsed = RosMsgsDatagramConverter::convertClientLocalizationPoseDatagram2Message(
    datagram, client_localization_pose, pose, covariance, lidar_odo_pose);

  poseWithCov.pose.pose = pose.pose;
  poseWithCov.header = pose.header;
  // assign right triangle of 3x3 matrix to 6x6 matrix
  poseWithCov.pose.covariance[0] = covariance[0];
  poseWithCov.pose.covariance[1] = covariance[1];
  poseWithCov.pose.covariance[5] = covariance[2];
  poseWithCov.pose.covariance[7] = covariance[3];
  poseWithCov.pose.covariance[11] = covariance[4];
  poseWithCov.pose.covariance[35] = covariance[5];

  if (bytes_parsed > 0) {
    // publish
    publishTransform(pose, MAP_FRAME_ID, LASER_FRAME_ID);
    client_localization_pose_pub_->publish(client_localization_pose);
    client_localization_pose_pose_pub_->publish(poseWithCov);
    client_localization_pose_lidar_odo_pose_pub_->publish(lidar_odo_pose);
  }
  return bytes_parsed;
}

ClientGlobalAlignVisualizationInterface::ClientGlobalAlignVisualizationInterface(
  const Poco::Net::IPAddress & hostadress,
  rclcpp::Node::SharedPtr node)
: ReceivingInterface(hostadress, BINARY_CLIENT_GLOBAL_ALIGN_VISUALIZATION_PORT, node)
{
  // Setup publisher
  client_global_align_visualization_pub_ =
    node->create_publisher<bosch_locator_bridge::msg::ClientGlobalAlignVisualization>(
    "~/client_global_align_visualization", 5);
  client_global_align_visualization_poses_pub_ =
    node->create_publisher<geometry_msgs::msg::PoseArray>(
    "~/client_global_align_visualization/poses", 5);
  client_global_align_visualization_landmarks_poses_pub_ =
    node->create_publisher<geometry_msgs::msg::PoseArray>(
    "~/client_global_align_visualization/landmarks/poses", 5);
}

size_t ClientGlobalAlignVisualizationInterface::tryToParseData(
  const std::vector<char> & datagram,
  rclcpp::Node::SharedPtr /*node*/)
{
  // convert datagram to ros messages
  bosch_locator_bridge::msg::ClientGlobalAlignVisualization client_global_align_visualization;
  geometry_msgs::msg::PoseArray poses;
  geometry_msgs::msg::PoseArray landmark_poses;

  const auto bytes_parsed =
    RosMsgsDatagramConverter::convertClientGlobalAlignVisualizationDatagram2Message(
    datagram, client_global_align_visualization, poses, landmark_poses);

  if (bytes_parsed > 0) {
    // publish
    client_global_align_visualization_pub_->publish(client_global_align_visualization);
    client_global_align_visualization_poses_pub_->publish(poses);
    client_global_align_visualization_landmarks_poses_pub_->publish(landmark_poses);
  }
  return bytes_parsed;
}
