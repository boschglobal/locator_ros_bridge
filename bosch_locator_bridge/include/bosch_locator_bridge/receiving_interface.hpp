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

#ifndef BOSCH_LOCATOR_BRIDGE__RECEIVING_INTERFACE_HPP_
#define BOSCH_LOCATOR_BRIDGE__RECEIVING_INTERFACE_HPP_

#include <Poco/BinaryReader.h>
#include <Poco/Net/NetException.h>
#include <Poco/Net/SocketNotification.h>
#include <Poco/Net/SocketReactor.h>
#include <Poco/Net/SocketStream.h>
#include <Poco/Net/StreamSocket.h>

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "bosch_locator_bridge/msg/client_control_mode.hpp"
#include "bosch_locator_bridge/msg/client_global_align_visualization.hpp"
#include "bosch_locator_bridge/msg/client_localization_pose.hpp"
#include "bosch_locator_bridge/msg/client_localization_visualization.hpp"
#include "bosch_locator_bridge/msg/client_map_visualization.hpp"
#include "bosch_locator_bridge/msg/client_recording_visualization.hpp"

/**
 * @brief The ReceivingInterface class is the base class for all receiving interfaces, such as
 * ClientControlModeInterface, etc.
 */
class ReceivingInterface : public Poco::Runnable
{
public:
  ReceivingInterface(
    const Poco::Net::IPAddress & hostadress, Poco::UInt16 port,
    rclcpp::Node::SharedPtr node);

  virtual ~ReceivingInterface();

  virtual void onReadEvent(const Poco::AutoPtr<Poco::Net::ReadableNotification> & notification);

  void run();

protected:
  /**
   * @brief Actual function to be overwritten by child to handle data, e.g., convert to ros messages and
   * publish
   * @param datagram_buffer The data received via the binary connection socket
   * @return amount of bytes successfully parsed and can be removed from the buffer (0 if not parsing failed)
   */
  virtual void tryToParseData(Poco::BinaryReader & binary_reader) = 0;

  void publishTransform(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & parent_frame, const std::string child_frame);

  //! Node
  rclcpp::Node::SharedPtr node_;

  //! Broadcaster
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // port definitions for the different interfaces. See Locator API documentation section 12.8
  static constexpr Poco::UInt16 BINARY_CLIENT_CONTROL_MODE_PORT {9004};
  static constexpr Poco::UInt16 BINARY_CLIENT_MAP_MAP_PORT {9005};
  static constexpr Poco::UInt16 BINARY_CLIENT_MAP_VISUALIZATION_PORT {9006};
  static constexpr Poco::UInt16 BINARY_CLIENT_RECORDING_MAP_PORT {9007};
  static constexpr Poco::UInt16 BINARY_CLIENT_RECORDING_VISUALIZATION_PORT {9008};
  static constexpr Poco::UInt16 BINARY_CLIENT_LOCALIZATION_MAP_PORT {9009};
  static constexpr Poco::UInt16 BINARY_CLIENT_LOCALIZATION_VISUALIZATION_PORT {9010};
  static constexpr Poco::UInt16 BINARY_CLIENT_LOCALIZATION_POSE_PORT {9011};
  static constexpr Poco::UInt16 BINARY_CLIENT_GLOBAL_ALIGN_VISUALIZATION_PORT {9012};

private:
  Poco::Net::StreamSocket ccm_socket_;
  Poco::Net::SocketReactor reactor_;
  Poco::Net::SocketInputStream buffer_stream_ {ccm_socket_};
  Poco::BinaryReader binary_reader_ {buffer_stream_, Poco::BinaryReader::LITTLE_ENDIAN_BYTE_ORDER};
};

class ClientControlModeInterface : public ReceivingInterface
{
public:
  ClientControlModeInterface(const Poco::Net::IPAddress & hostadress, rclcpp::Node::SharedPtr node);
  void tryToParseData(Poco::BinaryReader & binary_reader) override;

private:
  rclcpp::Publisher<bosch_locator_bridge::msg::ClientControlMode>
  ::SharedPtr client_control_mode_pub_;
};

class ClientMapMapInterface : public ReceivingInterface
{
public:
  ClientMapMapInterface(const Poco::Net::IPAddress & hostadress, rclcpp::Node::SharedPtr node);
  void tryToParseData(Poco::BinaryReader & binary_reader) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr client_map_map_pub_;
};

class ClientMapVisualizationInterface : public ReceivingInterface
{
public:
  ClientMapVisualizationInterface(
    const Poco::Net::IPAddress & hostadress,
    rclcpp::Node::SharedPtr node);
  void tryToParseData(Poco::BinaryReader & binary_reader) override;

private:
  rclcpp::Publisher<bosch_locator_bridge::msg::ClientMapVisualization>
  ::SharedPtr client_map_visualization_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>
  ::SharedPtr client_map_visualization_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>
  ::SharedPtr client_map_visualization_scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>
  ::SharedPtr client_map_visualization_path_poses_pub_;
};

class ClientRecordingMapInterface : public ReceivingInterface
{
public:
  ClientRecordingMapInterface(
    const Poco::Net::IPAddress & hostadress,
    rclcpp::Node::SharedPtr node);
  void tryToParseData(Poco::BinaryReader & binary_reader) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr client_recording_map_pub_;
};

class ClientRecordingVisualizationInterface : public ReceivingInterface
{
public:
  ClientRecordingVisualizationInterface(
    const Poco::Net::IPAddress & hostadress,
    rclcpp::Node::SharedPtr node);
  void tryToParseData(Poco::BinaryReader & binary_reader) override;

private:
  rclcpp::Publisher<bosch_locator_bridge::msg::ClientRecordingVisualization>
  ::SharedPtr client_recording_visualization_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>
  ::SharedPtr client_recording_visualization_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>
  ::SharedPtr client_recording_visualization_scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>
  ::SharedPtr client_recording_visualization_path_poses_pub_;
};

class ClientLocalizationMapInterface : public ReceivingInterface
{
public:
  ClientLocalizationMapInterface(
    const Poco::Net::IPAddress & hostadress,
    rclcpp::Node::SharedPtr node);
  void tryToParseData(Poco::BinaryReader & binary_reader) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr client_localization_map_pub_;
};

class ClientLocalizationVisualizationInterface : public ReceivingInterface
{
public:
  ClientLocalizationVisualizationInterface(
    const Poco::Net::IPAddress & hostadress,
    rclcpp::Node::SharedPtr node);
  void tryToParseData(Poco::BinaryReader & binary_reader) override;

private:
  rclcpp::Publisher<bosch_locator_bridge::msg::ClientLocalizationVisualization>
  ::SharedPtr client_localization_visualization_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>
  ::SharedPtr client_localization_visualization_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>
  ::SharedPtr client_localization_visualization_scan_pub_;
};

class ClientLocalizationPoseInterface : public ReceivingInterface
{
public:
  ClientLocalizationPoseInterface(
    const Poco::Net::IPAddress & hostadress,
    rclcpp::Node::SharedPtr node);
  void tryToParseData(Poco::BinaryReader & binary_reader) override;

private:
  rclcpp::Publisher<bosch_locator_bridge::msg::ClientLocalizationPose>
  ::SharedPtr client_localization_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>
  ::SharedPtr client_localization_pose_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>
  ::SharedPtr client_localization_pose_lidar_odo_pose_pub_;
};

class ClientGlobalAlignVisualizationInterface : public ReceivingInterface
{
public:
  ClientGlobalAlignVisualizationInterface(
    const Poco::Net::IPAddress & hostadress,
    rclcpp::Node::SharedPtr node);
  void tryToParseData(Poco::BinaryReader & binary_reader) override;

private:
  rclcpp::Publisher<bosch_locator_bridge::msg::ClientGlobalAlignVisualization>
  ::SharedPtr client_global_align_visualization_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>
  ::SharedPtr client_global_align_visualization_poses_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>
  ::SharedPtr client_global_align_visualization_landmarks_poses_pub_;
};

#endif  // BOSCH_LOCATOR_BRIDGE__RECEIVING_INTERFACE_HPP_
