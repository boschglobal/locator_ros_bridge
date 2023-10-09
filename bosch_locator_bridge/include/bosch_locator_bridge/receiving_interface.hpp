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

#include <Poco/Net/NetException.h>
#include <Poco/Net/SocketNotification.h>
#include <Poco/Net/SocketReactor.h>
#include <Poco/Net/StreamSocket.h>

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "bosch_locator_bridge/msg/client_control_mode.hpp"
#include "bosch_locator_bridge/msg/client_global_align_visualization.hpp"
#include "bosch_locator_bridge/msg/client_localization_pose.hpp"
#include "bosch_locator_bridge/msg/client_localization_visualization.hpp"
#include "bosch_locator_bridge/msg/client_map_visualization.hpp"
#include "bosch_locator_bridge/msg/client_recording_visualization.hpp"
#include "bosch_locator_bridge/msg/client_expand_map_visualization.hpp"

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
  virtual size_t tryToParseData(
    const std::vector<char> & datagram_buffer,
    rclcpp::Node::SharedPtr node) = 0;

  //! Node
  rclcpp::Node::SharedPtr node_;

private:
  Poco::Net::StreamSocket ccm_socket_;
  Poco::Net::SocketReactor reactor_;
  // TODO(): use a better suited data structure (a deque?)
  std::vector<char> datagram_buffer_;
};

class ClientControlModeInterface : public ReceivingInterface
{
public:
  ClientControlModeInterface(const Poco::Net::IPAddress & hostadress,
  const Poco::UInt16 binaryClientControlModePort,
  rclcpp::Node::SharedPtr node);
  size_t tryToParseData(
    const std::vector<char> & datagram,
    rclcpp::Node::SharedPtr node) override;

private:
  rclcpp::Publisher<bosch_locator_bridge::msg::ClientControlMode>
  ::SharedPtr client_control_mode_pub_;
};

class ClientMapMapInterface : public ReceivingInterface
{
public:
  ClientMapMapInterface(const Poco::Net::IPAddress & hostadress,
  const Poco::UInt16 binaryClientMapMapPort,
  rclcpp::Node::SharedPtr node);
  size_t tryToParseData(
    const std::vector<char> & datagram,
    rclcpp::Node::SharedPtr node) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr client_map_map_pub_;
};

class ClientMapVisualizationInterface : public ReceivingInterface
{
public:
  ClientMapVisualizationInterface(
    const Poco::Net::IPAddress & hostadress,
    const Poco::UInt16 binaryClientMapVisualizationPort,
    rclcpp::Node::SharedPtr node);
  size_t tryToParseData(
    const std::vector<char> & datagram,
    rclcpp::Node::SharedPtr node) override;

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
    const Poco::UInt16 binaryClientRecordingMapPort, 
    rclcpp::Node::SharedPtr node);
  size_t tryToParseData(
    const std::vector<char> & datagram,
    rclcpp::Node::SharedPtr node) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr client_recording_map_pub_;
};

class ClientRecordingVisualizationInterface : public ReceivingInterface
{
public:
  ClientRecordingVisualizationInterface(
    const Poco::Net::IPAddress & hostadress,
    const Poco::UInt16 binaryClientRecordingVisualizationPort,
    rclcpp::Node::SharedPtr node);
  size_t tryToParseData(
    const std::vector<char> & datagram,
    rclcpp::Node::SharedPtr node) override;

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
    const Poco::UInt16 binaryClientLocalizationMapPort,
    rclcpp::Node::SharedPtr node);
  size_t tryToParseData(
    const std::vector<char> & datagram,
    rclcpp::Node::SharedPtr node) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr client_localization_map_pub_;
};

class ClientLocalizationVisualizationInterface : public ReceivingInterface
{
public:
  ClientLocalizationVisualizationInterface(
    const Poco::Net::IPAddress & hostadress,
    const Poco::UInt16 binaryClientLocalizationVisualizationPort,
    rclcpp::Node::SharedPtr node);
  size_t tryToParseData(
    const std::vector<char> & datagram,
    rclcpp::Node::SharedPtr node) override;

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
    const Poco::UInt16 binaryClientLocalizationPosePort,
    rclcpp::Node::SharedPtr node);
  size_t tryToParseData(
    const std::vector<char> & datagram,
    rclcpp::Node::SharedPtr node) override;

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
    const Poco::UInt16 binaryClientGlobalAlignVisualizationPort,
    rclcpp::Node::SharedPtr node);
  size_t tryToParseData(
    const std::vector<char> & datagram,
    rclcpp::Node::SharedPtr node) override;

private:
  rclcpp::Publisher<bosch_locator_bridge::msg::ClientGlobalAlignVisualization>
  ::SharedPtr client_global_align_visualization_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>
  ::SharedPtr client_global_align_visualization_poses_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>
  ::SharedPtr client_global_align_visualization_landmarks_poses_pub_;
};


class ClientExpandMapVisualizationInterface : public ReceivingInterface
{
public:
  ClientExpandMapVisualizationInterface(
    const Poco::Net::IPAddress& hostadress,
    const Poco::UInt16 binaryClientExpandMapVisualizationPort,
    rclcpp::Node::SharedPtr node);
  size_t tryToParseData(
    const std::vector<char>& datagram,
    rclcpp::Node::SharedPtr node) override;

private:
  rclcpp::Publisher<bosch_locator_bridge::msg::ClientExpandMapVisualization>::SharedPtr client_expand_map_visualization_pub_;

class ClientExpandMapPriorMapInterface : public ReceivingInterface
{
public:
  ClientExpandMapPriorMapInterface(
    const Poco::Net::IPAddress& hostadress,
    const Poco::UInt16 binaryClientExpandMapPriorMapPort,
    rclcpp::Node::SharedPtr node);
  size_t tryToParseData(
    const std::vector<char>& datagram,
    rclcpp::Node::SharedPtr node) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr client_expand_map_priormap_pub_;
};

#endif  // BOSCH_LOCATOR_BRIDGE__RECEIVING_INTERFACE_HPP_
