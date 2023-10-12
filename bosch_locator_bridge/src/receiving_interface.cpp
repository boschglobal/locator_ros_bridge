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

#include <Poco/NObserver.h>

#include <vector>

#include "receiving_interface.hpp"

#include "rosmsgs_datagram_converter.hpp"

#include "bosch_locator_bridge/ClientControlMode.h"
#include "bosch_locator_bridge/ClientRecordingVisualization.h"
#include "bosch_locator_bridge/ClientMapVisualization.h"
#include "bosch_locator_bridge/ClientLocalizationVisualization.h"
#include "bosch_locator_bridge/ClientLocalizationPose.h"
#include "bosch_locator_bridge/ClientGlobalAlignVisualization.h"


ReceivingInterface::ReceivingInterface(
  const Poco::Net::IPAddress & hostadress,
  Poco::UInt16 port,
  ros::NodeHandle & nh)
: nh_(nh), ccm_socket_(Poco::Net::SocketAddress(hostadress, port))
{
  reactor_.addEventHandler(
    ccm_socket_,
    Poco::NObserver<ReceivingInterface, Poco::Net::ReadableNotification>(
      *this, &ReceivingInterface::onReadEvent));
}

ReceivingInterface::~ReceivingInterface()
{
  reactor_.stop();
  ccm_socket_.shutdown();
}

void ReceivingInterface::onReadEvent(
  const Poco::AutoPtr<Poco::Net::ReadableNotification> & notification)
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

      size_t bytes_to_delete = 0;
      // Try to parse messages from the buffer until tryToParseData fails to parse a full message
      do {
        bytes_to_delete = tryToParseData(datagram_buffer_);
        datagram_buffer_.erase(
          datagram_buffer_.begin(),
          datagram_buffer_.begin() + bytes_to_delete);
      } while (bytes_to_delete > 0);
    }
  } catch (const std::ios_base::failure & io_failure) {
    // catching this exception is actually no error: the datagram is just not yet completely
    // transmitted could not be parsed because of that. Will automatically retry after more data
    // is available.
  } catch (...) {
    ROS_ERROR_STREAM("Caught exception in ReceivingInterface!");
  }
}

void ReceivingInterface::run()
{
  reactor_.run();
}

ClientControlModeInterface::ClientControlModeInterface(
  const Poco::Net::IPAddress & hostadress,
  const Poco::UInt16 binaryClientControlModePort,
  ros::NodeHandle & nh)
: ReceivingInterface(hostadress, binaryClientControlModePort, nh)
{
  // Setup publisher
  publishers_.push_back(
    nh.advertise<bosch_locator_bridge::ClientControlMode>("client_control_mode", 5, true));
}

size_t ClientControlModeInterface::tryToParseData(const std::vector<char> & datagram)
{
  // convert datagram to ros message
  bosch_locator_bridge::ClientControlMode client_control_mode;
  const auto parsed_bytes =
    RosMsgsDatagramConverter::convertClientControlMode2Message(
    datagram, ros::Time::now(), client_control_mode);
  if (parsed_bytes > 0) {
    // publish client control mode
    publishers_[0].publish(client_control_mode);
  }
  return parsed_bytes;
}

ClientMapMapInterface::ClientMapMapInterface(
  const Poco::Net::IPAddress & hostadress,
  const Poco::UInt16 binaryClientMapMapPort,
  ros::NodeHandle & nh)
: ReceivingInterface(hostadress, binaryClientMapMapPort, nh)
{
  // Setup publisher
  publishers_.push_back(nh.advertise<sensor_msgs::PointCloud2>("client_map_map", 5));
}

size_t ClientMapMapInterface::tryToParseData(const std::vector<char> & datagram)
{
  // convert datagram to ros message
  sensor_msgs::PointCloud2 map;
  const auto parsed_bytes =
    RosMsgsDatagramConverter::convertMapDatagram2Message(datagram, ros::Time::now(), map);
  if (parsed_bytes > 0) {
    // publish
    publishers_[0].publish(map);
  }
  return parsed_bytes;
}

ClientMapVisualizationInterface::ClientMapVisualizationInterface(
  const Poco::Net::IPAddress & hostadress,
  const Poco::UInt16 binaryClientMapVisualizationPort,
  ros::NodeHandle & nh)
: ReceivingInterface(hostadress, binaryClientMapVisualizationPort, nh)
{
  // Setup publisher
  publishers_.push_back(
    nh.advertise<bosch_locator_bridge::ClientMapVisualization>("client_map_visualization", 5));
  publishers_.push_back(
    nh.advertise<geometry_msgs::PoseStamped>("client_map_visualization/pose", 5));
  publishers_.push_back(
    nh.advertise<sensor_msgs::PointCloud2>("client_map_visualization/scan", 5));
  publishers_.push_back(
    nh.advertise<geometry_msgs::PoseArray>("client_map_visualization/path_poses", 5));
}

size_t ClientMapVisualizationInterface::tryToParseData(const std::vector<char> & datagram)
{
  // convert datagram to ros messages
  bosch_locator_bridge::ClientMapVisualization client_map_visualization;
  geometry_msgs::PoseStamped pose;
  sensor_msgs::PointCloud2 scan;
  geometry_msgs::PoseArray path_poses;

  const auto bytes_parsed = RosMsgsDatagramConverter::convertClientMapVisualizationDatagram2Message(
    datagram, client_map_visualization, pose, scan, path_poses);

  if (bytes_parsed > 0) {
    // publish
    publishers_[0].publish(client_map_visualization);
    publishers_[1].publish(pose);
    publishers_[2].publish(scan);
    publishers_[3].publish(path_poses);
  }
  return bytes_parsed;
}

ClientRecordingMapInterface::ClientRecordingMapInterface(
  const Poco::Net::IPAddress & hostadress,
  const Poco::UInt16 binaryClientRecordingMapPort,
  ros::NodeHandle & nh)
: ReceivingInterface(hostadress, binaryClientRecordingMapPort, nh)
{
  // Setup publisher
  publishers_.push_back(nh.advertise<sensor_msgs::PointCloud2>("client_recording_map", 5));
}

size_t ClientRecordingMapInterface::tryToParseData(const std::vector<char> & datagram)
{
  // convert datagram to ros message
  sensor_msgs::PointCloud2 map;
  const auto parsed_bytes =
    RosMsgsDatagramConverter::convertMapDatagram2Message(datagram, ros::Time::now(), map);
  if (parsed_bytes > 0) {
    // publish
    publishers_[0].publish(map);
  }
  return parsed_bytes;
}

ClientRecordingVisualizationInterface::ClientRecordingVisualizationInterface(
  const Poco::Net::IPAddress & hostadress,
  const Poco::UInt16 binaryClientRecordingVisualizationPort,
  ros::NodeHandle & nh)
: ReceivingInterface(hostadress, binaryClientRecordingVisualizationPort, nh)
{
  // Setup publisher
  publishers_.push_back(
    nh.advertise<bosch_locator_bridge::ClientRecordingVisualization>(
      "client_recording_visualization", 5));
  publishers_.push_back(
    nh.advertise<geometry_msgs::PoseStamped>(
      "client_recording_visualization/pose", 5));
  publishers_.push_back(
    nh.advertise<sensor_msgs::PointCloud2>(
      "client_recording_visualization/scan", 5));
  publishers_.push_back(
    nh.advertise<geometry_msgs::PoseArray>(
      "client_recording_visualization/path_poses", 5));
}

size_t ClientRecordingVisualizationInterface::tryToParseData(const std::vector<char> & datagram)
{
  // convert datagram to ros messages
  bosch_locator_bridge::ClientRecordingVisualization client_recording_visualization;
  geometry_msgs::PoseStamped pose;
  sensor_msgs::PointCloud2 scan;
  geometry_msgs::PoseArray path_poses;

  const auto parsed_bytes =
    RosMsgsDatagramConverter::convertClientRecordingVisualizationDatagram2Message(
    datagram, client_recording_visualization, pose, scan, path_poses);

  if (parsed_bytes > 0) {
    // publish
    publishers_[0].publish(client_recording_visualization);
    publishers_[1].publish(pose);
    publishers_[2].publish(scan);
    publishers_[3].publish(path_poses);
  }
  return parsed_bytes;
}

ClientLocalizationMapInterface::ClientLocalizationMapInterface(
  const Poco::Net::IPAddress & hostadress,
  const Poco::UInt16 binaryClientLocalizationMapPort,
  ros::NodeHandle & nh)
: ReceivingInterface(hostadress, binaryClientLocalizationMapPort, nh)
{
  // Setup publisher
  // enable latching, since this is usually only published once
  publishers_.push_back(nh.advertise<sensor_msgs::PointCloud2>("client_localization_map", 5, true));
}

size_t ClientLocalizationMapInterface::tryToParseData(const std::vector<char> & datagram)
{
  // convert datagram to ros message
  sensor_msgs::PointCloud2 map;
  const auto bytes_parsed =
    RosMsgsDatagramConverter::convertMapDatagram2Message(datagram, ros::Time::now(), map);
  if (bytes_parsed > 0) {
    // publish
    publishers_[0].publish(map);
  }
  return bytes_parsed;
}

ClientLocalizationVisualizationInterface::ClientLocalizationVisualizationInterface(
  const Poco::Net::IPAddress & hostadress,
  const Poco::UInt16 binaryClientLocalizationVisualizationPort,
  ros::NodeHandle & nh)
: ReceivingInterface(hostadress, binaryClientLocalizationVisualizationPort, nh)
{
  // Setup publisher
  publishers_.push_back(
    nh.advertise<bosch_locator_bridge::ClientLocalizationVisualization>(
      "client_localization_visualization", 5));
  publishers_.push_back(
    nh.advertise<geometry_msgs::PoseStamped>(
      "client_localization_visualization/pose", 5));
  publishers_.push_back(
    nh.advertise<sensor_msgs::PointCloud2>(
      "client_localization_visualization/scan", 5));
}

size_t ClientLocalizationVisualizationInterface::tryToParseData(const std::vector<char> & datagram)
{
  // convert datagram to ros messages
  bosch_locator_bridge::ClientLocalizationVisualization client_localization_visualization;
  geometry_msgs::PoseStamped pose;
  sensor_msgs::PointCloud2 scan;

  const auto bytes_parsed =
    RosMsgsDatagramConverter::convertClientLocalizationVisualizationDatagram2Message(
    datagram, client_localization_visualization, pose, scan);

  if (bytes_parsed > 0) {
    // publish
    publishers_[0].publish(client_localization_visualization);
    publishers_[1].publish(pose);
    publishers_[2].publish(scan);
  }
  return bytes_parsed;
}

ClientLocalizationPoseInterface::ClientLocalizationPoseInterface(
  const Poco::Net::IPAddress & hostadress,
  const Poco::UInt16 binaryClientLocalizationPosePort,
  ros::NodeHandle & nh)
: ReceivingInterface(hostadress, binaryClientLocalizationPosePort, nh)
{
  // Setup publisher
  publishers_.push_back(
    nh.advertise<bosch_locator_bridge::ClientLocalizationPose>(
      "client_localization_pose", 5));
  publishers_.push_back(
    nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "client_localization_pose/pose", 5));
  publishers_.push_back(
    nh.advertise<geometry_msgs::PoseStamped>(
      "client_localization_pose/lidar_odo_pose", 5));
}

size_t ClientLocalizationPoseInterface::tryToParseData(const std::vector<char> & datagram)
{
  // convert datagram to ros messages
  bosch_locator_bridge::ClientLocalizationPose client_localization_pose;
  geometry_msgs::PoseStamped pose;
  geometry_msgs::PoseWithCovarianceStamped poseWithCov;
  geometry_msgs::PoseStamped lidar_odo_pose;

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
    publishers_[0].publish(client_localization_pose);
    publishers_[1].publish(poseWithCov);
    publishers_[2].publish(lidar_odo_pose);
  }
  return bytes_parsed;
}

ClientGlobalAlignVisualizationInterface::ClientGlobalAlignVisualizationInterface(
  const Poco::Net::IPAddress & hostadress,
  const Poco::UInt16 binaryClientGlobalAlignVisualizationPort,
  ros::NodeHandle & nh)
: ReceivingInterface(hostadress, binaryClientGlobalAlignVisualizationPort, nh)
{
  // Setup publisher
  publishers_.push_back(
    nh.advertise<bosch_locator_bridge::ClientGlobalAlignVisualization>(
      "client_global_align_visualization", 5));
  publishers_.push_back(
    nh.advertise<geometry_msgs::PoseArray>(
      "client_global_align_visualization/poses", 5));
  publishers_.push_back(
    nh.advertise<geometry_msgs::PoseArray>(
      "client_global_align_visualization/landmarks/poses", 5));
}

size_t ClientGlobalAlignVisualizationInterface::tryToParseData(const std::vector<char> & datagram)
{
  // convert datagram to ros messages
  bosch_locator_bridge::ClientGlobalAlignVisualization client_global_align_visualization;
  geometry_msgs::PoseArray poses;
  geometry_msgs::PoseArray landmark_poses;

  const auto bytes_parsed =
    RosMsgsDatagramConverter::convertClientGlobalAlignVisualizationDatagram2Message(
    datagram, client_global_align_visualization, poses, landmark_poses);

  if (bytes_parsed > 0) {
    // publish
    publishers_[0].publish(client_global_align_visualization);
    publishers_[1].publish(poses);
    publishers_[2].publish(landmark_poses);
  }
  return bytes_parsed;
}

ClientExpandMapVisualizationInterface::ClientExpandMapVisualizationInterface(
  const Poco::Net::IPAddress & hostadress,
  const Poco::UInt16 binaryClientExpandMapVisualizationPort,
  ros::NodeHandle & nh)
: ReceivingInterface(hostadress, binaryClientExpandMapVisualizationPort, nh)
{
  // Setup publisher
  // enable latching, since this is usually only published once
  publishers_.push_back(
    nh.advertise<bosch_locator_bridge::ClientExpandMapVisualization>(
      "client_expandmap_visualization", 5, true));
}

size_t ClientExpandMapVisualizationInterface::tryToParseData(const std::vector<char> & datagram)
{
  // convert datagram to ros message
  bosch_locator_bridge::ClientExpandMapVisualization client_expandmap_visualization;
  const auto bytes_parsed =
    RosMsgsDatagramConverter::convertClientExpandMapVisualizationDatagram2Message(
    datagram, client_expandmap_visualization);
  if (bytes_parsed > 0) {
    // publish
    publishers_[0].publish(client_expandmap_visualization);
  }
  return bytes_parsed;
}

ClientExpandMapPriorMapInterface::ClientExpandMapPriorMapInterface(
  const Poco::Net::IPAddress & hostadress,
  const Poco::UInt16 binaryClientExpandMapPriorMapPort,
  ros::NodeHandle & nh)
: ReceivingInterface(hostadress, binaryClientExpandMapPriorMapPort, nh)
{
  // Setup publisher
  // enable latching, since this is usually only published once
  publishers_.push_back(
    nh.advertise<sensor_msgs::PointCloud2>(
      "client_expandmap_priormap", 5, true));
}

size_t ClientExpandMapPriorMapInterface::tryToParseData(const std::vector<char> & datagram)
{
  // convert datagram to ros message
  sensor_msgs::PointCloud2 map;
  const auto parsed_bytes =
    RosMsgsDatagramConverter::convertMapDatagram2Message(datagram, ros::Time::now(), map);
  if (parsed_bytes > 0) {
    // publish
    publishers_[0].publish(map);
  }
  return parsed_bytes;
}
