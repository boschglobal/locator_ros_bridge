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

#pragma once

#include <iostream>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include "bosch_locator_bridge/ClientControlMode.h"
#include "bosch_locator_bridge/ClientRecordingVisualization.h"
#include "bosch_locator_bridge/ClientMapVisualization.h"
#include "bosch_locator_bridge/ClientLocalizationVisualization.h"
#include "bosch_locator_bridge/ClientLocalizationPose.h"
#include "bosch_locator_bridge/ClientGlobalAlignVisualization.h"

#include <Poco/BinaryReader.h>
#include <Poco/JSON/Object.h>

#define MAP_FRAME_ID "map"
#define ODOM_FRAME_ID "odom"

/**
 * Class with static function to convert ros messages to locator's datagrams.
 */
class RosMsgsDatagramConverter
{
public:
  /**
   * @brief convertClientControlMode2Message
   * @param datagram The binary data input datagram [INPUT]
   * @param stamp ROS timestamp to assign to the message [INPUT]
   * @param client_control_mode Describes the state of the localization client [OUTPUT]
   * @return number of bytes parsed successfully
   */
  static size_t convertClientControlMode2Message(const std::vector<char>& datagram, const ros::Time& stamp,
                                                 bosch_locator_bridge::ClientControlMode& client_control_mode);

  /**
   * @brief convertMapDatagram2Message
   * @param datagram The binary data input datagram [INPUT]
   * @param stamp ROS timestamp to assign to the message [INPUT]
   * @param out_pointcloud Resulting converted map as point cloud [OUTPUT]
   * @return number of bytes parsed successfully
   */
  static size_t convertMapDatagram2Message(const std::vector<char>& datagram, const ros::Time& stamp,
                                           sensor_msgs::PointCloud2& out_pointcloud);

  /**
   * @brief convertClientGlobalAlignVisualizationDatagram2Message
   * @param datagram The binary data input datagram [INPUT]
   * @param client_global_align_visualization ClientGlobalAlignVisualization message [OUTPUT]
   * @param poses A set of poses previously visited by the platform. The platform may have observed landmarks from some
   * of these poses [OUTPUT]
   * @param landmark_poses The array of poses according to the landmarks [OUTPUT]
   * @return number of bytes parsed successfully
   */
  static size_t convertClientGlobalAlignVisualizationDatagram2Message(
      const std::vector<char>& datagram,
      bosch_locator_bridge::ClientGlobalAlignVisualization& client_global_align_visualization,
      geometry_msgs::PoseArray& poses, geometry_msgs::PoseArray& landmark_poses);

  /**
   * @brief convertClientLocalizationPoseDatagram2Message
   * @param datagram The binary data input datagram [INPUT]
   * @param client_localization_pose ClientLocalizationPose message [OUTPUT]
   * @param pose Pose message [OUTPUT]
   * @param lidar_odo_pose Pose message in odom frame [OUTPUT]
   * @param covariance array of 6 elements represents a 3x3 right triangular matrix [OUTPUT]
   * @return number of bytes parsed successfully
   */
  static size_t convertClientLocalizationPoseDatagram2Message(
      const std::vector<char>& datagram, bosch_locator_bridge::ClientLocalizationPose& client_localization_pose,
      geometry_msgs::PoseStamped& pose, double covariance[6], geometry_msgs::PoseStamped& lidar_odo_pose);

  /**
   * @brief convertClientLocalizationVisualizationDatagram2Message
   * @param datagram The binary data input datagram [INPUT]
   * @param client_localization_visualization ClientLocalizationVisualization message [OUTPUT]
   * @param pose Pose message [OUTPUT]
   * @param scan Scan message [OUTPUT]
   * @return number of bytes parsed successfully
   */
  static size_t convertClientLocalizationVisualizationDatagram2Message(
      const std::vector<char>& datagram,
      bosch_locator_bridge::ClientLocalizationVisualization& client_localization_visualization,
      geometry_msgs::PoseStamped& pose, sensor_msgs::PointCloud2& scan);

  /**
   * @brief convertClientMapVisualizationDatagram2Message
   * @param datagram The binary data input datagram [INPUT]
   * @param client_map_visualization ClientMapVisualization message [OUTPUT]
   * @param pose Pose message [OUTPUT]
   * @param scan Scan message [OUTPUT]
   * @param path_poses PathPoses message [OUTPUT]
   * @return number of bytes parsed successfully
   */
  static size_t convertClientMapVisualizationDatagram2Message(
      const std::vector<char>& datagram, bosch_locator_bridge::ClientMapVisualization& client_map_visualization,
      geometry_msgs::PoseStamped& pose, sensor_msgs::PointCloud2& scan, geometry_msgs::PoseArray& path_poses);

  /**
   * @brief convertClientRecordingVisualizationDatagram2Message
   * @param datagram The binary data input datagram [INPUT]
   * @param client_recording_visualization ClientRecordingVisualization message [OUTPUT]
   * @param pose Pose message [OUTPUT]
   * @param scan Scan message [OUTPUT]
   * @param path_poses PathPoses message [OUTPUT]
   * @return number of bytes parsed successfully
   */
  static size_t convertClientRecordingVisualizationDatagram2Message(
      const std::vector<char>& datagram,
      bosch_locator_bridge::ClientRecordingVisualization& client_recording_visualization,
      geometry_msgs::PoseStamped& pose, sensor_msgs::PointCloud2& scan, geometry_msgs::PoseArray& path_poses);

  /**
   * @brief convertPose2DDoubleDatagram2Message Takes a binary_reader with a DOUBLE precision pose datagram coming next
   * and converts to a geometry_msgs::Pose
   * @param binary_reader The binary_reader pointing to a DOUBLE precision pose datagram
   * @param pose Converted pose [OUTPUT]
   * @return number of bytes parsed successfully
   */
  static size_t convertPose2DDoubleDatagram2Message(Poco::BinaryReader& binary_reader, geometry_msgs::Pose& pose);

  /**
   * @brief convertPose2DSingleDatagram2Message Takes a binary_reader with a SINGLE precision pose datagram coming next
   * and converts to a geometry_msgs::Pose
   * @param binary_reader The binary_reader pointing to a SINGLE precision pose datagram
   * @param pose Converted pose [OUTPUT]
   * @return number of bytes parsed successfully
   */
  static size_t convertPose2DSingleDatagram2Message(Poco::BinaryReader& binary_reader, geometry_msgs::Pose& pose);

  /**
   * @brief convertLaserScan2DataGram Converts a sensor_msgs::LaserScan message from ros and converts
   *                                  it to the datagram structure required for the binary interface of the locator.
   * @param msg The laser scan message
   * @param scan_num The current scan number
   * @param scan_time Time between scans [seconds], if not specified in scan message
   * @return The data shaped into the datagram structure required by the locator
   */
  static Poco::Buffer<char> convertLaserScan2DataGram(const sensor_msgs::LaserScan& msg, size_t scan_num, float scan_time = 0.0f);

  /**
   * @brief convertOdometry2DataGram Converts a nav_msgs::Odometry message from ros and converts
   *                                  it to the datagram structure required for the binary interface of the locator.
   * @param msg The odometry message
   * @param odom_num The current odomerty observation number
   * @return The data shaped into the datagram structure required by the locator
   */
  static Poco::Buffer<char> convertOdometry2DataGram(const nav_msgs::Odometry& msg, size_t odom_num);

  static Poco::JSON::Object makePose2d(const geometry_msgs::Pose2D& pose);

private:
  static size_t convertMapDatagram2Message(Poco::BinaryReader& binary_reader, const ros::Time& stamp,
                                           sensor_msgs::PointCloud2& out_pointcloud);
  static size_t convertMapDatagram2PointCloud(Poco::BinaryReader& binary_reader,
                                              pcl::PointCloud<pcl::PointXYZRGB>& out_pointcloud);
  static void colorizePointCloud(pcl::PointCloud<pcl::PointXYZRGB>& point_cloud,
                                 const std::vector<uint64_t>& sensor_offsets);
  static size_t discardExtension(Poco::BinaryReader& binary_reader);
  static void readIntensities(Poco::BinaryReader& binary_reader);
  static std::vector<uint64_t> readSensorOffsets(Poco::BinaryReader& binary_reader);

  /// clamp scan data to specified range
  static float clamp_range(float r, float min, float max)
  {
    return std::min({ std::max({ r, min }), max });
  };
};
