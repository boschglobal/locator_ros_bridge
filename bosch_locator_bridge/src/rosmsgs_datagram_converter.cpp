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

#include "rosmsgs_datagram_converter.hpp"

#include <Poco/BinaryWriter.h>
#include <Poco/MemoryStream.h>

#include <fstream>
#include <string>
#include <vector>

#include "pcl_conversions/pcl_conversions.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"

#ifdef ROS_GALACTIC
  #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
  #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif

#include "bosch_locator_bridge/msg/client_global_align_landmark_observation_notice.hpp"
#include "bosch_locator_bridge/msg/client_global_align_landmark_visualization_information.hpp"

namespace {
size_t discardExtension(Poco::BinaryReader& binary_reader)
{
  uint32_t extensionSize {0u};
  binary_reader >> extensionSize;

  const auto bytesToDiscard = extensionSize - 4u;
  std::vector<char> dataToDiscard(bytesToDiscard);
  
  binary_reader.readRaw(dataToDiscard.data(), bytesToDiscard);

  return extensionSize;
}
}



size_t
RosMsgsDatagramConverter::convertClientControlMode2Message(
  const std::vector<char> & datagram, const rclcpp::Time & stamp,
  bosch_locator_bridge::msg::ClientControlMode & client_control_mode)
{
  if (datagram.size() < 4) {
    return 0;
  }
  client_control_mode.stamp = stamp;

  uint32_t client_control_mode_datagram = *reinterpret_cast<const uint32_t *>(&(datagram[0]));

  client_control_mode.mask_state = static_cast<uint8_t>(client_control_mode_datagram & 0b111);
  client_control_mode.alignment_state =
    static_cast<uint8_t>((client_control_mode_datagram >> 3) & 0b111);
  client_control_mode.recording_state =
    static_cast<uint8_t>((client_control_mode_datagram >> 6) & 0b111);
  client_control_mode.localization_state =
    static_cast<uint8_t>((client_control_mode_datagram >> 9) & 0b111);
  client_control_mode.map_state =
    static_cast<uint8_t>((client_control_mode_datagram >> 12) & 0b111);
  client_control_mode.visual_recording_state =
    static_cast<uint8_t>((client_control_mode_datagram >> 15) & 0b111);
  return 4;
}

size_t RosMsgsDatagramConverter::convertMapDatagram2Message(
  const std::vector<char> & datagram, const rclcpp::Time & stamp,
  sensor_msgs::msg::PointCloud2 & out_pointcloud)
{
  Poco::MemoryInputStream inStream(&datagram[0], datagram.size());
  auto binary_reader = Poco::BinaryReader(inStream, Poco::BinaryReader::LITTLE_ENDIAN_BYTE_ORDER);
  binary_reader.setExceptions(
    std::ifstream::failbit | std::ifstream::badbit |
    std::ifstream::eofbit);
  return convertMapDatagram2Message(binary_reader, stamp, out_pointcloud);
}

size_t RosMsgsDatagramConverter::convertMapDatagram2Message(
  Poco::BinaryReader & binary_reader, const rclcpp::Time & stamp,
  sensor_msgs::msg::PointCloud2 & out_pointcloud)
{
  // Convert datagram to point cloud
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  uint32_t map_length;
  binary_reader >> map_length;
  size_t bytes_parsed = 4;

  for (unsigned int i = 0; i < map_length; i++) {
    pcl::PointXYZ pt(0.f, 0.f, 0.f);
    binary_reader >> pt.x >> pt.y;
    bytes_parsed += 8;
    point_cloud.push_back(pt);
  }

  // Discard the extension part of the datagram
  bytes_parsed += discardExtension(binary_reader);

  // Create message
  pcl::toROSMsg(point_cloud, out_pointcloud);
  out_pointcloud.header.frame_id = MAP_FRAME_ID;
  out_pointcloud.header.stamp = stamp;

  return bytes_parsed;
}

size_t RosMsgsDatagramConverter::convertMapDatagram2PointCloud(
  Poco::BinaryReader & binary_reader,
  pcl::PointCloud<pcl::PointXYZRGB> & out_pointcloud)
{
  // Convert datagram to point cloud
  uint32_t map_length;
  binary_reader >> map_length;
  size_t bytes_parsed = 4;

  for (unsigned int i = 0; i < map_length; i++) {
    pcl::PointXYZRGB pt(0.f, 0.f, 0.f);
    binary_reader >> pt.x >> pt.y;
    bytes_parsed += 8;
    out_pointcloud.push_back(pt);
  }

  return bytes_parsed;
}

size_t RosMsgsDatagramConverter::convertClientGlobalAlignVisualizationDatagram2Message(
  const std::vector<char> & datagram,
  bosch_locator_bridge::msg::ClientGlobalAlignVisualization & client_global_align_visualization,
  geometry_msgs::msg::PoseArray & poses, geometry_msgs::msg::PoseArray & landmark_poses)
{
  Poco::MemoryInputStream inStream(&datagram[0], datagram.size());
  auto binary_reader = Poco::BinaryReader(inStream, Poco::BinaryReader::LITTLE_ENDIAN_BYTE_ORDER);
  binary_reader.setExceptions(
    std::ifstream::failbit | std::ifstream::badbit |
    std::ifstream::eofbit);

  double stamp;
  binary_reader >> stamp;
  client_global_align_visualization.timestamp = rclcpp::Time(stamp * 1e9);
  binary_reader >> client_global_align_visualization.visualization_id;

  // Retrieve poses
  uint32_t num_poses;
  binary_reader >> num_poses;
  poses.header.stamp = client_global_align_visualization.timestamp;
  poses.header.frame_id = MAP_FRAME_ID;
  for (unsigned int i = 0; i < num_poses; i++) {
    geometry_msgs::msg::Pose pose;
    convertPose2DSingleDatagram2Message(binary_reader, pose);
    poses.poses.push_back(pose);
  }

  // retrieve landmarks
  uint32_t num_landmarks;
  binary_reader >> num_landmarks;
  landmark_poses.header.stamp = client_global_align_visualization.timestamp;
  landmark_poses.header.frame_id = MAP_FRAME_ID;
  for (unsigned int i = 0; i < num_landmarks; i++) {
    geometry_msgs::msg::Pose pose;
    convertPose2DSingleDatagram2Message(binary_reader, pose);
    landmark_poses.poses.push_back(pose);

    bosch_locator_bridge::msg::ClientGlobalAlignLandmarkVisualizationInformation vis_info;
    binary_reader >> vis_info.type;
    uint8_t has_orientation;
    binary_reader >> has_orientation;
    vis_info.has_orientation = static_cast<bool>(has_orientation);

    uint32_t name_length;
    binary_reader >> name_length;
    std::vector<char> name(name_length);
    for (unsigned int j = 0; j < name_length; j++) {
      binary_reader >> name[j];
    }
    vis_info.name = std::string(name.begin(), name.end());

    client_global_align_visualization.landmarks.push_back(vis_info);
  }

  // retrieve observations
  uint32_t num_observations;
  binary_reader >> num_observations;
  for (unsigned int i = 0; i < num_observations; i++) {
    bosch_locator_bridge::msg::ClientGlobalAlignLandmarkObservationNotice notice;
    binary_reader >> notice.pose_index >> notice.landmark_index;
    client_global_align_visualization.observations.push_back(notice);
  }
  return datagram.size() - binary_reader.available();
}

size_t RosMsgsDatagramConverter::convertClientLocalizationPoseDatagram2Message(
  const std::vector<char> & datagram,
  bosch_locator_bridge::msg::ClientLocalizationPose & client_localization_pose,
  geometry_msgs::msg::PoseStamped & pose, double covariance[6],
  geometry_msgs::msg::PoseStamped & lidar_odo_pose)
{
  Poco::MemoryInputStream inStream(&datagram[0], datagram.size());
  auto binary_reader = Poco::BinaryReader(inStream, Poco::BinaryReader::LITTLE_ENDIAN_BYTE_ORDER);
  binary_reader.setExceptions(
    std::ifstream::failbit | std::ifstream::badbit |
    std::ifstream::eofbit);

  double age, stamp;
  binary_reader >> age >> stamp;
  client_localization_pose.age = rclcpp::Duration::from_seconds(age);
  client_localization_pose.timestamp = rclcpp::Time(stamp * 1e9);
  binary_reader >> client_localization_pose.unique_id >> client_localization_pose.state;

  // Get pose
  pose.header.stamp = client_localization_pose.timestamp;
  pose.header.frame_id = MAP_FRAME_ID;
  convertPose2DDoubleDatagram2Message(binary_reader, pose.pose);

  for (int i = 0; i < 6; ++i) {
    binary_reader >> covariance[i];
  }

  // The following messages are redundant information of the pose and not yet required.
  double poseZ, quaternion_w, quaternion_x, quaternion_y, quaternion_z;
  binary_reader >> poseZ >> quaternion_w >> quaternion_x >> quaternion_y >> quaternion_z;

  binary_reader >> client_localization_pose.epoch;

  // Get lidar-odo-pose
  lidar_odo_pose.header.stamp = client_localization_pose.timestamp;
  // TODO(): here we might have an different reference frame (arbitrary according to API)
  lidar_odo_pose.header.frame_id = ODOM_FRAME_ID;
  convertPose2DDoubleDatagram2Message(binary_reader, lidar_odo_pose.pose);

  return datagram.size() - binary_reader.available();
}

size_t RosMsgsDatagramConverter::convertClientLocalizationVisualizationDatagram2Message(
  const std::vector<char> & datagram,
  bosch_locator_bridge::msg::ClientLocalizationVisualization & client_localization_visualization,
  geometry_msgs::msg::PoseStamped & pose, sensor_msgs::msg::PointCloud2 & scan)
{
  Poco::MemoryInputStream inStream(&datagram[0], datagram.size());
  auto binary_reader = Poco::BinaryReader(inStream, Poco::BinaryReader::LITTLE_ENDIAN_BYTE_ORDER);
  binary_reader.setExceptions(
    std::ifstream::failbit | std::ifstream::badbit |
    std::ifstream::eofbit);

  double stamp;
  binary_reader >> stamp;
  client_localization_visualization.timestamp = rclcpp::Time(stamp * 1e9);
  binary_reader >> client_localization_visualization.unique_id >>
  client_localization_visualization.loc_state;

  // Get pose
  pose.header.stamp = client_localization_visualization.timestamp;
  pose.header.frame_id = MAP_FRAME_ID;
  convertPose2DDoubleDatagram2Message(binary_reader, pose.pose);

  binary_reader >> client_localization_visualization.delay;
  pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
  convertMapDatagram2PointCloud(binary_reader, point_cloud);

  // Get sensor offsets and intensities
  std::vector<uint64_t> sensor_offsets = readSensorOffsets(binary_reader);
  readIntensities(binary_reader);

  // Discard the extension part of the datagram
  discardExtension(binary_reader);

  // Use sensor offsets to colorize point cloud
  colorizePointCloud(point_cloud, sensor_offsets);

  // Create point cloud message
  pcl::toROSMsg(point_cloud, scan);
  scan.header.frame_id = MAP_FRAME_ID;
  scan.header.stamp = client_localization_visualization.timestamp;

  return datagram.size() - binary_reader.available();
}

size_t RosMsgsDatagramConverter::convertClientMapVisualizationDatagram2Message(
  const std::vector<char> & datagram,
  bosch_locator_bridge::msg::ClientMapVisualization & client_map_visualization,
  geometry_msgs::msg::PoseStamped & pose, sensor_msgs::msg::PointCloud2 & scan,
  geometry_msgs::msg::PoseArray & path_poses)
{
  Poco::MemoryInputStream inStream(&datagram[0], datagram.size());
  auto binary_reader = Poco::BinaryReader(inStream, Poco::BinaryReader::LITTLE_ENDIAN_BYTE_ORDER);
  binary_reader.setExceptions(
    std::ifstream::failbit | std::ifstream::badbit |
    std::ifstream::eofbit);
  double stamp;
  binary_reader >> stamp;
  client_map_visualization.timestamp = rclcpp::Time(stamp * 1e9);
  binary_reader >> client_map_visualization.visualization_id >> client_map_visualization.status;

  // Get pose
  pose.header.stamp = client_map_visualization.timestamp;
  pose.header.frame_id = MAP_FRAME_ID;
  convertPose2DDoubleDatagram2Message(binary_reader, pose.pose);

  binary_reader >> client_map_visualization.distance_to_last_lc >> client_map_visualization.delay >>
  client_map_visualization.progress;
  pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
  convertMapDatagram2PointCloud(binary_reader, point_cloud);

  // Get path poses
  path_poses.header.stamp = client_map_visualization.timestamp;
  path_poses.header.frame_id = MAP_FRAME_ID;
  uint32_t path_poses_length;
  binary_reader >> path_poses_length;

  for (unsigned int i = 0; i < path_poses_length; i++) {
    geometry_msgs::msg::Pose nextPose;
    convertPose2DSingleDatagram2Message(binary_reader, nextPose);
    path_poses.poses.push_back(nextPose);
  }

  // Get path types
  uint32_t path_types_length;
  binary_reader >> path_types_length;

  client_map_visualization.path_types.resize(path_types_length);
  for (unsigned int i = 0; i < path_types_length; i++) {
    binary_reader >> client_map_visualization.path_types[i];
  }

  // Get sensor offsets and read intensities
  std::vector<uint64_t> sensor_offsets = readSensorOffsets(binary_reader);
  readIntensities(binary_reader);

  // Discard the extension part of the datagram
  discardExtension(binary_reader);

  // Use sensor offsets to colorize point cloud
  colorizePointCloud(point_cloud, sensor_offsets);

  // Create point cloud message
  pcl::toROSMsg(point_cloud, scan);
  scan.header.frame_id = MAP_FRAME_ID;
  scan.header.stamp = client_map_visualization.timestamp;

  return datagram.size() - binary_reader.available();
}

size_t RosMsgsDatagramConverter::convertClientRecordingVisualizationDatagram2Message(
  const std::vector<char> & datagram,
  bosch_locator_bridge::msg::ClientRecordingVisualization & client_recording_visualization,
  geometry_msgs::msg::PoseStamped & pose,
  sensor_msgs::msg::PointCloud2 & scan, geometry_msgs::msg::PoseArray & path_poses)
{
  Poco::MemoryInputStream inStream(&datagram[0], datagram.size());
  auto binary_reader = Poco::BinaryReader(inStream, Poco::BinaryReader::LITTLE_ENDIAN_BYTE_ORDER);
  binary_reader.setExceptions(
    std::ifstream::failbit | std::ifstream::badbit |
    std::ifstream::eofbit);

  double stamp;
  binary_reader >> stamp;
  client_recording_visualization.timestamp = rclcpp::Time(stamp * 1e9);
  binary_reader >> client_recording_visualization.visualization_id >>
  client_recording_visualization.status;

  // Get pose
  pose.header.stamp = client_recording_visualization.timestamp;
  pose.header.frame_id = MAP_FRAME_ID;
  convertPose2DDoubleDatagram2Message(binary_reader, pose.pose);

  binary_reader >> client_recording_visualization.distance_to_last_lc >>
  client_recording_visualization.delay >>
  client_recording_visualization.progress;
  pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
  convertMapDatagram2PointCloud(binary_reader, point_cloud);

  // Get path poses
  path_poses.header.stamp = client_recording_visualization.timestamp;
  path_poses.header.frame_id = MAP_FRAME_ID;
  uint32_t path_poses_length;
  binary_reader >> path_poses_length;

  for (unsigned int i = 0; i < path_poses_length; i++) {
    geometry_msgs::msg::Pose nextPose;
    convertPose2DSingleDatagram2Message(binary_reader, nextPose);
    path_poses.poses.push_back(nextPose);
  }

  // Get path types
  uint32_t path_types_length;
  binary_reader >> path_types_length;

  client_recording_visualization.path_types.resize(path_types_length);
  for (unsigned int i = 0; i < path_types_length; i++) {
    binary_reader >> client_recording_visualization.path_types[i];
  }

  // Get sensor offsets and read intensities
  std::vector<uint64_t> sensor_offsets = readSensorOffsets(binary_reader);
  readIntensities(binary_reader);

  // Discard the extension part of the datagram
  discardExtension(binary_reader);

  // Use sensor offsets to colorize point cloud
  colorizePointCloud(point_cloud, sensor_offsets);

  // Create point cloud message
  pcl::toROSMsg(point_cloud, scan);
  scan.header.frame_id = MAP_FRAME_ID;
  scan.header.stamp = client_recording_visualization.timestamp;

  return datagram.size() - binary_reader.available();
}

size_t RosMsgsDatagramConverter::convertPose2DDoubleDatagram2Message(
  Poco::BinaryReader & binary_reader,
  geometry_msgs::msg::Pose & pose)
{
  double pose_x, pose_y, pose_yaw;
  binary_reader >> pose_x >> pose_y >> pose_yaw;
  pose.position.x = pose_x;
  pose.position.y = pose_y;
  pose.position.z = 0;
  tf2::Quaternion pose_quaternion;
  pose_quaternion.setRPY(0, 0, pose_yaw);
  pose.orientation = tf2::toMsg(pose_quaternion);
  return 3 * 8;
}

size_t RosMsgsDatagramConverter::convertPose2DSingleDatagram2Message(
  Poco::BinaryReader & binary_reader,
  geometry_msgs::msg::Pose & pose)
{
  float pose_x, pose_y, pose_yaw;
  binary_reader >> pose_x >> pose_y >> pose_yaw;
  pose.position.x = pose_x;
  pose.position.y = pose_y;
  pose.position.z = 0;
  tf2::Quaternion pose_quaternion;
  pose_quaternion.setRPY(0, 0, pose_yaw);
  pose.orientation = tf2::toMsg(pose_quaternion);

  return 3 * 4;
}

Poco::Buffer<char> RosMsgsDatagramConverter::convertLaserScan2DataGram(
  const sensor_msgs::msg::LaserScan::SharedPtr msg,
  size_t scan_num, rclcpp::Node::SharedPtr node)
{
  // convert the ROS message to a locator ClientSensorLaserDatagram
  const size_t resulting_msg_size = 2 +  // scanNum
    5 * 8 +                  // time_start, uniqueId, duration_beam, duration_scan, duration_rotate
    6 * 4 +                  // numBeams, angleStart, angleEnd, angleInc, minRange, maxRange
    4 +                                  // ranges->length
    msg->ranges.size() * 4 +             // ranges->elements
    1 +                                  // hasIntensities
    2 * 4 +                              // minIntensity, maxIntensity
    4 +                                  // intensities->length
    msg->intensities.size() * 4;         // intensities->elements

  Poco::Buffer<char> buffer(resulting_msg_size);
  Poco::MemoryBinaryWriter writer(buffer,
    Poco::BinaryWriter::StreamByteOrder::LITTLE_ENDIAN_BYTE_ORDER);

  // scanNum
  writer << static_cast<uint16_t>(scan_num);
  // time_start
  writer << (static_cast<double>(msg->header.stamp.sec) +
  1e-9 * static_cast<double>(msg->header.stamp.nanosec));
  // uniqueId
  writer << static_cast<uint64_t>(0);
  // duration_beam
  double duration_beam =
    static_cast<double>(msg->time_increment != 0.0f ? fabs(msg->time_increment) :
    (msg->angle_max - msg->angle_min) * msg->scan_time / (2.0 * M_PI * (msg->ranges.size() - 1)));
  writer << duration_beam;
  // duration_scan
  writer << duration_beam * static_cast<double>(msg->ranges.size() - 1);
  // duration_rotate (has to be > 0 for motion correction of scans)
  writer << static_cast<double>(msg->scan_time);
  // numBeams
  writer << static_cast<uint32_t>(msg->ranges.size());
  // angleStart
  writer << static_cast<float>(msg->angle_min);
  // angleEnd
  writer << static_cast<float>(msg->angle_max);
  // angleInc
  writer << static_cast<float>(msg->angle_increment);
  // minRange
  writer << static_cast<float>(msg->range_min);
  // maxRange
  writer << static_cast<float>(msg->range_max);
  // ranges.length
  writer << static_cast<uint32_t>(msg->ranges.size());
  // ranges.elements
  for (const auto r : msg->ranges) {
    writer << static_cast<float>(std::isnan(r) ? -1e4f : clamp_range(r, -1e4f, 1e4f));
  }
  writer << static_cast<char>(!msg->intensities.empty());
  // FIXME intensities ranges not in sensor_msgs/LaserScan. Where to get from?
  // minIntensity
  writer << static_cast<float>(0.f);
  // maxIntensity
  writer << static_cast<float>(1.f);
  // intensitites.length
  writer << static_cast<uint32_t>(msg->intensities.size());
  // intensities.elements
  for (const auto intensity : msg->intensities) {
    writer << static_cast<float>(intensity);
  }
  writer.flush();

  RCLCPP_ERROR_STREAM_EXPRESSION(
    node->get_logger(),
    resulting_msg_size != buffer.size(), "convertLaserScan2DataGram: message size mismatch!");

  return buffer;
}

Poco::Buffer<char> RosMsgsDatagramConverter::convertOdometry2DataGram(
  const nav_msgs::msg::Odometry::SharedPtr msg, size_t odom_num, rclcpp::Node::SharedPtr node)
{
  // convert the ROS message to a locator ClientSensorLaserDatagram
  const size_t resulting_msg_size = 8 +      // timestamp
    4 +                                      // odomNum
    8 +                                      // epoch
    6 * 8 +                                  // x,y,yaw,v_x,v_y,omega
    1;                                       // velocitySet

  Poco::Buffer<char> buffer(resulting_msg_size);
  Poco::MemoryBinaryWriter writer(buffer,
    Poco::BinaryWriter::StreamByteOrder::LITTLE_ENDIAN_BYTE_ORDER);

  // timestamp
  writer << (static_cast<double>(msg->header.stamp.sec) +
  1e-9 * static_cast<double>(msg->header.stamp.nanosec));
  // odomNum
  writer << static_cast<uint32_t>(odom_num);
  // epoch... no information available from nav_msgs::msg::Odometry. Keep constant at 0.
  writer << static_cast<uint64_t>(0);

  // Extract yaw from quaternion
  tf2::Quaternion quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 mat(quaternion);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  // Write pose
  writer << msg->pose.pose.position.x << msg->pose.pose.position.y << yaw;

  // Write velocity
  writer << msg->twist.twist.linear.x << msg->twist.twist.linear.y << msg->twist.twist.angular.z;

  // velocity
  // (available in the nav_msgs::msg::Odometry message though
  // its reliability cannot be judged at this stage, activate)
  writer << static_cast<char>(true);

  writer.flush();

  RCLCPP_ERROR_STREAM_EXPRESSION(
    node->get_logger(),
    resulting_msg_size != buffer.size(), "convertOdometry2DataGram: message size mismatch!");

  return buffer;
}

Poco::JSON::Object RosMsgsDatagramConverter::makePose2d(const geometry_msgs::msg::Pose2D & pose)
{
  Poco::JSON::Object obj;
  obj.set("x", pose.x);
  obj.set("y", pose.y);
  obj.set("a", pose.theta);
  return obj;
}

void RosMsgsDatagramConverter::colorizePointCloud(
  pcl::PointCloud<pcl::PointXYZRGB> & point_cloud,
  const std::vector<uint64_t> & sensor_offsets)
{
  for (unsigned int i = sensor_offsets[0];
    i < (sensor_offsets.size() == 2 ? sensor_offsets[1] : point_cloud.size()); i++)
  {
    point_cloud[i].r = 239;
    point_cloud[i].g = 41;
    point_cloud[i].b = 41;
  }
  if (sensor_offsets.size() == 2) {
    for (unsigned int i = sensor_offsets[1]; i < point_cloud.size(); i++) {
      point_cloud[i].r = 114;
      point_cloud[i].g = 159;
      point_cloud[i].b = 207;
    }
  }
}

void RosMsgsDatagramConverter::readIntensities(Poco::BinaryReader & binary_reader)
{
  bool has_intensities;
  float min_intensity, max_intensity;
  uint32_t intensities_length;
  binary_reader >> has_intensities >> min_intensity >> max_intensity >> intensities_length;

  std::vector<float> intensities(intensities_length);
  for (unsigned int i = 0; i < intensities_length; i++) {
    binary_reader >> intensities[i];
  }
}

std::vector<uint64_t> RosMsgsDatagramConverter::readSensorOffsets(
  Poco::BinaryReader & binary_reader)
{
  uint32_t sensor_offsets_length;
  binary_reader >> sensor_offsets_length;

  std::vector<uint64_t> sensor_offsets(sensor_offsets_length);
  for (unsigned int i = 0; i < sensor_offsets_length; i++) {
    binary_reader >> sensor_offsets[i];
  }

  return sensor_offsets;
}
