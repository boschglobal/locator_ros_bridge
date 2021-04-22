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

#include "bosch_locator_bridge/ClientGlobalAlignLandmarkObservationNotice.h"
#include "bosch_locator_bridge/ClientGlobalAlignLandmarkVisualizationInformation.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

#include <pcl_conversions/pcl_conversions.h>

#include <Poco/BinaryWriter.h>
#include <Poco/MemoryStream.h>

#include <fstream>

size_t
RosMsgsDatagramConverter::convertClientControlMode2Message(const std::vector<char>& datagram, const ros::Time& stamp,
                                                           bosch_locator_bridge::ClientControlMode& client_control_mode)
{
  if (datagram.size() < 4)
  {
    return 0;
  }
  client_control_mode.stamp = stamp;

  uint32_t client_control_mode_datagram = *reinterpret_cast<const uint32_t*>(&(datagram[0]));

  client_control_mode.mask_state = static_cast<uint8_t>(client_control_mode_datagram & 0b111);
  client_control_mode.alignment_state = static_cast<uint8_t>((client_control_mode_datagram >> 3) & 0b111);
  client_control_mode.recording_state = static_cast<uint8_t>((client_control_mode_datagram >> 6) & 0b111);
  client_control_mode.localization_state = static_cast<uint8_t>((client_control_mode_datagram >> 9) & 0b111);
  client_control_mode.map_state = static_cast<uint8_t>((client_control_mode_datagram >> 12) & 0b111);
  client_control_mode.visual_recording_state = static_cast<uint8_t>((client_control_mode_datagram >> 15) & 0b111);
  return 4;
}

size_t RosMsgsDatagramConverter::convertMapDatagram2Message(const std::vector<char>& datagram, const ros::Time& stamp,
                                                            sensor_msgs::PointCloud2& out_pointcloud)
{
  Poco::MemoryInputStream inStream(&datagram[0], datagram.size());
  auto binary_reader = Poco::BinaryReader(inStream, Poco::BinaryReader::LITTLE_ENDIAN_BYTE_ORDER);
  binary_reader.setExceptions(std::ifstream::failbit | std::ifstream::badbit | std::ifstream::eofbit);
  return convertMapDatagram2Message(binary_reader, stamp, out_pointcloud);
}

size_t RosMsgsDatagramConverter::convertMapDatagram2Message(Poco::BinaryReader& binary_reader, const ros::Time& stamp,
                                                            sensor_msgs::PointCloud2& out_pointcloud)
{
  // Convert datagram to point cloud
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  uint32_t map_length;
  binary_reader >> map_length;
  size_t bytes_parsed = 4;

  for (unsigned int i = 0; i < map_length; i++)
  {
    pcl::PointXYZ pt(0.f, 0.f, 0.f);
    binary_reader >> pt.x >> pt.y;
    bytes_parsed += 8;
    point_cloud.push_back(pt);
  }
  // Create message
  pcl::toROSMsg(point_cloud, out_pointcloud);
  out_pointcloud.header.frame_id = MAP_FRAME_ID;
  out_pointcloud.header.stamp = stamp;

  return bytes_parsed;
}

size_t RosMsgsDatagramConverter::convertClientGlobalAlignVisualizationDatagram2Message(
    const std::vector<char>& datagram,
    bosch_locator_bridge::ClientGlobalAlignVisualization& client_global_align_visualization,
    geometry_msgs::PoseArray& poses, geometry_msgs::PoseArray& landmark_poses)
{
  Poco::MemoryInputStream inStream(&datagram[0], datagram.size());
  auto binary_reader = Poco::BinaryReader(inStream, Poco::BinaryReader::LITTLE_ENDIAN_BYTE_ORDER);
  binary_reader.setExceptions(std::ifstream::failbit | std::ifstream::badbit | std::ifstream::eofbit);

  double stamp;
  binary_reader >> stamp;
  client_global_align_visualization.timestamp = ros::Time(stamp);
  binary_reader >> client_global_align_visualization.visualization_id;

  // Retrieve poses
  uint32_t num_poses;
  binary_reader >> num_poses;
  poses.header.stamp = client_global_align_visualization.timestamp;
  poses.header.frame_id = MAP_FRAME_ID;
  for (unsigned int i = 0; i < num_poses; i++)
  {
    geometry_msgs::Pose pose;
    convertPose2DSingleDatagram2Message(binary_reader, pose);
    poses.poses.push_back(pose);
  }

  // retrieve landmarks
  uint32_t num_landmarks;
  binary_reader >> num_landmarks;
  landmark_poses.header.stamp = client_global_align_visualization.timestamp;
  landmark_poses.header.frame_id = MAP_FRAME_ID;
  for (unsigned int i = 0; i < num_landmarks; i++)
  {
    geometry_msgs::Pose pose;
    convertPose2DSingleDatagram2Message(binary_reader, pose);
    landmark_poses.poses.push_back(pose);

    bosch_locator_bridge::ClientGlobalAlignLandmarkVisualizationInformation vis_info;
    binary_reader >> vis_info.type;
    uint8_t has_orientation;
    binary_reader >> has_orientation;
    vis_info.has_orientation = static_cast<bool>(has_orientation);

    uint32_t name_length;
    binary_reader >> name_length;
    std::vector<char> name(name_length);
    for (unsigned int j = 0; j < name_length; j++)
    {
      binary_reader >> name[j];
    }
    vis_info.name = std::string(name.begin(), name.end());

    client_global_align_visualization.landmarks.push_back(vis_info);
  }

  // retrieve observations
  uint32_t num_observations;
  binary_reader >> num_observations;
  for (unsigned int i = 0; i < num_observations; i++)
  {
    bosch_locator_bridge::ClientGlobalAlignLandmarkObservationNotice notice;
    binary_reader >> notice.pose_index >> notice.landmark_index;
    client_global_align_visualization.observations.push_back(notice);
  }
  return datagram.size() - binary_reader.available();
}

size_t RosMsgsDatagramConverter::convertClientLocalizationPoseDatagram2Message(
    const std::vector<char>& datagram, bosch_locator_bridge::ClientLocalizationPose& client_localization_pose,
    geometry_msgs::PoseStamped& pose, double covariance[6], geometry_msgs::PoseStamped& lidar_odo_pose)
{
  Poco::MemoryInputStream inStream(&datagram[0], datagram.size());
  auto binary_reader = Poco::BinaryReader(inStream, Poco::BinaryReader::LITTLE_ENDIAN_BYTE_ORDER);
  binary_reader.setExceptions(std::ifstream::failbit | std::ifstream::badbit | std::ifstream::eofbit);

  double age, stamp;
  binary_reader >> age >> stamp;
  client_localization_pose.age = ros::Duration(age);
  client_localization_pose.timestamp = ros::Time(stamp);
  binary_reader >> client_localization_pose.unique_id >> client_localization_pose.state;

  // Get pose
  pose.header.stamp = client_localization_pose.timestamp;
  pose.header.frame_id = MAP_FRAME_ID;
  convertPose2DDoubleDatagram2Message(binary_reader, pose.pose);

  for (int i = 0; i < 6; ++i)
  {
    binary_reader >> covariance[i];
  }

  // The following messages are redundant information of the pose and not yet required.
  double poseZ, quaternion_w, quaternion_x, quaternion_y, quaternion_z;
  binary_reader >> poseZ >> quaternion_w >> quaternion_x >> quaternion_y >> quaternion_z;

  binary_reader >> client_localization_pose.epoch;

  // Get lidar-odo-pose
  lidar_odo_pose.header.stamp = client_localization_pose.timestamp;
  lidar_odo_pose.header.frame_id =
      ODOM_FRAME_ID;  // TODO here we might have an different reference frame (arbitrary according to API)
  convertPose2DDoubleDatagram2Message(binary_reader, lidar_odo_pose.pose);

  return datagram.size() - binary_reader.available();
}

size_t RosMsgsDatagramConverter::convertClientLocalizationVisualizationDatagram2Message(
    const std::vector<char>& datagram,
    bosch_locator_bridge::ClientLocalizationVisualization& client_localization_visualization,
    geometry_msgs::PoseStamped& pose, sensor_msgs::PointCloud2& scan)
{
  Poco::MemoryInputStream inStream(&datagram[0], datagram.size());
  auto binary_reader = Poco::BinaryReader(inStream, Poco::BinaryReader::LITTLE_ENDIAN_BYTE_ORDER);
  binary_reader.setExceptions(std::ifstream::failbit | std::ifstream::badbit | std::ifstream::eofbit);

  double stamp;
  binary_reader >> stamp;
  client_localization_visualization.timestamp = ros::Time(stamp);
  binary_reader >> client_localization_visualization.unique_id >> client_localization_visualization.loc_state;

  // Get pose
  pose.header.stamp = client_localization_visualization.timestamp;
  pose.header.frame_id = MAP_FRAME_ID;
  convertPose2DDoubleDatagram2Message(binary_reader, pose.pose);

  binary_reader >> client_localization_visualization.delay;
  convertMapDatagram2Message(binary_reader, client_localization_visualization.timestamp, scan);

  return datagram.size() - binary_reader.available();
}

size_t RosMsgsDatagramConverter::convertClientMapVisualizationDatagram2Message(
    const std::vector<char>& datagram, bosch_locator_bridge::ClientMapVisualization& client_map_visualization,
    geometry_msgs::PoseStamped& pose, sensor_msgs::PointCloud2& scan, geometry_msgs::PoseArray& path_poses)
{
  Poco::MemoryInputStream inStream(&datagram[0], datagram.size());
  auto binary_reader = Poco::BinaryReader(inStream, Poco::BinaryReader::LITTLE_ENDIAN_BYTE_ORDER);
  binary_reader.setExceptions(std::ifstream::failbit | std::ifstream::badbit | std::ifstream::eofbit);
  double stamp;
  binary_reader >> stamp;
  client_map_visualization.timestamp = ros::Time(stamp);
  binary_reader >> client_map_visualization.visualization_id >> client_map_visualization.status;

  // Get pose
  pose.header.stamp = client_map_visualization.timestamp;
  pose.header.frame_id = MAP_FRAME_ID;
  convertPose2DDoubleDatagram2Message(binary_reader, pose.pose);

  binary_reader >> client_map_visualization.distanceToLastLC >> client_map_visualization.delay >>
      client_map_visualization.progress;
  convertMapDatagram2Message(binary_reader, client_map_visualization.timestamp, scan);

  // Get path poses
  path_poses.header.stamp = client_map_visualization.timestamp;
  path_poses.header.frame_id = MAP_FRAME_ID;
  uint32_t path_poses_length;
  binary_reader >> path_poses_length;

  for (unsigned int i = 0; i < path_poses_length; i++)
  {
    geometry_msgs::Pose nextPose;
    convertPose2DSingleDatagram2Message(binary_reader, nextPose);
    path_poses.poses.push_back(nextPose);
  }

  // Get path types
  uint32_t path_types_length;
  binary_reader >> path_types_length;

  client_map_visualization.path_types.resize(path_types_length);
  for (unsigned int i = 0; i < path_types_length; i++)
  {
    binary_reader >> client_map_visualization.path_types[i];
  }
  return datagram.size() - binary_reader.available();
}

size_t RosMsgsDatagramConverter::convertClientRecordingVisualizationDatagram2Message(
    const std::vector<char>& datagram,
    bosch_locator_bridge::ClientRecordingVisualization& client_recording_visualization,
    geometry_msgs::PoseStamped& pose, sensor_msgs::PointCloud2& scan, geometry_msgs::PoseArray& path_poses)
{
  Poco::MemoryInputStream inStream(&datagram[0], datagram.size());
  auto binary_reader = Poco::BinaryReader(inStream, Poco::BinaryReader::LITTLE_ENDIAN_BYTE_ORDER);
  binary_reader.setExceptions(std::ifstream::failbit | std::ifstream::badbit | std::ifstream::eofbit);

  double stamp;
  binary_reader >> stamp;
  client_recording_visualization.timestamp = ros::Time(stamp);
  binary_reader >> client_recording_visualization.visualization_id >> client_recording_visualization.status;

  // Get pose
  pose.header.stamp = client_recording_visualization.timestamp;
  pose.header.frame_id = MAP_FRAME_ID;
  convertPose2DDoubleDatagram2Message(binary_reader, pose.pose);

  binary_reader >> client_recording_visualization.distanceToLastLC >> client_recording_visualization.delay >>
      client_recording_visualization.progress;
  convertMapDatagram2Message(binary_reader, client_recording_visualization.timestamp, scan);

  // Get path poses
  path_poses.header.stamp = client_recording_visualization.timestamp;
  path_poses.header.frame_id = MAP_FRAME_ID;
  uint32_t path_poses_length;
  binary_reader >> path_poses_length;

  for (unsigned int i = 0; i < path_poses_length; i++)
  {
    geometry_msgs::Pose nextPose;
    convertPose2DSingleDatagram2Message(binary_reader, nextPose);
    path_poses.poses.push_back(nextPose);
  }

  // Get path types
  uint32_t path_types_length;
  binary_reader >> path_types_length;

  client_recording_visualization.path_types.resize(path_types_length);
  for (unsigned int i = 0; i < path_types_length; i++)
  {
    binary_reader >> client_recording_visualization.path_types[i];
  }
  return datagram.size() - binary_reader.available();
}

size_t RosMsgsDatagramConverter::convertPose2DDoubleDatagram2Message(Poco::BinaryReader& binary_reader,
                                                                     geometry_msgs::Pose& pose)
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

size_t RosMsgsDatagramConverter::convertPose2DSingleDatagram2Message(Poco::BinaryReader& binary_reader,
                                                                     geometry_msgs::Pose& pose)
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

Poco::Buffer<char> RosMsgsDatagramConverter::convertLaserScan2DataGram(const sensor_msgs::LaserScan& msg,
                                                                       size_t scan_num)
{
  // convert the ROS message to a locator ClientSensorLaserDatagram
  const size_t resulting_msg_size = 2        // scanNum
                                    + 5 * 8  // time_start, uniqueId, duration_beam, duration_scan, duration_rotate
                                    + 6 * 4  // numBeams, angleStart, angleEnd, angleInc, minRange, maxRange
                                    + 4      // ranges->length
                                    + msg.ranges.size() * 4        // ranges->elements
                                    + 1                            // hasIntensities
                                    + 2 * 4                        // minIntensity, maxIntensity
                                    + 4                            // intensities->length
                                    + msg.intensities.size() * 4;  // intensities->elements

  Poco::Buffer<char> buffer(resulting_msg_size);
  Poco::MemoryBinaryWriter writer(buffer, Poco::BinaryWriter::StreamByteOrder::LITTLE_ENDIAN_BYTE_ORDER);

  // scanNum
  writer << static_cast<uint16_t>(scan_num);
  // time_start
  writer << msg.header.stamp.toSec();
  // uniqueId
  writer << static_cast<uint64_t>(0);
  // duration_beam
  double duration_beam = static_cast<double>(fabs(msg.time_increment));
  writer << duration_beam;
  // duration_scan
  writer << duration_beam * static_cast<double>(msg.ranges.size());
  // duration_rotate
  writer << static_cast<double>(msg.scan_time);
  // numBeams
  writer << static_cast<uint32_t>(msg.ranges.size());
  // angleStart
  writer << static_cast<float>(msg.angle_min);
  // angleEnd
  writer << static_cast<float>(msg.angle_max);
  // angleInc
  writer << static_cast<float>(msg.angle_increment);
  // minRange
  writer << static_cast<float>(msg.range_min);
  // maxRange
  writer << static_cast<float>(msg.range_max);
  // ranges.length
  writer << static_cast<uint32_t>(msg.ranges.size());
  // ranges.elements
  for (const auto r : msg.ranges)
  {
    writer << static_cast<float>(clamp_range(r, -1e4f, 1e4f));
  }
  writer << static_cast<char>(!msg.intensities.empty());
  // FIXME intensities ranges not in sensor_msgs/LaserScan. Where to get from?
  // minIntensity
  writer << static_cast<float>(0.f);
  // maxIntensity
  writer << static_cast<float>(1.f);
  // intensitites.length
  writer << static_cast<uint32_t>(msg.intensities.size());
  // intensities.elements
  for (const auto intensity : msg.intensities)
  {
    writer << static_cast<float>(intensity);
  }
  writer.flush();

  ROS_ERROR_STREAM_COND(resulting_msg_size != buffer.size(), "convertLaserScan2DataGram: message size mismatch!");

  return buffer;
}

Poco::Buffer<char> RosMsgsDatagramConverter::convertOdometry2DataGram(const nav_msgs::Odometry& msg, size_t odom_num)
{
  // convert the ROS message to a locator ClientSensorLaserDatagram
  const size_t resulting_msg_size = 8        // timestamp
                                    + 4      // odomNum
                                    + 8      // epoch
                                    + 6 * 8  // x,y,yaw,v_x,v_y,omega
                                    + 1;     // velocitySet

  Poco::Buffer<char> buffer(resulting_msg_size);
  Poco::MemoryBinaryWriter writer(buffer, Poco::BinaryWriter::StreamByteOrder::LITTLE_ENDIAN_BYTE_ORDER);

  // timestamp
  writer << msg.header.stamp.toSec();
  // odomNum
  writer << static_cast<uint32_t>(odom_num);
  // epoch... no information available from nav_msgs::Odometry. Keep constant at 0.
  writer << static_cast<uint64_t>(0);

  // Extract yaw from quaternion
  tf2::Quaternion quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                             msg.pose.pose.orientation.w);
  tf2::Matrix3x3 mat(quaternion);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  // Write pose
  writer << msg.pose.pose.position.x << msg.pose.pose.position.y << yaw;

  // Write velocity
  writer << msg.twist.twist.linear.x << msg.twist.twist.linear.y << msg.twist.twist.angular.z;

  // velocity
  // (available in the nav_msgs::Odometry message though its reliability cannot be judged at this stage, activate)
  writer << static_cast<char>(true);

  writer.flush();

  ROS_ERROR_STREAM_COND(resulting_msg_size != buffer.size(), "convertOdometry2DataGram: message size mismatch!");

  return buffer;
}

Poco::JSON::Object RosMsgsDatagramConverter::makePose2d(const geometry_msgs::Pose2D& pose)
{
  Poco::JSON::Object obj;
  obj.set("x", pose.x);
  obj.set("y", pose.y);
  obj.set("a", pose.theta);
  return obj;
}
