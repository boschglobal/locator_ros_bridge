// Copyright (c) 2021 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/boschglobal/rokit_ros_bridge.
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

#include "bosch_navigator_bridge/rosmsgs_datagram_converter.hpp"

#include <fstream>
#include <string>
#include <vector>
#include <utility>

#include "Poco/BinaryWriter.h"
#include "Poco/MemoryStream.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "bosch_navigator_bridge/enums.hpp"


Poco::Buffer<char> RosMsgsDatagramConverter::convertOdometry2FeedbackDataGram(
  const nav_msgs::msg::Odometry::SharedPtr odometry_msg, size_t odom_num_, bool odometrySet,
  rclcpp::Node::SharedPtr node)
{
  constexpr size_t resulting_msg_size = 6 +  // datagram_id
    1 +                                  // Major_version
    1 +                                  // Minor_version
    8 +                                  // timestamp
    4 +                                  // feedback_number
    8 +                                  // error_flags
    8 +                                  // info_flags
    1 +                                  // field_mask
    1 +                                  // available_driving_modes
    2 +                                  // reserved
    8 +                                  // epoch
    3 * 8 +                              // odometry
    12 +                              // velocity
    4 * 4 +                              // wheel_speeds
    4 * 4 +                              // steering_orientations
    1 +                                  // state_of_charge
    1 +                                  // active_monitoring_case
    2 +                                  // active_protection_fields_id
    4 +                                  // allowed_max_linear_speed
    4 +                                  // allowed_max_angular_speed
    4 +                                  // allowed_max_linear_acceleration
    4 +                                  // allowed_max_linear_deceleration
    1 +                                  // custom_action1_number
    1 +                                  // custom_action2_number
    1 +                                  // custom_action1_status
    1 +                                  // custom_action2_status
    4 +                                  // custom_action1_result
    4;                                   // custom_action2_result


  Poco::Buffer<char> buffer(resulting_msg_size);
  Poco::MemoryBinaryWriter writer(buffer,
    Poco::BinaryWriter::StreamByteOrder::LITTLE_ENDIAN_BYTE_ORDER);

  // datagram_id

  writer << static_cast<uint8_t>('R');
  writer << static_cast<uint8_t>('N');
  writer << static_cast<uint8_t>('C');
  writer << static_cast<uint8_t>('M');
  writer << static_cast<uint8_t>('F');
  writer << static_cast<uint8_t>('B');


  // Major_version
  writer << static_cast<uint8_t>(1);

  // Minor_version
  writer << static_cast<uint8_t>(0);


  // timestamp
  rclcpp::Time time_stamp = odometry_msg->header.stamp;
  writer << time_stamp.seconds();

  // feedback_number
  writer << static_cast<uint32_t>(odom_num_);

  // errorFlags
  writer << static_cast<uint64_t>(0x00);

  // infoFlags
  writer <<
    static_cast<uint64_t>(FeedbackInfoFlags::READY_TO_DRIVE | FeedbackInfoFlags::AUTOMATIC_MODE);

  // fieldMask
  if (odometrySet) {
    writer <<
      static_cast<uint8_t>(FeedbackFieldMask::ODOMETRY_VALID | FeedbackFieldMask::VELOCITY_VALID);
  } else {
    writer << static_cast<uint8_t>(FeedbackFieldMask::VELOCITY_VALID);
  }

  // availableDrivingModes
  writer << static_cast<uint8_t>(FeedbackAvailableDrivingModes::BACKWARD_DRIVING |
  FeedbackAvailableDrivingModes::FORWARD_DRIVING | FeedbackAvailableDrivingModes::TURN_ON_SPOT |
  FeedbackAvailableDrivingModes::OMNIDIRECTIONAL_DRIVING);

  // reserved
  writer << static_cast<uint16_t>(0);

  // epoch
  writer << static_cast<uint64_t>(0);

  // odometry
  writer << static_cast<double>(odometry_msg->pose.pose.position.x);
  writer << static_cast<double>(odometry_msg->pose.pose.position.y);

  // angle from the quaternion
  double yaw = tf2::getYaw(odometry_msg->pose.pose.orientation);

  writer << static_cast<double>(yaw);

  // velocity
  writer << static_cast<float>(odometry_msg->twist.twist.linear.x);
  writer << static_cast<float>(odometry_msg->twist.twist.linear.y);
  writer << static_cast<float>(odometry_msg->twist.twist.angular.z);

  // wheelSpeeds
  writer << static_cast<float>(0.0);
  writer << static_cast<float>(0.0);
  writer << static_cast<float>(0.0);
  writer << static_cast<float>(0.0);

  // steeringOrientations
  writer << static_cast<float>(0.0);
  writer << static_cast<float>(0.0);
  writer << static_cast<float>(0.0);
  writer << static_cast<float>(0.0);

  // stateOfCharge
  writer << static_cast<uint8_t>(100);

  // activeMonitoringCase
  writer << static_cast<uint8_t>(0);

  // activeProtectionFieldIds
  writer << static_cast<uint8_t>(0);
  writer << static_cast<uint8_t>(0);

  // allowedMaxLinearSpeed
  writer << static_cast<float>(std::numeric_limits<float>::max());

  // allowedMaxAngularSpeed
  writer << static_cast<float>(std::numeric_limits<float>::max());

  // allowedMaxLinearAcceleration
  writer << static_cast<float>(std::numeric_limits<float>::max());

  // allowedMaxLinearDeceleration
  writer << static_cast<float>(std::numeric_limits<float>::max());

  // customAction1Number
  writer << static_cast<uint8_t>(0);

  // customAction2Number
  writer << static_cast<uint8_t>(0);

  // customAction1Status
  writer << static_cast<int8_t>(0);

  // customAction2Status
  writer << static_cast<int8_t>(0);

  // customAction1Result
  writer << static_cast<float>(0);
  // customAction2Result
  writer << static_cast<float>(0);


  writer.flush();

  RCLCPP_ERROR_STREAM_EXPRESSION(
    node->get_logger(),
    resulting_msg_size != buffer.size(),
    "convertOdometry2FeedbackDataGram: message size mismatch!");

  return buffer;
}

size_t RosMsgsDatagramConverter::convertMotionCommand2Twist(
  const std::vector<char> & datagram,
  geometry_msgs::msg::Twist & twist)
{
  Poco::MemoryInputStream inStream(&datagram[0], datagram.size());
  auto binary_reader = Poco::BinaryReader(inStream, Poco::BinaryReader::LITTLE_ENDIAN_BYTE_ORDER);
  binary_reader.setExceptions(
    std::ifstream::failbit | std::ifstream::badbit |
    std::ifstream::eofbit);

  // datagramId
  std::array<uint8_t, 6> datagramId;
  binary_reader >> datagramId[0];
  binary_reader >> datagramId[1];
  binary_reader >> datagramId[2];
  binary_reader >> datagramId[3];
  binary_reader >> datagramId[4];
  binary_reader >> datagramId[5];

  // majorVersion
  uint8_t majorVersion;
  binary_reader >> majorVersion;

  // minorVersion
  uint8_t minorVersion;
  binary_reader >> minorVersion;

  // timestamp
  double timestamp;
  binary_reader >> timestamp;

  // commandNumber
  uint32_t commandNumber;
  binary_reader >> commandNumber;

  // referenceTimestamp
  double referenceTimestamp;
  binary_reader >> referenceTimestamp;

  // errorFlags
  uint64_t errorFlags;
  binary_reader >> errorFlags;

  // infoFlags
  uint64_t infoFlags;
  binary_reader >> infoFlags;

  // motionCommand
  std::array<float, 3> motionCommand;

  binary_reader >> motionCommand[0];
  binary_reader >> motionCommand[1];
  binary_reader >> motionCommand[2];

  twist.linear.x = motionCommand[0];
  twist.linear.y = motionCommand[1];
  twist.linear.z = 0.0;

  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = motionCommand[2];

  // vehiclePose
  std::array<double, 3> vehiclePose;


  binary_reader >> vehiclePose[0];
  binary_reader >> vehiclePose[1];
  binary_reader >> vehiclePose[2];


  // poweredWheelSpeeds
  std::array<float, 4> poweredWheelSpeeds;

  binary_reader >> poweredWheelSpeeds[0];
  binary_reader >> poweredWheelSpeeds[1];
  binary_reader >> poweredWheelSpeeds[2];
  binary_reader >> poweredWheelSpeeds[3];

  // steeringActorOrientations
  std::array<float, 4> steeringActorOrientations;


  binary_reader >> steeringActorOrientations[0];
  binary_reader >> steeringActorOrientations[1];
  binary_reader >> steeringActorOrientations[2];
  binary_reader >> steeringActorOrientations[3];

  // requestMonitoringCase
  uint8_t requestMonitoringCase;
  binary_reader >> requestMonitoringCase;

  // requestDrivingMode
  uint8_t requestDrivingMode;
  binary_reader >> requestDrivingMode;

  // requestCostomAction1Number
  uint8_t requestCostomAction1Number;
  binary_reader >> requestCostomAction1Number;

  // requestCostomAction2Number
  uint8_t requestCostomAction2Number;
  binary_reader >> requestCostomAction2Number;

  // customAction1Id
  std::array<uint8_t, 4> customAction1Id;


  binary_reader >> customAction1Id[0];
  binary_reader >> customAction1Id[1];
  binary_reader >> customAction1Id[2];
  binary_reader >> customAction1Id[3];

  // customAction2Id
  std::array<uint8_t, 4> customAction2Id;

  binary_reader >> customAction2Id[0];
  binary_reader >> customAction2Id[1];
  binary_reader >> customAction2Id[2];
  binary_reader >> customAction2Id[3];

  // customAction1Data
  std::array<float, 8> customAction1Data;


  binary_reader >> customAction1Data[0];
  binary_reader >> customAction1Data[1];
  binary_reader >> customAction1Data[2];
  binary_reader >> customAction1Data[3];
  binary_reader >> customAction1Data[4];
  binary_reader >> customAction1Data[5];
  binary_reader >> customAction1Data[6];
  binary_reader >> customAction1Data[7];

  // customAction2Data
  float customAction2Data;


  binary_reader >> customAction2Data;


  return datagram.size() - binary_reader.available();
}
