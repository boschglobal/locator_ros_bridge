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

#ifndef BOSCH_NAVIGATOR_BRIDGE__ENUMS_HPP_
#define BOSCH_NAVIGATOR_BRIDGE__ENUMS_HPP_

#include <cstdint>
#include <string>

/**
 * @brief Identifiers for various modules within the Navigator system
 */
enum ModuleIdentifier : uint16_t
{
  COMMON = 0x000,
  ABOUT_MODULES = 0x0001,
  SESSION = 0x0002,
  LICENSING = 0x0003,
  DIAGNOSTIC = 0x0008,
  CONFIG = 0x0004,
  ABOUT_BUILD = 0x0005,
  CERTIFICATE = 0x0006,
  SYSTEM = 0x0007,
  SUPPORT_REPORT = 0x0300,
  SUPPORT_RECOVERY = 0x0301,
  CLIENT_APPLICATION = 0x0500,
};


/**
 * @brief Flags representing error feedback from the Navigator
 */
enum FeedbackErrorFlags : uint64_t
{
  VEHICLE_ERROR = 0x01,
  EMERGENCY_STOP = 0x02,
};

enum FeedbackInfoFlags : uint64_t
{
  READY_TO_DRIVE = 0x0001,
  AUTOMATIC_MODE = 0x0002,
  PROTECTION_FIELD_VIOLATED = 0x0004,
  WARNING_FIELD_VIOLATED = 0x0008,
  STATIONARY = 0x0010,
  CHARGING = 0x0020,
  LOADED = 0x0040,
  DOCKED = 0x0080,
  DIRECTION_INDICATOR_LEFT_ACTIVE = 0x0100,
  DIRECTION_INDICATOR_RIGHT_ACTIVE = 0x0200,
  DIRECTION_INDICATOR_FORWARD_ACTIVE = 0x0400,
  DIRECTION_INDICATOR_BACKWARD_ACTIVE = 0x0800,
  HAZARD_INDICATOR_ACTIVE = 0x1000,
  HORN_ACTIVE = 0x2000,
};

enum FeedbackFieldMask : uint8_t
{
  ODOMETRY_VALID = 0x01,
  VELOCITY_VALID = 0x02,
  ARRAY_WHEEL_SPEEDS_VALID = 0x04,
  ARRAY_STEERING_ORIENTATIONS_VALID = 0x08,
  ACTIVE_MONITORING_CASE_VALID = 0x10,
  ARRAY_ACTIVE_PROTECTION_FIELDIDS_VALID = 0X20,
};

enum FeedbackAvailableDrivingModes : uint8_t
{
  FORWARD_DRIVING = 0x01,
  BACKWARD_DRIVING = 0x02,
  TURN_ON_SPOT = 0x04,
  OMNIDIRECTIONAL_DRIVING = 0x20,
};

inline std::string stringifyModuleId(ModuleIdentifier id)
{
  switch (id) {
    case COMMON:
      return "Common";
    case ABOUT_MODULES:
      return "ABOUT_MODULES";
    case SESSION:
      return "SESSION";
    case DIAGNOSTIC:
      return "DIAGNOSTIC";
    case LICENSING:
      return "LICENSINGFEATURE";
    case CONFIG:
      return "CONFIG";
    case ABOUT_BUILD:
      return "ABOUT_BUILD";
    case CERTIFICATE:
      return "CERTIFICATES";
    case SYSTEM:
      return "SYSTEM";
    case SUPPORT_REPORT:
      return "SUPPORT_REPORT";
    case SUPPORT_RECOVERY:
      return "SUPPORT_RECOVERY";
    case CLIENT_APPLICATION:
      return "CLIENT_APPLICATION";
    default:
      return "<unknown module>";
  }
}

enum CommonResponseCode : uint64_t
{
  OK = 0x0000000000000000,
  WARNING = 0x0000000000000001,
  INTERNAL_ERROR = 0x0000000000000002,
  UNKNOWN_ERROR = 0x0000000000000003,
  SESSION_INVALID = 0x0000000000000004,
  SESSION_EXPIRED = 0x0000000000000005,
  NOT_AUTHORIZED = 0x0000000000000006,
  NOT_IN_REQUIRED_STATE = 0x0000000000000007,
  FEATURE_NOT_LICENSED = 0x0000000000000008,
  INVALID_MESSAGE_CONTENT = 0x0000000000000009,
  ENTITY_ALREADY_EXISTS = 0x000000000000000a,
  ENTITY_NOT_FOUND = 0x000000000000000b,
  FILE_ACCESS_FAILED = 0x000000000000000c,
  SENSOR_NOT_AVAILABLE = 0x000000000000000d,
  ENTITY_IN_USE = 0x000000000000000e,
};

inline std::string stringifyCommonResponseCode(CommonResponseCode c)
{
  switch (c) {
    case OK:
      return "OK";
    case WARNING:
      return "WARNING";
    case INTERNAL_ERROR:
      return "INTERNAL_ERROR";
    case UNKNOWN_ERROR:
      return "UNKNOWN_ERROR";
    case SESSION_INVALID:
      return "SESSION_INVALID";
    case SESSION_EXPIRED:
      return "SESSION_EXPIRED";
    case NOT_AUTHORIZED:
      return "NOT_AUTHORIZED";
    case NOT_IN_REQUIRED_STATE:
      return "NOT_IN_REQUIRED_STATE";
    case FEATURE_NOT_LICENSED:
      return "FEATURE_NOT_LICENSED";
    case INVALID_MESSAGE_CONTENT:
      return "INVALID_MESSAGE_CONTENT";
    case ENTITY_ALREADY_EXISTS:
      return "ENTITY_ALREADY_EXISTS";
    case ENTITY_NOT_FOUND:
      return "ENTITY_NOT_FOUND";
    case FILE_ACCESS_FAILED:
      return "FILE_ACCESS_FAILED";
    case SENSOR_NOT_AVAILABLE:
      return "SENSOR_NOT_AVAILABLE";
    case ENTITY_IN_USE:
      return "ENTITY_IN_USE";
    default:
      return "";
  }
}

#endif  // BOSCH_NAVIGATOR_BRIDGE__ENUMS_HPP_
