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

#ifndef BOSCH_LOCATOR_BRIDGE__ENUMS_HPP_
#define BOSCH_LOCATOR_BRIDGE__ENUMS_HPP_

#include <stdint.h>
#include <string>

/**
 * Module identifiers. See api documentation, section 6.2
 */
enum ModuleIdentifier : uint16_t
{
  COMMON = 0x000,
  ABOUT_MODULES = 0x0001,
  SESSION = 0x0002,
  // error in documentation, 0x0002 is used twice
  // DIAGNOSTIC = 0x0002,
  LICENSINGFEATURE = 0x0003,
  CONFIG = 0x0004,
  ABOUT_BUILD = 0x0005,
  CERTIFICATES = 0x0006,
  SYSTEM = 0x0007,
  CLIENT_RECORDING = 0x0100,
  CLIENT_MAP = 0x0101,
  CLIENT_LOCALIZATION = 0x0102,
  CLIENT_MANUAL_ALIGN = 0x0103,
  CLIENT_GLOBAL_ALIGN = 0x0104,
  CLIENT_LASER_MASK = 0x0105,
  CLIENT_USER = 0x0106,
  SERVER_MAP = 0x0200,
  SERVER_USER = 0x0201,
  SUPPORT_REPORT = 0x0300,
  SUPPORT_RECOVERY = 0x0301,
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
    // case DIAGNOSTIC:
    //  return "DIAGNOSTIC";
    case LICENSINGFEATURE:
      return "LICENSINGFEATURE";
    case CONFIG:
      return "CONFIG";
    case ABOUT_BUILD:
      return "ABOUT_BUILD";
    case CERTIFICATES:
      return "CERTIFICATES";
    case SYSTEM:
      return "SYSTEM";
    case CLIENT_RECORDING:
      return "CLIENT_RECORDING";
    case CLIENT_MAP:
      return "CLIENT_MAP";
    case CLIENT_LOCALIZATION:
      return "CLIENT_LOCALIZATION";
    case CLIENT_MANUAL_ALIGN:
      return "CLIENT_MANUAL_ALIGN";
    case CLIENT_GLOBAL_ALIGN:
      return "CLIENT_GLOBAL_ALIGN";
    case CLIENT_LASER_MASK:
      return "CLIENT_LASER_MASK";
    case CLIENT_USER:
      return "CLIENT_USER";
    case SERVER_MAP:
      return "SERVER_MAP";
    case SERVER_USER:
      return "SERVER_USER";
    case SUPPORT_REPORT:
      return "SUPPORT_REPORT";
    case SUPPORT_RECOVERY:
      return "SUPPORT_RECOVERY";
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

#endif  // BOSCH_LOCATOR_BRIDGE__ENUMS_HPP_
