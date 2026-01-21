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

#include "bosch_navigator_bridge/navigator_bridge_node.hpp"

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "bosch_navigator_bridge/receiving_interface.hpp"
#include "bosch_navigator_bridge/sending_interface.hpp"
#include "bosch_navigator_bridge/rosmsgs_datagram_converter.hpp"
#include "bosch_navigator_bridge/navigator_interface.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;


// navigator module versions to check against. Format is name, { major_version, minor_version }
static const std::unordered_map<std::string, std::pair<int32_t, int32_t>> REQUIRED_MODULE_VERSIONS({
  {"AboutModules", {5, 0}},
  {"Session", {4, 0}},
  {"Config", {8, 0}},
  {"User", {1, 0}},
  {"ClientMotion", {2, 0}},
});


NavigatorBridgeNode::NavigatorBridgeNode(const std::string & nodeName)
: Node(nodeName,
    rclcpp::NodeOptions().allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
{
}

NavigatorBridgeNode::~NavigatorBridgeNode()
{
  odom_sending_interface_->stop();
  odom_sending_interface_thread_.join();
}

void NavigatorBridgeNode::init()
{
  std::string host;
  get_parameter("nav_host", host);

  int tmp_binaryPortsStart, tmp_rpcPort;
  get_parameter("nav_binary_ports_start", tmp_binaryPortsStart);
  uint16_t binaryPortsStart{static_cast<uint16_t>(tmp_binaryPortsStart)};
  get_parameter("nav_rpc_port", tmp_rpcPort);
  uint16_t rpcPort{static_cast<uint16_t>(tmp_rpcPort)};

  std::string user, pwd;
  get_parameter("user_name", user);
  get_parameter("password", pwd);

  callback_group_services_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // NOTE for now, we only have a session management with the navigation client
  // Same thing is likely needed for the map server
  nav_client_interface_.reset(new NavigatorInterface(host, rpcPort));
  nav_client_interface_->login(user, pwd);
  session_refresh_timer_ = create_wall_timer(
    30s, [&]() {
      RCLCPP_INFO_STREAM(get_logger(), "refreshing session!");
      nav_client_interface_->refresh();
    },
    callback_group_services_);

  const auto module_versions = nav_client_interface_->getAboutModules();
  if (!check_module_versions(module_versions)) {
    throw std::runtime_error("navigator software incompatible with this bridge!");
  }

  syncConfig();

  // Create interface to send binary odometry data if requested
  int feedback_datagram_port;
  get_parameter("feedback_datagram_port", feedback_datagram_port);

  odom_sending_interface_.reset(new SendingInterface(feedback_datagram_port, shared_from_this()));
  odom_sending_interface_thread_.start(*odom_sending_interface_);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
    [this](nav_msgs::msg::Odometry::SharedPtr msg) {this->odom_callback(msg);}
  );

  get_parameter("odometry_pose_set", odometry_pose_set_);
  setupBinaryReceiverInterfaces(host, static_cast<Poco::UInt16>(binaryPortsStart));

  RCLCPP_INFO_STREAM(get_logger(), "initialization done");
}


bool NavigatorBridgeNode::check_module_versions(
  const std::unordered_map<std::string, std::pair<int32_t, int32_t>> & module_versions)
{
  RCLCPP_INFO(get_logger(), "-----------------check_module_versions");
  for (const auto & required_pair : REQUIRED_MODULE_VERSIONS) {
    const auto & module_name = required_pair.first;
    const auto & required_version = required_pair.second;

    const auto & actual_version_iter = module_versions.find(module_name);
    if (actual_version_iter == module_versions.end()) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "required Navigator module " << module_name << " not found!");
      return false;
    }
    const auto & actual_version = actual_version_iter->second;
    // major version number needs to match, minor version number equal or bigger
    if ((actual_version.first == required_version.first) &&
      (actual_version.second >= required_version.second))
    {
      RCLCPP_DEBUG_STREAM(get_logger(), "Navigator module " << module_name << ": version ok!");
    } else {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "---------8 module: " << module_name << " required version: " << required_version.first <<
          "." << required_version.second << " (actual version: " << actual_version.first << "." <<
          actual_version.second << ")");
      return false;
    }
  }
  return true;
}


void NavigatorBridgeNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  Poco::Buffer<char> feedback_datagram = RosMsgsDatagramConverter::convertOdometry2FeedbackDataGram(
    msg,
    ++odom_num_, odometry_pose_set_,
    shared_from_this());
  odom_sending_interface_->sendData(feedback_datagram.begin(), feedback_datagram.size());
}


void NavigatorBridgeNode::syncConfig()
{
  RCLCPP_INFO_STREAM(get_logger(), "syncing config");


  auto nav_client_config = nav_client_interface_->getConfigList();
  if (odometry_pose_set_) {
    nav_client_config["ClientMotion.odometrySource"] = "MOTION_FEEDBACK_ODOMETRY";
  } else {
    nav_client_config["ClientMotion.odometrySource"] = "MOTION_FEEDBACK_VELOCITY";
  }
  // overwrite current navigator config with ros params

  std::map<std::string, rclcpp::Parameter> navigator_parameters;
  get_node_parameters_interface()->get_parameters_by_prefix(
    "navigator_client_config",
    navigator_parameters);
  std::for_each(
    navigator_parameters.begin(), navigator_parameters.end(), [&nav_client_config,
    logger = get_logger()](const std::pair<std::string, rclcpp::Parameter> & param) {
      switch (param.second.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          nav_client_config[param.first] = param.second.as_bool();
          break;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          nav_client_config[param.first] = param.second.as_int();
          break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          nav_client_config[param.first] = param.second.as_double();
          break;
        case rclcpp::ParameterType::PARAMETER_STRING:
          nav_client_config[param.first] = param.second.as_string();
          break;
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
          nav_client_config[param.first] = param.second.as_bool_array();
          break;
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
          nav_client_config[param.first] = param.second.as_integer_array();
          break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
          nav_client_config[param.first] = param.second.as_double_array();
          break;
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
          nav_client_config[param.first] = param.second.as_string_array();
          break;
        default:
          RCLCPP_WARN(
            logger, "Parameter type %s is unsupported for Navigator config!",
            param.second.get_type_name().c_str());
      }
    });


  RCLCPP_INFO_STREAM(get_logger(), "new nav client config: " << nav_client_config.toString());
  for (const auto & c : nav_client_config) {
    RCLCPP_INFO_STREAM(get_logger(), "- " << c.first << ": " << c.second.toString());
  }


  if (!nav_client_interface_->setConfigList(nav_client_config)) {
    // Try to stop everything before setting config list
    RCLCPP_ERROR(
      get_logger(),
      "One of the modes appears to be in a RUN-state. In order to set the configuration parameters,"
      " all modes are now stopped! ");
  }
}


void NavigatorBridgeNode::setupBinaryReceiverInterfaces(
  const std::string & host,
  const Poco::UInt16 binaryPortsStart)
{
  Poco::UInt16 binaryClientMotionCommandPort{binaryPortsStart /*default: 9104*/};

  // Create binary interface for client motion Command
  client_motion_command_interface_.reset(
    new ClientMotionCommandInterface(
      Poco::Net::IPAddress(host),
      binaryClientMotionCommandPort,
      shared_from_this()));
  client_motion_command_interface_thread_.start(*client_motion_command_interface_);
}
