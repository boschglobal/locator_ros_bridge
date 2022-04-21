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

#include "locator_bridge_node.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "tf2/convert.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "locator_rpc_interface.hpp"
#include "receiving_interface.hpp"
#include "rosmsgs_datagram_converter.hpp"
#include "sending_interface.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/// locator module versions to check against. Format is name, { major_version, minor_version }
static const std::unordered_map<std::string, std::pair<int32_t, int32_t>> REQUIRED_MODULE_VERSIONS({
  {"AboutModules", {5, 0}},
  {"Session", {3, 1}},
//  {"Diagnostic", {4, 0}},
  {"Licensing", {6, 1}},
  {"Config", {5, 1}},
  {"AboutBuild", {3, 0}},
  {"Certificate", {3, 0}},
  {"System", {3, 1}},
//  {"ClientApplication", {1, 0}},
  {"ClientControl", {3, 1}},
  {"ClientRecording", {4, 0}},
  {"ClientMap", {4, 0}},
  {"ClientLocalization", {6, 0}},
//  {"ClientManualAlign", {5, 0}},
  {"ClientGlobalAlign", {4, 0}},
//  {"ClientLaserMask", {5, 0}},
  {"ClientSensor", {5, 1}},
//  {"ClientUser", {4, 0}},
//  {"User", {1, 0}},
//  {"ClientExpandMap", {2, 0}},
});

LocatorBridgeNode::LocatorBridgeNode(const std::string & nodeName)
: Node(nodeName)
{
}

LocatorBridgeNode::~LocatorBridgeNode()
{
  laser_sending_interface_->stop();
  laser_sending_interface_thread_.join();

  if (laser2_sending_interface_) {
    laser2_sending_interface_->stop();
    laser2_sending_interface_thread_.join();
  }

  if (odom_sending_interface_) {
    odom_sending_interface_->stop();
    odom_sending_interface_thread_.join();
  }
}

void LocatorBridgeNode::init()
{
  std::string host;
  declare_parameter("locator_host", host);
  get_parameter("locator_host", host);

  std::string user, pwd;
  declare_parameter("user_name", user);
  get_parameter("user_name", user);
  declare_parameter("password", pwd);
  get_parameter("password", pwd);

  // NOTE for now, we only have a session management with the localization client
  // Same thing is likely needed for the map server
  loc_client_interface_.reset(new LocatorRPCInterface(host, 8080));
  loc_client_interface_->login(user, pwd);
  session_refresh_timer_ = create_wall_timer(
    30s, [&]() {
      RCLCPP_INFO_STREAM(get_logger(), "refreshing session!");
      loc_client_interface_->refresh();
    });

  const auto module_versions = loc_client_interface_->getAboutModules();
  if (!check_module_versions(module_versions)) {
    throw std::runtime_error("locator software incompatible with this bridge!");
  }

  syncConfig();

  callback_group_services_ = create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

  services_.push_back(
    create_service<bosch_locator_bridge::srv::ClientConfigGetEntry>(
      "~/get_config_entry",
      std::bind(&LocatorBridgeNode::clientConfigGetEntryCb, this, _1, _2),
      rmw_qos_profile_services_default, callback_group_services_));

  services_.push_back(
    create_service<bosch_locator_bridge::srv::StartRecording>(
      "~/start_visual_recording",
      std::bind(&LocatorBridgeNode::clientRecordingStartVisualRecordingCb, this, _1, _2),
      rmw_qos_profile_services_default, callback_group_services_));
  services_.push_back(
    create_service<std_srvs::srv::Empty>(
      "~/stop_visual_recording",
      std::bind(&LocatorBridgeNode::clientRecordingStopVisualRecordingCb, this, _1, _2),
      rmw_qos_profile_services_default, callback_group_services_));

  services_.push_back(
    create_service<bosch_locator_bridge::srv::ClientMapStart>(
      "~/start_map", std::bind(&LocatorBridgeNode::clientMapStartCb, this, _1, _2),
      rmw_qos_profile_services_default, callback_group_services_));
  services_.push_back(
    create_service<std_srvs::srv::Empty>(
      "~/stop_map", std::bind(&LocatorBridgeNode::clientMapStopCb, this, _1, _2),
      rmw_qos_profile_services_default, callback_group_services_));

  services_.push_back(
    create_service<std_srvs::srv::Empty>(
      "~/start_localization",
      std::bind(&LocatorBridgeNode::clientLocalizationStartCb, this, _1, _2),
      rmw_qos_profile_services_default, callback_group_services_));
  services_.push_back(
    create_service<std_srvs::srv::Empty>(
      "~/stop_localization", std::bind(&LocatorBridgeNode::clientLocalizationStopCb, this, _1, _2),
      rmw_qos_profile_services_default, callback_group_services_));

  services_.push_back(
    create_service<bosch_locator_bridge::srv::ClientMapSend>(
      "~/send_map", std::bind(&LocatorBridgeNode::clientMapSendCb, this, _1, _2),
      rmw_qos_profile_services_default, callback_group_services_));
  services_.push_back(
    create_service<bosch_locator_bridge::srv::ClientMapSet>(
      "~/set_map", std::bind(&LocatorBridgeNode::clientMapSetCb, this, _1, _2),
      rmw_qos_profile_services_default, callback_group_services_));
  services_.push_back(
    create_service<bosch_locator_bridge::srv::ClientMapList>(
      "~/list_client_maps", std::bind(&LocatorBridgeNode::clientMapList, this, _1, _2),
      rmw_qos_profile_services_default, callback_group_services_));

  // subscribe to default topic published by rviz "2D Pose Estimate" button for setting seed
  set_seed_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose",
    1,
    std::bind(&LocatorBridgeNode::setSeedCallback, this, _1));

  // Create interface to send binary laser data if requested
  if (provide_laser_data_) {
    int laser_datagram_port;
    declare_parameter("laser_datagram_port", laser_datagram_port);
    get_parameter("laser_datagram_port", laser_datagram_port);

    laser_sending_interface_.reset(new SendingInterface(laser_datagram_port, shared_from_this()));
    laser_sending_interface_thread_.start(*laser_sending_interface_);

    // Create subscriber to laser data

    std::string scan_topic = "/scan";
    declare_parameter("scan_topic", scan_topic);
    get_parameter("scan_topic", scan_topic);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        qos_profile.history,
        qos_profile.depth), qos_profile);
    laser_sub_ =
      create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, qos,
      std::bind(&LocatorBridgeNode::laser_callback, this, _1));
  }

  // Create interface to send binary laser2 data if requested
  if (provide_laser2_data_) {
    int laser2_datagram_port;
    declare_parameter("laser2_datagram_port", laser2_datagram_port);
    get_parameter("laser2_datagram_port", laser2_datagram_port);

    laser2_sending_interface_.reset(new SendingInterface(laser2_datagram_port, shared_from_this()));
    laser2_sending_interface_thread_.start(*laser2_sending_interface_);

    // Create subscriber to laser2 data

    std::string scan2_topic = "/scan2";
    declare_parameter("scan2_topic", scan2_topic);
    get_parameter("scan2_topic", scan2_topic);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        qos_profile.history,
        qos_profile.depth), qos_profile);
    laser2_sub_ =
      create_subscription<sensor_msgs::msg::LaserScan>(
      scan2_topic, qos,
      std::bind(&LocatorBridgeNode::laser2_callback, this, _1));
  }

  // Create interface to send binary odometry data if requested
  if (provide_odometry_data_) {
    int odom_datagram_port;
    declare_parameter("odom_datagram_port", odom_datagram_port);
    get_parameter("odom_datagram_port", odom_datagram_port);

    odom_sending_interface_.reset(new SendingInterface(odom_datagram_port, shared_from_this()));
    odom_sending_interface_thread_.start(*odom_sending_interface_);

    // Create subscriber to odometry data

    std::string odom_topic = "/odom";
    declare_parameter("odom_topic", odom_topic);
    get_parameter("odom_topic", odom_topic);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        qos_profile.history,
        qos_profile.depth), qos_profile);
    odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, qos,
      std::bind(&LocatorBridgeNode::odom_callback, this, _1));
  }

  setupBinaryReceiverInterfaces(host);

  RCLCPP_INFO_STREAM(get_logger(), "initialization done");
}

bool LocatorBridgeNode::check_module_versions(
  const std::unordered_map<std::string, std::pair<int32_t, int32_t>> & module_versions)
{
  RCLCPP_INFO(get_logger(), "-----------------check_module_versions");
  for (const auto & required_pair : REQUIRED_MODULE_VERSIONS) {
    const auto & module_name = required_pair.first;
    const auto & required_version = required_pair.second;

    const auto & actual_version_iter = module_versions.find(module_name);
    if (actual_version_iter == module_versions.end()) {
      RCLCPP_WARN_STREAM(get_logger(), "required locator module " << module_name << " not found!");
      return false;
    }
    const auto & actual_version = actual_version_iter->second;
    // major version number needs to match, minor version number equal or bigger
    if ((actual_version.first == required_version.first) &&
      (actual_version.second >= required_version.second))
    {
      RCLCPP_DEBUG_STREAM(get_logger(), "locator module " << module_name << ": version ok!");
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

void LocatorBridgeNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // If scan_time is not set, use timestamp difference to set it.
  if (!msg->scan_time) {
    rclcpp::Time laser_timestamp = msg->header.stamp;
    if (prev_laser_timestamp_.seconds() != 0.0) {
      msg->scan_time = (laser_timestamp - prev_laser_timestamp_).seconds();
    }
    prev_laser_timestamp_ = laser_timestamp;
  }

  Poco::Buffer<char> laserscan_datagram = RosMsgsDatagramConverter::convertLaserScan2DataGram(
    msg,
    ++scan_num_,
    shared_from_this());
  laser_sending_interface_->sendData(laserscan_datagram.begin(), laserscan_datagram.size());
}

void LocatorBridgeNode::laser2_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // If scan_time is not set, use timestamp difference to set it.
  if (!msg->scan_time) {
    rclcpp::Time laser_timestamp = msg->header.stamp;
    if (prev_laser2_timestamp_.seconds() != 0.0) {
      msg->scan_time = (laser_timestamp - prev_laser2_timestamp_).seconds();
    }
    prev_laser2_timestamp_ = laser_timestamp;
  }

  Poco::Buffer<char> laserscan_datagram = RosMsgsDatagramConverter::convertLaserScan2DataGram(
    msg,
    ++scan2_num_,
    shared_from_this());
  laser2_sending_interface_->sendData(laserscan_datagram.begin(), laserscan_datagram.size());
}

void LocatorBridgeNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  Poco::Buffer<char> odom_datagram = RosMsgsDatagramConverter::convertOdometry2DataGram(
    msg,
    ++odom_num_,
    shared_from_this());
  odom_sending_interface_->sendData(odom_datagram.begin(), odom_datagram.size());
}

bool LocatorBridgeNode::clientConfigGetEntryCb(
  const std::shared_ptr<bosch_locator_bridge::srv::ClientConfigGetEntry::Request> req,
  std::shared_ptr<bosch_locator_bridge::srv::ClientConfigGetEntry::Response> res)
{
  const auto & loc_client_config = loc_client_interface_->getConfigList();

  try {
    res->value = loc_client_config[req->name].toString();
  } catch (const Poco::NotFoundException & error) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not find config entry " << req->name << ".");
    return false;
  }

  return true;
}

bool LocatorBridgeNode::clientMapSendCb(
  const std::shared_ptr<bosch_locator_bridge::srv::ClientMapSend::Request> req,
  std::shared_ptr<bosch_locator_bridge::srv::ClientMapSend::Response>/*res*/)
{
  const std::string client_map_name = req->name.empty() ? last_map_name_ : req->name;

  auto query = loc_client_interface_->getSessionQuery();
  query.set("clientMapName", client_map_name);
  auto response = loc_client_interface_->call("clientMapSend", query);
  return true;
}

bool LocatorBridgeNode::clientMapSetCb(
  const std::shared_ptr<bosch_locator_bridge::srv::ClientMapSet::Request> req,
  std::shared_ptr<bosch_locator_bridge::srv::ClientMapSet::Response>/*res*/)
{
  const std::string active_map_name = req->name.empty() ? last_map_name_ : req->name;

  Poco::DynamicStruct config;
  config.insert("ClientLocalization.activeMapName", active_map_name);
  loc_client_interface_->setConfigList(config);
  return true;
}

bool LocatorBridgeNode::clientMapList(
  const std::shared_ptr<bosch_locator_bridge::srv::ClientMapList::Request>/*req*/,
  std::shared_ptr<bosch_locator_bridge::srv::ClientMapList::Response> res)
{
  auto query = loc_client_interface_->getSessionQuery();
  auto response = loc_client_interface_->call("clientMapList", query);
  if (response.has("clientMapNames")) {
    const auto entries = response.getArray("clientMapNames");
    for (size_t i = 0; i < entries->size(); i++) {
      res->names.push_back(entries->get(i).toString());
    }
  }
  return true;
}

bool LocatorBridgeNode::clientLocalizationStartCb(
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  auto query = loc_client_interface_->getSessionQuery();
  auto response = loc_client_interface_->call("clientLocalizationStart", query);
  return true;
}

bool LocatorBridgeNode::clientLocalizationStopCb(
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  auto query = loc_client_interface_->getSessionQuery();
  auto response = loc_client_interface_->call("clientLocalizationStop", query);
  return true;
}

void LocatorBridgeNode::setSeedCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (msg->header.frame_id != MAP_FRAME_ID) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "2D Pose Estimate sent in wrong frame! Is: " << msg->header.frame_id <<
        " but should be " << MAP_FRAME_ID);
    return;
  }
  auto query = loc_client_interface_->getSessionQuery();
  geometry_msgs::msg::Pose2D pose;
  pose.x = msg->pose.pose.position.x;
  pose.y = msg->pose.pose.position.y;

  tf2::Transform transform;
  tf2::fromMsg(msg->pose.pose, transform);

  double r, p, yaw;
  transform.getBasis().getRPY(r, p, yaw);
  pose.theta = yaw;

  query.set("enforceSeed", true);
  query.set("seedPose", RosMsgsDatagramConverter::makePose2d(pose));
  auto response = loc_client_interface_->call("clientLocalizationSetSeed", query);
}

bool LocatorBridgeNode::clientRecordingStartVisualRecordingCb(
  const std::shared_ptr<bosch_locator_bridge::srv::StartRecording::Request> req,
  std::shared_ptr<bosch_locator_bridge::srv::StartRecording::Response>/*res*/)
{
  auto query = loc_client_interface_->getSessionQuery();
  query.set("recordingName", req->name);  // TODO( ): rename srv attributes
  last_recording_name_ = req->name;
  auto response = loc_client_interface_->call("clientRecordingStartVisualRecording", query);
  return true;
}

bool LocatorBridgeNode::clientRecordingStopVisualRecordingCb(
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  auto query = loc_client_interface_->getSessionQuery();
  auto response = loc_client_interface_->call("clientRecordingStopVisualRecording", query);
  return true;
}

bool LocatorBridgeNode::clientMapStartCb(
  const std::shared_ptr<bosch_locator_bridge::srv::ClientMapStart::Request> req,
  std::shared_ptr<bosch_locator_bridge::srv::ClientMapStart::Response>/*res*/)
{
  const std::string recording_name =
    req->recording_name.empty() ? last_recording_name_ : req->recording_name;
  const std::string client_map_name = req->client_map_name.empty() ? "map-from-" +
    recording_name : req->client_map_name;

  auto query = loc_client_interface_->getSessionQuery();
  query.set("recordingName", recording_name);
  query.set("clientMapName", client_map_name);
  last_map_name_ = client_map_name;
  auto response = loc_client_interface_->call("clientMapStart", query);
  return true;
}

bool LocatorBridgeNode::clientMapStopCb(
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  auto query = loc_client_interface_->getSessionQuery();
  auto response = loc_client_interface_->call("clientMapStop", query);
  return true;
}

void LocatorBridgeNode::syncConfig()
{
  RCLCPP_INFO_STREAM(get_logger(), "syncing config");

  auto loc_client_config = loc_client_interface_->getConfigList();

  // overwrite current locator config with ros params

  std::string laser_type;
  declare_parameter("ClientSensor.laser.type", laser_type);
  get_parameter("ClientSensor.laser.type", laser_type);
  loc_client_config["ClientSensor.laser.type"] = laser_type;

  std::string laser_address;
  declare_parameter("ClientSensor.laser.address", laser_address);
  get_parameter("ClientSensor.laser.address", laser_address);
  loc_client_config["ClientSensor.laser.address"] = laser_address;

  bool laser_mirror_laser_scans = false;
  declare_parameter("ClientSensor.laser.mirrorLaserScans", laser_mirror_laser_scans);
  get_parameter("ClientSensor.laser.mirrorLaserScans", laser_mirror_laser_scans);
  loc_client_config["ClientSensor.laser.mirrorLaserScans"] = laser_mirror_laser_scans;

  double laser_vehicle_transform_laser_x = 0.0;
  declare_parameter("ClientSensor.laser.vehicleTransformLaser.x", laser_vehicle_transform_laser_x);
  get_parameter("ClientSensor.laser.vehicleTransformLaser.x", laser_vehicle_transform_laser_x);
  loc_client_config["ClientSensor.laser.vehicleTransformLaser.x"] = laser_vehicle_transform_laser_x;

  double laser_vehicle_transform_laser_y = 0.0;
  declare_parameter("ClientSensor.laser.vehicleTransformLaser.y", laser_vehicle_transform_laser_y);
  get_parameter("ClientSensor.laser.vehicleTransformLaser.y", laser_vehicle_transform_laser_y);
  loc_client_config["ClientSensor.laser.vehicleTransformLaser.y"] = laser_vehicle_transform_laser_y;

  double laser_vehicle_transform_laser_yaw = 0.0;  // angle in degrees
  declare_parameter(
    "ClientSensor.laser.vehicleTransformLaser.yaw",
    laser_vehicle_transform_laser_yaw);
  get_parameter("ClientSensor.laser.vehicleTransformLaser.yaw", laser_vehicle_transform_laser_yaw);
  loc_client_config["ClientSensor.laser.vehicleTransformLaser.yaw"] =
    laser_vehicle_transform_laser_yaw;

  bool laser_use_intensites = false;
  declare_parameter("ClientSensor.laser.useIntensities", laser_use_intensites);
  get_parameter("ClientSensor.laser.useIntensities", laser_use_intensites);
  loc_client_config["ClientSensor.laser.useIntensities"] = laser_use_intensites;

  bool enable_laser2 = false;
  declare_parameter("ClientSensor.enableLaser2", enable_laser2);
  get_parameter("ClientSensor.enableLaser2", enable_laser2);
  loc_client_config["ClientSensor.enableLaser2"] = enable_laser2;

  std::string laser2_type;
  declare_parameter("ClientSensor.laser2.type", laser2_type);
  get_parameter("ClientSensor.laser2.type", laser2_type);
  loc_client_config["ClientSensor.laser2.type"] = laser2_type;

  std::string laser2_address;
  declare_parameter("ClientSensor.laser2.address", laser2_address);
  get_parameter("ClientSensor.laser2.address", laser2_address);
  loc_client_config["ClientSensor.laser2.address"] = laser2_address;

  bool laser2_mirror_laser_scans = false;
  declare_parameter("ClientSensor.laser2.mirrorLaserScans", laser2_mirror_laser_scans);
  get_parameter("ClientSensor.laser2.mirrorLaserScans", laser2_mirror_laser_scans);
  loc_client_config["ClientSensor.laser2.mirrorLaserScans"] = laser2_mirror_laser_scans;

  double laser2_vehicle_transform_laser_x = 0.0;
  declare_parameter(
    "ClientSensor.laser2.vehicleTransformLaser.x",
    laser2_vehicle_transform_laser_x);
  get_parameter("ClientSensor.laser2.vehicleTransformLaser.x", laser2_vehicle_transform_laser_x);
  loc_client_config["ClientSensor.laser2.vehicleTransformLaser.x"] =
    laser2_vehicle_transform_laser_x;

  double laser2_vehicle_transform_laser_y = 0.0;
  declare_parameter(
    "ClientSensor.laser2.vehicleTransformLaser.y",
    laser2_vehicle_transform_laser_y);
  get_parameter("ClientSensor.laser2.vehicleTransformLaser.y", laser2_vehicle_transform_laser_y);
  loc_client_config["ClientSensor.laser2.vehicleTransformLaser.y"] =
    laser2_vehicle_transform_laser_y;

  double laser2_vehicle_transform_laser_yaw = 0.0;  // angle in degrees
  declare_parameter(
    "ClientSensor.laser2.vehicleTransformLaser.yaw",
    laser2_vehicle_transform_laser_yaw);
  get_parameter(
    "ClientSensor.laser2.vehicleTransformLaser.yaw",
    laser2_vehicle_transform_laser_yaw);
  loc_client_config["ClientSensor.laser2.vehicleTransformLaser.yaw"] =
    laser2_vehicle_transform_laser_yaw;

  bool laser2_use_intensites = false;
  declare_parameter("ClientSensor.laser2.useIntensities", laser2_use_intensites);
  get_parameter("ClientSensor.laser2.useIntensities", laser2_use_intensites);
  loc_client_config["ClientSensor.laser2.useIntensities"] = laser2_use_intensites;

  bool enable_reflector_markers = false;
  declare_parameter("ClientSensor.enableReflectorMarkers", enable_reflector_markers);
  get_parameter("ClientSensor.enableReflectorMarkers", enable_reflector_markers);
//  loc_client_config["ClientSensor.enableReflectorMarkers"] = enable_reflector_markers;

  bool autostart = false;
  declare_parameter("ClientLocalization.autostart", autostart);
  get_parameter("ClientLocalization.autostart", autostart);
  loc_client_config["ClientLocalization.autostart"] = autostart;

  bool odometry_enabled = false;
  declare_parameter("ClientSensor.enableOdometry", odometry_enabled);
  get_parameter("ClientSensor.enableOdometry", odometry_enabled);
  loc_client_config["ClientSensor.enableOdometry"] = odometry_enabled;

  bool odometry_tls = false;
  declare_parameter("ClientSensor.odometryEncryption", odometry_tls);
  get_parameter("ClientSensor.odometryEncryption", odometry_tls);
  loc_client_config["ClientSensor.odometryEncryption"] = odometry_tls;

  std::string odometry_address;
  declare_parameter("ClientSensor.odometryAddress", odometry_address);
  get_parameter("ClientSensor.odometryAddress", odometry_address);
  loc_client_config["ClientSensor.odometryAddress"] = odometry_address;

  RCLCPP_INFO_STREAM(get_logger(), "new loc client config: " << loc_client_config.toString());
  for (const auto & c : loc_client_config) {
    RCLCPP_INFO_STREAM(get_logger(), "- " << c.first << ": " << c.second.toString());
  }

  if (loc_client_config["ClientSensor.laser.type"].toString() == "simple") {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "ClientSensor.laser.type:" << loc_client_config["ClientSensor.laser.type"].toString() <<
        ". Will provide laser data.");
    provide_laser_data_ = true;
  } else {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "ClientSensor.laser.type:" << loc_client_config["ClientSensor.laser.type"].toString() <<
        ". Laser data will not be provided.");
    provide_laser_data_ = false;
  }

  if (loc_client_config["ClientSensor.enableLaser2"].toString() == "true" &&
    loc_client_config["ClientSensor.laser2.type"].toString() == "simple")
  {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "ClientSensor.laser2.type:" << loc_client_config["ClientSensor.laser2.type"].toString() <<
        ". Will provide laser2 data.");
    provide_laser2_data_ = true;
  } else {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "ClientSensor.laser2.type:" << loc_client_config["ClientSensor.laser2.type"].toString() <<
        ". Laser2 data will not be provided.");
    provide_laser2_data_ = false;
  }

  if (loc_client_config["ClientSensor.enableOdometry"].toString() == "true") {
    RCLCPP_INFO_STREAM(
      get_logger(), "ClientSensor.enableOdometry is set to true. Will provide odometry data.");
    provide_odometry_data_ = true;
  } else {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "ClientSensor.enableOdometry is set to false. Odometry data will not be provided.");
    provide_odometry_data_ = false;
  }

  loc_client_interface_->setConfigList(loc_client_config);
}

void LocatorBridgeNode::setupBinaryReceiverInterfaces(const std::string & host)
{
  // Create binary interface for client control mode
  client_control_mode_interface_.reset(
    new ClientControlModeInterface(
      Poco::Net::IPAddress(host),
      shared_from_this()));
  client_control_mode_interface_thread_.start(*client_control_mode_interface_);
  // Create binary interface for client map map
  client_map_map_interface_.reset(
    new ClientMapMapInterface(
      Poco::Net::IPAddress(host),
      shared_from_this()));
  client_map_map_interface_thread_.start(*client_map_map_interface_);
  // Create binary interface for client map visualization
  client_map_visualization_interface_.reset(
    new ClientMapVisualizationInterface(
      Poco::Net::IPAddress(
        host), shared_from_this()));
  client_map_visualization_interface_thread_.start(*client_map_visualization_interface_);
  // Create binary interface for client recording map
  client_recording_map_interface_.reset(
    new ClientRecordingMapInterface(
      Poco::Net::IPAddress(host),
      shared_from_this()));
  client_recording_map_interface_thread_.start(*client_recording_map_interface_);
  // Create binary interface for client recording visualization
  client_recording_visualization_interface_.reset(
    new ClientRecordingVisualizationInterface(Poco::Net::IPAddress(host), shared_from_this()));
  client_recording_visualization_interface_thread_.start(
    *client_recording_visualization_interface_);
  // Create binary interface for client localization map
  client_localization_map_interface_.reset(
    new ClientLocalizationMapInterface(
      Poco::Net::IPAddress(
        host), shared_from_this()));
  client_localization_map_interface_thread_.start(*client_localization_map_interface_);
  // Create binary interface for ClientLocalizationVisualizationInterface
  client_localization_visualization_interface_.reset(
    new ClientLocalizationVisualizationInterface(Poco::Net::IPAddress(host), shared_from_this()));
  client_localization_visualization_interface_thread_.start(
    *client_localization_visualization_interface_);
  // Create binary interface for ClientLocalizationPoseInterface
  client_localization_pose_interface_.reset(
    new ClientLocalizationPoseInterface(
      Poco::Net::IPAddress(
        host), shared_from_this()));
  client_localization_pose_interface_thread_.start(*client_localization_pose_interface_);
  // Create binary interface for ClientGlobalAlignVisualizationInterface
  client_global_align_visualization_interface_.reset(
    new ClientGlobalAlignVisualizationInterface(Poco::Net::IPAddress(host), shared_from_this()));
  client_global_align_visualization_interface_thread_.start(
    *client_global_align_visualization_interface_);
}
