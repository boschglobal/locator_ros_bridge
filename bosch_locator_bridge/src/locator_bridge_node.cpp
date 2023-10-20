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

#include <string>
#include <unordered_map>
#include <utility>

#include "locator_bridge_node.hpp"

#include "sending_interface.hpp"
#include "receiving_interface.hpp"
#include "rosmsgs_datagram_converter.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

/// locator module versions to check against. Format is name, { major_version, minor_version }
static const std::unordered_map<std::string, std::pair<int32_t, int32_t>> REQUIRED_MODULE_VERSIONS({
  {"AboutModules", {5, 0}},
  {"Session", {3, 1}},
//  {"Diagnostic", {5, 0}},
  {"Licensing", {7, 0}},
  {"Config", {7, 0}},
  {"AboutBuild", {3, 1}},
  {"Certificate", {3, 0}},
  {"System", {3, 2}},
//  {"ClientApplication", {1, 1}},
  {"ClientControl", {3, 2}},
  {"ClientRecording", {4, 0}},
  {"ClientMap", {4, 0}},
  {"ClientLocalization", {7, 1}},
//  {"ClientManualAlign", {5, 0}},
  {"ClientGlobalAlign", {4, 0}},
//  {"ClientLaserMask", {6, 0}},
  {"ClientSensor", {6, 0}},
//  {"ClientUser", {4, 0}},
//  {"User", {1, 0}},
  {"ClientExpandMap", {3, 0}},
});

LocatorBridgeNode::LocatorBridgeNode()
: nh_("~")
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
  nh_.getParam("locator_host", host);
  int tmp_binaryPortsStart, tmp_rpcPort;
  nh_.getParam("locator_binaryPortsStart", tmp_binaryPortsStart);
  uint16_t binaryPortsStart{static_cast<uint16_t>(tmp_binaryPortsStart)};
  nh_.getParam("locator_RPC_Port", tmp_rpcPort);
  uint16_t rpcPort{static_cast<uint16_t>(tmp_rpcPort)};

  std::string user, pwd;
  nh_.getParam("user_name", user);
  nh_.getParam("password", pwd);

  // NOTE for now, we only have a session management with the localization client
  // Same thing is likely needed for the map server
  loc_client_interface_.reset(new LocatorRPCInterface(host, rpcPort));
  loc_client_interface_->login(user, pwd);
  session_refresh_timer_ = nh_.createTimer(
    ros::Duration(30.), [&](const ros::TimerEvent &) {
      ROS_INFO_STREAM("refreshing session!");
      loc_client_interface_->refresh();
    });

  const auto module_versions = loc_client_interface_->getAboutModules();
  if (!check_module_versions(module_versions)) {
    throw std::runtime_error("locator software incompatible with this bridge!");
  }

  syncConfig();

  services_.push_back(
    nh_.advertiseService("get_config_entry", &LocatorBridgeNode::clientConfigGetEntryCb, this));

  services_.push_back(
    nh_.advertiseService(
      "start_visual_recording",
      &LocatorBridgeNode::clientRecordingStartVisualRecordingCb,
      this));
  services_.push_back(
    nh_.advertiseService(
      "stop_visual_recording",
      &LocatorBridgeNode::clientRecordingStopVisualRecordingCb,
      this));

  services_.push_back(
    nh_.advertiseService(
      "start_map",
      &LocatorBridgeNode::clientMapStartCb,
      this));
  services_.push_back(nh_.advertiseService("stop_map", &LocatorBridgeNode::clientMapStopCb, this));

  services_.push_back(
    nh_.advertiseService(
      "start_localization",
      &LocatorBridgeNode::clientLocalizationStartCb,
      this));
  services_.push_back(
    nh_.advertiseService(
      "stop_localization",
      &LocatorBridgeNode::clientLocalizationStopCb,
      this));

  services_.push_back(nh_.advertiseService("send_map", &LocatorBridgeNode::clientMapSendCb, this));
  services_.push_back(nh_.advertiseService("set_map", &LocatorBridgeNode::clientMapSetCb, this));
  services_.push_back(
    nh_.advertiseService(
      "list_client_maps",
      &LocatorBridgeNode::clientMapList,
      this));

  services_.push_back(
    nh_.advertiseService(
      "enable_map_expansion",
      &LocatorBridgeNode::clientExpandMapEnableCb,
      this));
  services_.push_back(
    nh_.advertiseService(
      "disable_map_expansion",
      &LocatorBridgeNode::clientExpandMapDisableCb,
      this));

  services_.push_back(
    nh_.advertiseService(
      "recording_set_current_pose",
      &LocatorBridgeNode::clientRecordingSetCurrentPoseCb,
      this));


  // subscribe to default topic published by rviz "2D Pose Estimate" button for setting seed
  set_seed_sub_ = nh_.subscribe("/initialpose", 1, &LocatorBridgeNode::setSeedCallback, this);

  // Create interface to send binary laser data if requested
  if (provide_laser_data_) {
    int laser_datagram_port;
    nh_.getParam("laser_datagram_port", laser_datagram_port);

    laser_sending_interface_.reset(new SendingInterface(laser_datagram_port));
    laser_sending_interface_thread_.start(*laser_sending_interface_);
    // Create subscriber to laser data
    std::string scan_topic = "";
    nh_.getParam("scan_topic", scan_topic);
    laser_sub_ = nh_.subscribe(scan_topic, 1, &LocatorBridgeNode::laser_callback, this);
  }

  // Create interface to send binary laser2 data if requested
  if (provide_laser2_data_) {
    int laser2_datagram_port;
    nh_.getParam("laser2_datagram_port", laser2_datagram_port);

    laser2_sending_interface_.reset(new SendingInterface(laser2_datagram_port));
    laser2_sending_interface_thread_.start(*laser2_sending_interface_);
    // Create subscriber to laser2 data
    std::string scan2_topic = "";
    nh_.getParam("scan2_topic", scan2_topic);
    laser2_sub_ = nh_.subscribe(scan2_topic, 1, &LocatorBridgeNode::laser2_callback, this);
  }

  // Create interface to send binary odometry data if requested
  if (provide_odometry_data_) {
    int odom_datagram_port;
    nh_.getParam("odom_datagram_port", odom_datagram_port);

    odom_sending_interface_.reset(new SendingInterface(odom_datagram_port));
    odom_sending_interface_thread_.start(*odom_sending_interface_);
    // Create subscriber to odometry data
    std::string odom_topic = "/odom";
    nh_.getParam("odom_topic", odom_topic);
    odom_sub_ = nh_.subscribe(odom_topic, 1, &LocatorBridgeNode::odom_callback, this);

    nh_.getParam("odometry_velocity_set", odometry_velocity_set_);
  }

  setupBinaryReceiverInterfaces(host, static_cast<Poco::UInt16>(binaryPortsStart));

  ROS_INFO_STREAM("initialization done");
}

bool LocatorBridgeNode::check_module_versions(
  const std::unordered_map<std::string, std::pair<int32_t, int32_t>> & module_versions)
{
  ROS_INFO("-----------------check_module_versions");
  for (const auto & required_pair : REQUIRED_MODULE_VERSIONS) {
    const auto & module_name = required_pair.first;
    const auto & required_version = required_pair.second;

    const auto & actual_version_iter = module_versions.find(module_name);
    if (actual_version_iter == module_versions.end()) {
      ROS_WARN_STREAM("required locator module " << module_name << " not found!");
      return false;
    }
    const auto & actual_version = actual_version_iter->second;
    // major version number needs to match, minor version number equal or bigger
    if ((actual_version.first == required_version.first) &&
      (actual_version.second >= required_version.second))
    {
      ROS_DEBUG_STREAM("locator module " << module_name << ": version ok!");
    } else {
      ROS_WARN_STREAM(
        "---------8 module: " << module_name << " required version: " <<
          required_version.first << "." <<
          required_version.second << " (actual version: " <<
          actual_version.first <<
          "." << actual_version.second << ")");
      return false;
    }
  }
  return true;
}

void LocatorBridgeNode::laser_callback(const sensor_msgs::LaserScan & msg)
{
  // If scan_time is not set, use timestamp difference to set it.
  float scan_time = 0.0f;
  if (!msg.scan_time) {
    ros::Time laser_timestamp = msg.header.stamp;
    if (prev_laser_timestamp_.toSec() != 0.0) {
      scan_time = (laser_timestamp - prev_laser_timestamp_).toSec();
    }
    prev_laser_timestamp_ = laser_timestamp;
  }

  Poco::Buffer<char> laserscan_datagram =
    RosMsgsDatagramConverter::convertLaserScan2DataGram(msg, ++scan_num_, scan_time);
  if (laser_sending_interface_->sendData(laserscan_datagram.begin(), laserscan_datagram.size()) ==
    SendingInterface::SendingStatus::IO_EXCEPTION)
  {
    checkLaserScan(msg, "laser");
  }
}

void LocatorBridgeNode::laser2_callback(const sensor_msgs::LaserScan & msg)
{
  // If scan_time is not set, use timestamp difference to set it.
  float scan_time = 0.0f;
  if (!msg.scan_time) {
    ros::Time laser_timestamp = msg.header.stamp;
    if (prev_laser2_timestamp_.toSec() != 0.0) {
      scan_time = (laser_timestamp - prev_laser2_timestamp_).toSec();
    }
    prev_laser2_timestamp_ = laser_timestamp;
  }

  Poco::Buffer<char> laserscan_datagram =
    RosMsgsDatagramConverter::convertLaserScan2DataGram(msg, ++scan2_num_, scan_time);
  if (laser2_sending_interface_->sendData(laserscan_datagram.begin(), laserscan_datagram.size()) ==
    SendingInterface::SendingStatus::IO_EXCEPTION)
  {
    checkLaserScan(msg, "laser2");
  }
}

void LocatorBridgeNode::odom_callback(const nav_msgs::Odometry & msg)
{
  Poco::Buffer<char> odom_datagram =
    RosMsgsDatagramConverter::convertOdometry2DataGram(msg, ++odom_num_, odometry_velocity_set_);
  odom_sending_interface_->sendData(odom_datagram.begin(), odom_datagram.size());
}

bool LocatorBridgeNode::clientConfigGetEntryCb(
  bosch_locator_bridge::ClientConfigGetEntry::Request & req,
  bosch_locator_bridge::ClientConfigGetEntry::Response & res)
{
  return get_config_entry(req.name, res.value);
}

bool LocatorBridgeNode::clientMapSendCb(
  bosch_locator_bridge::ClientMapSend::Request & req,
  bosch_locator_bridge::ClientMapSend::Response & res)
{
  const std::string client_map_name = req.name.empty() ? last_map_name_ : req.name;

  auto query = loc_client_interface_->getSessionQuery();
  query.set("clientMapName", client_map_name);
  auto response = loc_client_interface_->call("clientMapSend", query);
  return true;
}

bool LocatorBridgeNode::clientMapSetCb(
  bosch_locator_bridge::ClientMapSet::Request & req,
  bosch_locator_bridge::ClientMapSet::Response & res)
{
  const std::string active_map_name = req.name.empty() ? last_map_name_ : req.name;

  Poco::DynamicStruct config;
  config.insert("ClientLocalization.activeMapName", active_map_name);
  loc_client_interface_->setConfigList(config);
  return true;
}

bool LocatorBridgeNode::clientMapList(
  bosch_locator_bridge::ClientMapList::Request & req,
  bosch_locator_bridge::ClientMapList::Response & res)
{
  auto query = loc_client_interface_->getSessionQuery();
  auto response = loc_client_interface_->call("clientMapList", query);
  if (response.has("clientMapNames")) {
    const auto entries = response.getArray("clientMapNames");
    for (size_t i = 0; i < entries->size(); i++) {
      res.names.push_back(entries->get(i).toString());
    }
  }
  return true;
}

bool LocatorBridgeNode::clientLocalizationStartCb(
  std_srvs::Empty::Request & req,
  std_srvs::Empty::Response & res)
{
  auto query = loc_client_interface_->getSessionQuery();
  auto response = loc_client_interface_->call("clientLocalizationStart", query);
  return true;
}

bool LocatorBridgeNode::clientLocalizationStopCb(
  std_srvs::Empty::Request & req,
  std_srvs::Empty::Response & res)
{
  auto query = loc_client_interface_->getSessionQuery();
  auto response = loc_client_interface_->call("clientLocalizationStop", query);
  return true;
}

bool LocatorBridgeNode::clientExpandMapEnableCb(
  bosch_locator_bridge::ClientExpandMapEnable::Request & req,
  bosch_locator_bridge::ClientExpandMapEnable::Response & res)
{
  const std::string prior_map_name =
    req.prior_map_name.empty() ? last_map_name_ : req.prior_map_name;

  auto query = loc_client_interface_->getSessionQuery();
  query.set("priorMapName", prior_map_name);
  auto response = loc_client_interface_->call("clientExpandMapEnable", query);
  return true;
}

bool LocatorBridgeNode::clientExpandMapDisableCb(
  std_srvs::Empty::Request & req,
  std_srvs::Empty::Response & res)
{
  auto query = loc_client_interface_->getSessionQuery();
  auto response = loc_client_interface_->call("clientExpandMapDisable", query);
  return true;
}

bool LocatorBridgeNode::clientRecordingSetCurrentPoseCb(
  bosch_locator_bridge::ClientRecordingSetCurrentPose::Request & req,
  bosch_locator_bridge::ClientRecordingSetCurrentPose::Response & res)
{
  auto query = loc_client_interface_->getSessionQuery();

  query.set("pose", RosMsgsDatagramConverter::makePose2d(req.pose));
  auto response = loc_client_interface_->call("clientRecordingSetCurrentPose", query);
  return true;
}

void LocatorBridgeNode::setSeedCallback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
  if (msg.header.frame_id != MAP_FRAME_ID) {
    ROS_ERROR_STREAM(
      "2D Pose Estimate sent in wrong frame! Is: " <<
        msg.header.frame_id << " but should be " <<
        MAP_FRAME_ID);
    return;
  }
  auto query = loc_client_interface_->getSessionQuery();
  geometry_msgs::Pose2D pose;
  pose.x = msg.pose.pose.position.x;
  pose.y = msg.pose.pose.position.y;

  tf2::Transform transform;
  tf2::fromMsg(msg.pose.pose, transform);

  double r, p, yaw;
  transform.getBasis().getRPY(r, p, yaw);
  pose.theta = yaw;

  query.set("enforceSeed", true);
  query.set("seedPose", RosMsgsDatagramConverter::makePose2d(pose));
  auto response = loc_client_interface_->call("clientLocalizationSetSeed", query);
}

bool LocatorBridgeNode::clientRecordingStartVisualRecordingCb(
  bosch_locator_bridge::StartRecording::Request & req,
  bosch_locator_bridge::StartRecording::Response & res)
{
  auto query = loc_client_interface_->getSessionQuery();
  query.set("recordingName", req.name);  // TODO( ): rename srv attributes
  last_recording_name_ = req.name;
  auto response = loc_client_interface_->call("clientRecordingStartVisualRecording", query);
  return true;
}

bool LocatorBridgeNode::clientRecordingStopVisualRecordingCb(
  std_srvs::Empty::Request & req,
  std_srvs::Empty::Response & res)
{
  auto query = loc_client_interface_->getSessionQuery();
  auto response = loc_client_interface_->call("clientRecordingStopVisualRecording", query);
  return true;
}

bool LocatorBridgeNode::clientMapStartCb(
  bosch_locator_bridge::ClientMapStart::Request & req,
  bosch_locator_bridge::ClientMapStart::Response & res)
{
  const std::string recording_name =
    req.recording_name.empty() ? last_recording_name_ : req.recording_name;
  const std::string client_map_name =
    req.client_map_name.empty() ? "map-from-" + recording_name : req.client_map_name;

  auto query = loc_client_interface_->getSessionQuery();
  query.set("recordingName", recording_name);
  query.set("clientMapName", client_map_name);
  last_map_name_ = client_map_name;
  auto response = loc_client_interface_->call("clientMapStart", query);
  return true;
}

bool LocatorBridgeNode::clientMapStopCb(
  std_srvs::Empty::Request & req,
  std_srvs::Empty::Response & res)
{
  auto query = loc_client_interface_->getSessionQuery();
  auto response = loc_client_interface_->call("clientMapStop", query);
  return true;
}

void LocatorBridgeNode::syncConfig()
{
  ROS_INFO_STREAM("syncing config");
  XmlRpc::XmlRpcValue localization_client_rosconfig;
  nh_.getParam("localization_client_config", localization_client_rosconfig);

  auto loc_client_config = loc_client_interface_->getConfigList();
  // overwrite current locator config with ros params
  for (auto & iter : localization_client_rosconfig) {
    const auto & key = iter.first;
    auto & value = iter.second;
    if (!loc_client_config.contains(key)) {
      ROS_WARN_STREAM("invalid locator rosparam found: " << key << ", " << value);
      continue;
    }
    ROS_INFO_STREAM("setting value for " << key << ", " << value);
    switch (value.getType()) {
      case XmlRpc::XmlRpcValue::TypeBoolean:
        loc_client_config[key] = static_cast<bool>(value);
        break;
      case XmlRpc::XmlRpcValue::TypeInt:
        loc_client_config[key] = static_cast<int>(value);
        break;
      case XmlRpc::XmlRpcValue::TypeDouble:
        loc_client_config[key] = static_cast<double>(value);
        break;
      case XmlRpc::XmlRpcValue::TypeString:
        loc_client_config[key] = static_cast<std::string>(value);
        break;
      case XmlRpc::XmlRpcValue::TypeArray:
        if (value.size() == 0) {
          // for an empty array the type does not matter here, we arbitrarily choose double
          loc_client_config[key] = convert_value_array_to_vector<double>(value);
        } else {
          switch (value[0].getType()) {
            case XmlRpc::XmlRpcValue::TypeBoolean:
              {
                loc_client_config[key] = convert_value_array_to_vector<bool>(value);
                break;
              }
            case XmlRpc::XmlRpcValue::TypeInt:
              {
                loc_client_config[key] = convert_value_array_to_vector<int>(value);
                break;
              }
            case XmlRpc::XmlRpcValue::TypeDouble:
              {
                loc_client_config[key] = convert_value_array_to_vector<double>(value);
                break;
              }
            case XmlRpc::XmlRpcValue::TypeString:
              {
                loc_client_config[key] = convert_value_array_to_vector<std::string>(value);
                break;
              }
            default:
              ROS_ERROR_STREAM("unknown element type for " << key);
              break;
          }
        }
        break;
      default:
        ROS_ERROR_STREAM("unknown config type for " << key);
        break;
    }
  }
  ROS_INFO_STREAM("new loc client config: " << loc_client_config.toString());
  for (const auto & c : loc_client_config) {
    ROS_INFO_STREAM("- " << c.first << ": " << c.second.toString());
  }

  if (loc_client_config["ClientSensor.laser.type"].toString() == "simple") {
    ROS_INFO_STREAM(
      "ClientSensor.laser.type:" <<
        loc_client_config["ClientSensor.laser.type"].toString() <<
        ". Will provide laser data.");
    provide_laser_data_ = true;
  } else {
    ROS_INFO_STREAM(
      "ClientSensor.laser.type:" <<
        loc_client_config["ClientSensor.laser.type"].toString() <<
        ". Laser data will not be provided.");
    provide_laser_data_ = false;
  }

  if (loc_client_config["ClientSensor.enableLaser2"].toString() == "true" &&
    loc_client_config["ClientSensor.laser2.type"].toString() == "simple")
  {
    ROS_INFO_STREAM(
      "ClientSensor.laser2.type:" <<
        loc_client_config["ClientSensor.laser2.type"].toString() <<
        ". Will provide laser2 data.");
    provide_laser2_data_ = true;
  } else {
    ROS_INFO_STREAM(
      "ClientSensor.laser2.type:" <<
        loc_client_config["ClientSensor.laser2.type"].toString() <<
        ". Laser2 data will not be provided.");
    provide_laser2_data_ = false;
  }

  if (loc_client_config["ClientSensor.enableOdometry"].toString() == "true") {
    ROS_INFO_STREAM("ClientSensor.enableOdometry is set to true. Will provide odometry data.");
    provide_odometry_data_ = true;
  } else {
    ROS_INFO_STREAM(
      "ClientSensor.enableOdometry is set to false."
      " Odometry data will not be provided.");
    provide_odometry_data_ = false;
  }

  if (!loc_client_interface_->setConfigList(loc_client_config)) {
    // Try to stop everything before setting config list
    ROS_WARN(
      "One of the modes appears to be in a RUN-state. In order to set the configuration parameters,"
      " all modes are now stopped! Localization is started afterwards automatically.");
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    clientRecordingStopVisualRecordingCb(req, res);
    clientMapStopCb(req, res);
    clientLocalizationStopCb(req, res);
    loc_client_interface_->setConfigList(loc_client_config);
  }
}

void LocatorBridgeNode::checkLaserScan(
  const sensor_msgs::LaserScan & msg,
  const std::string & laser) const
{
  if (fabs(msg.angle_min + (msg.ranges.size() - 1) * msg.angle_increment - msg.angle_max) >
    fabs(0.5 * msg.angle_increment))
  {
    ROS_ERROR_STREAM(
      "LaserScan message is INVALID: " << msg.angle_min << " (angle_min) + " <<
      (msg.ranges.size() - 1) << " (ranges.size - 1) * " << msg.angle_increment <<
        " (angle_increment) = " <<
      (msg.angle_min + (msg.ranges.size() - 1) * msg.angle_increment) << ", expected " <<
        msg.angle_max << " (angle_max)");
  } else {
    const std::string param_name = "ClientSensor." + laser + ".useIntensities";
    std::string laser_use_intensities;
    if (get_config_entry(
        param_name, laser_use_intensities) &&
      laser_use_intensities == "true" && msg.ranges.size() != msg.intensities.size())
    {
      ROS_ERROR_STREAM(
        "LaserScan message is INVALID: " << param_name << " is true, but ranges.size (" <<
          msg.ranges.size() << ") unequal intensities.size (" << msg.intensities.size() << ")");
    }
  }
}

void LocatorBridgeNode::setupBinaryReceiverInterfaces(
  const std::string & host,
  const Poco::UInt16 binaryPortsStart)
{
  // port definitions for the different interfaces. See Locator API documentation section 12.8
  Poco::UInt16 binaryClientControlModePort{binaryPortsStart /*default: 9004*/};
  Poco::UInt16 binaryClientMapMapPort{static_cast<Poco::UInt16>(binaryPortsStart + 1)};
  Poco::UInt16 binaryClientMapVisualizationPort{static_cast<Poco::UInt16>(binaryPortsStart + 2)};
  Poco::UInt16 binaryClientRecordingMapPort{static_cast<Poco::UInt16>(binaryPortsStart + 3)};
  Poco::UInt16 binaryClientRecordingVisualizationPort{
    static_cast<Poco::UInt16>(binaryPortsStart + 4)};
  Poco::UInt16 binaryClientLocalizationMapPort{
    static_cast<Poco::UInt16>(binaryPortsStart + 5)};
  Poco::UInt16 binaryClientLocalizationVisualizationPort{
    static_cast<Poco::UInt16>(binaryPortsStart + 6)};
  Poco::UInt16 binaryClientLocalizationPosePort{
    static_cast<Poco::UInt16>(binaryPortsStart + 7)};
  Poco::UInt16 binaryClientGlobalAlignVisualizationPort{
    static_cast<Poco::UInt16>(binaryPortsStart + 8)};
  Poco::UInt16 binaryClientExpandMapVisualizationPort{
    static_cast<Poco::UInt16>(binaryPortsStart + 9)};
  Poco::UInt16 binaryClientExpandMapPriorMapPort{
    static_cast<Poco::UInt16>(binaryPortsStart + 10)};

  // Create binary interface for client control mode
  client_control_mode_interface_.reset(
    new ClientControlModeInterface(Poco::Net::IPAddress(host), binaryClientControlModePort, nh_));
  client_control_mode_interface_thread_.start(*client_control_mode_interface_);
  // Create binary interface for client map map
  client_map_map_interface_.reset(
    new ClientMapMapInterface(Poco::Net::IPAddress(host), binaryClientMapMapPort, nh_));
  client_map_map_interface_thread_.start(*client_map_map_interface_);
  // Create binary interface for client map visualization
  client_map_visualization_interface_.reset(
    new ClientMapVisualizationInterface(
      Poco::Net::IPAddress(host),
      binaryClientMapVisualizationPort, nh_));
  client_map_visualization_interface_thread_.start(*client_map_visualization_interface_);
  // Create binary interface for client recording map
  client_recording_map_interface_.reset(
    new ClientRecordingMapInterface(Poco::Net::IPAddress(host), binaryClientRecordingMapPort, nh_));
  client_recording_map_interface_thread_.start(*client_recording_map_interface_);
  // Create binary interface for client recording visualization
  client_recording_visualization_interface_.reset(
    new ClientRecordingVisualizationInterface(
      Poco::Net::IPAddress(host),
      binaryClientRecordingVisualizationPort, nh_));
  client_recording_visualization_interface_thread_.start(
    *client_recording_visualization_interface_);
  // Create binary interface for client localization map
  client_localization_map_interface_.reset(
    new ClientLocalizationMapInterface(
      Poco::Net::IPAddress(host),
      binaryClientLocalizationMapPort, nh_));
  client_localization_map_interface_thread_.start(*client_localization_map_interface_);
  // Create binary interface for ClientLocalizationVisualizationInterface
  client_localization_visualization_interface_.reset(
    new ClientLocalizationVisualizationInterface(
      Poco::Net::IPAddress(host),
      binaryClientLocalizationVisualizationPort, nh_));
  client_localization_visualization_interface_thread_.start(
    *client_localization_visualization_interface_);
  // Create binary interface for ClientLocalizationPoseInterface
  client_localization_pose_interface_.reset(
    new ClientLocalizationPoseInterface(
      Poco::Net::IPAddress(host),
      binaryClientLocalizationPosePort, nh_));
  client_localization_pose_interface_thread_.start(*client_localization_pose_interface_);
  // Create binary interface for ClientGlobalAlignVisualizationInterface
  client_global_align_visualization_interface_.reset(
    new ClientGlobalAlignVisualizationInterface(
      Poco::Net::IPAddress(host),
      binaryClientGlobalAlignVisualizationPort, nh_));
  client_global_align_visualization_interface_thread_.start(
    *client_global_align_visualization_interface_);

  // Create binary interface for ClientExpandMapVisualizationInterface
  client_expandmap_visualization_interface_.reset(
    new ClientExpandMapVisualizationInterface(
      Poco::Net::IPAddress(host),
      binaryClientExpandMapVisualizationPort, nh_));
  client_expandmap_visualization_interface_thread_.start(
    *client_expandmap_visualization_interface_);

  // Create binary interface for ClientExpandMapPriorMapInterface
  client_expandmap_priormap_interface_.reset(
    new ClientExpandMapPriorMapInterface(
      Poco::Net::IPAddress(host),
      binaryClientExpandMapPriorMapPort, nh_));
  client_expandmap_priormap_interface_thread_.start(*client_expandmap_priormap_interface_);
}
