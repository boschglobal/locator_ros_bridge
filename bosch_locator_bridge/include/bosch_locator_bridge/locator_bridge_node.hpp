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

#ifndef BOSCH_LOCATOR_BRIDGE__LOCATOR_BRIDGE_NODE_HPP_
#define BOSCH_LOCATOR_BRIDGE__LOCATOR_BRIDGE_NODE_HPP_

#include <Poco/Thread.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "bosch_locator_bridge/srv/client_config_get_entry.hpp"
#include "bosch_locator_bridge/srv/client_expand_map_enable.hpp"
#include "bosch_locator_bridge/srv/client_recording_set_current_pose.hpp"
#include "bosch_locator_bridge/srv/client_map_list.hpp"
#include "bosch_locator_bridge/srv/client_map_send.hpp"
#include "bosch_locator_bridge/srv/client_map_set.hpp"
#include "bosch_locator_bridge/srv/client_map_start.hpp"
#include "bosch_locator_bridge/srv/start_recording.hpp"

#include "locator_rpc_interface.hpp"

// forward declarations
class LocatorRPCInterface;
class SendingInterface;
class ClientControlModeInterface;
class ClientMapMapInterface;
class ClientMapVisualizationInterface;
class ClientRecordingMapInterface;
class ClientRecordingVisualizationInterface;
class ClientLocalizationMapInterface;
class ClientLocalizationVisualizationInterface;
class ClientLocalizationPoseInterface;
class ClientGlobalAlignVisualizationInterface;
class ClientExpandMapVisualizationInterface;
class ClientExpandMapPriorMapInterface;

/**
 * This is the main ROS node. It binds together the ROS interface and the Locator API.
 */
class LocatorBridgeNode : public rclcpp::Node
{
public:
  explicit LocatorBridgeNode(const std::string & nodeName);
  ~LocatorBridgeNode();

  void init();

private:
  bool check_module_versions(
    const std::unordered_map<std::string, std::pair<int32_t,
    int32_t>> & module_versions);
  template<typename T>
  bool get_config_entry(const std::string & name, T & value) const;
  template<typename T>
  bool set_config_entry(const std::string & name, const T & value) const;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void laser2_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  bool clientConfigGetEntryCb(
    const std::shared_ptr<bosch_locator_bridge::srv::ClientConfigGetEntry::Request> req,
    std::shared_ptr<bosch_locator_bridge::srv::ClientConfigGetEntry::Response> res);

  bool clientMapSendCb(
    const std::shared_ptr<bosch_locator_bridge::srv::ClientMapSend::Request> req,
    std::shared_ptr<bosch_locator_bridge::srv::ClientMapSend::Response> res);
  bool clientMapSetCb(
    const std::shared_ptr<bosch_locator_bridge::srv::ClientMapSet::Request> req,
    std::shared_ptr<bosch_locator_bridge::srv::ClientMapSet::Response> res);
  bool clientMapList(
    const std::shared_ptr<bosch_locator_bridge::srv::ClientMapList::Request> req,
    std::shared_ptr<bosch_locator_bridge::srv::ClientMapList::Response> res);

  bool clientLocalizationStartCb(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);
  bool clientLocalizationStopCb(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);

  void setSeedCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  bool clientRecordingStartVisualRecordingCb(
    const std::shared_ptr<bosch_locator_bridge::srv::StartRecording::Request> req,
    std::shared_ptr<bosch_locator_bridge::srv::StartRecording::Response> res);
  bool clientRecordingStopVisualRecordingCb(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);

  bool clientMapStartCb(
    const std::shared_ptr<bosch_locator_bridge::srv::ClientMapStart::Request> req,
    std::shared_ptr<bosch_locator_bridge::srv::ClientMapStart::Response> res);
  bool clientMapStopCb(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);

  bool clientExpandMapEnableCb(
    const std::shared_ptr<bosch_locator_bridge::srv::ClientExpandMapEnable::Request> req,
    std::shared_ptr<bosch_locator_bridge::srv::ClientExpandMapEnable::Response> res);
  bool clientExpandMapDisableCb(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);

  bool clientRecordingSetCurrentPoseCb(
    const std::shared_ptr<bosch_locator_bridge::srv::ClientRecordingSetCurrentPose::Request> req,
    std::shared_ptr<bosch_locator_bridge::srv::ClientRecordingSetCurrentPose::Response> res);

  /// read out ROS parameters and use them to update the locator config
  void syncConfig();

  /// Check if laser scan message is valid
  void checkLaserScan(
    const sensor_msgs::msg::LaserScan::SharedPtr msg,
    const std::string & laser) const;
  void setupBinaryReceiverInterfaces(const std::string & host, const Poco::UInt16 binaryPortsStart);

  std::unique_ptr<LocatorRPCInterface> loc_client_interface_;

  rclcpp::TimerBase::SharedPtr session_refresh_timer_;

  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  rclcpp::CallbackGroup::SharedPtr callback_group_services_;

  // Flag to indicate if the bridge should send odometry data to the locator.
  // Value retrieved by the locator settings.
  bool provide_odometry_data_;
  bool odometry_velocity_set_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  std::unique_ptr<SendingInterface> laser_sending_interface_;
  Poco::Thread laser_sending_interface_thread_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser2_sub_;
  std::unique_ptr<SendingInterface> laser2_sending_interface_;
  Poco::Thread laser2_sending_interface_thread_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_seed_sub_;

  // Flag to indicate if the bridge should send odometry data to the locator.
  // Value retrieved by the locator settings.
  bool provide_laser_data_;
  bool provide_laser2_data_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<SendingInterface> odom_sending_interface_;
  Poco::Thread odom_sending_interface_thread_;

  //! Binary interfaces and according threads
  std::unique_ptr<ClientControlModeInterface> client_control_mode_interface_;
  Poco::Thread client_control_mode_interface_thread_;
  std::unique_ptr<ClientMapMapInterface> client_map_map_interface_;
  Poco::Thread client_map_map_interface_thread_;
  std::unique_ptr<ClientMapVisualizationInterface> client_map_visualization_interface_;
  Poco::Thread client_map_visualization_interface_thread_;
  std::unique_ptr<ClientRecordingMapInterface> client_recording_map_interface_;
  Poco::Thread client_recording_map_interface_thread_;
  std::unique_ptr<ClientRecordingVisualizationInterface>
  client_recording_visualization_interface_;
  Poco::Thread client_recording_visualization_interface_thread_;
  std::unique_ptr<ClientLocalizationMapInterface> client_localization_map_interface_;
  Poco::Thread client_localization_map_interface_thread_;
  std::unique_ptr<ClientLocalizationVisualizationInterface>
  client_localization_visualization_interface_;
  Poco::Thread client_localization_visualization_interface_thread_;
  std::unique_ptr<ClientLocalizationPoseInterface> client_localization_pose_interface_;
  Poco::Thread client_localization_pose_interface_thread_;
  std::unique_ptr<ClientGlobalAlignVisualizationInterface>
  client_global_align_visualization_interface_;
  Poco::Thread client_global_align_visualization_interface_thread_;
  std::unique_ptr<ClientExpandMapVisualizationInterface> client_expandmap_visualization_interface_;
  Poco::Thread client_expandmap_visualization_interface_thread_;
  std::unique_ptr<ClientExpandMapPriorMapInterface> client_expandmap_priormap_interface_;
  Poco::Thread client_expandmap_priormap_interface_thread_;

  size_t scan_num_ {0};
  size_t scan2_num_ {0};
  size_t odom_num_ {0};

  std::string last_recording_name_;
  std::string last_map_name_;

  rclcpp::Time prev_laser_timestamp_;
  rclcpp::Time prev_laser2_timestamp_;
};

template<typename T>
bool LocatorBridgeNode::get_config_entry(const std::string & name, T & value) const
{
  const auto & loc_client_config = loc_client_interface_->getConfigList();

  try {
    loc_client_config[name].convert(value);
  } catch (const Poco::NotFoundException & error) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not find config entry " << name << ".");
    return false;
  }

  return true;
}

template<typename T>
bool LocatorBridgeNode::set_config_entry(const std::string & name, const T & value) const
{
  auto loc_client_config = loc_client_interface_->getConfigList();

  try {
    loc_client_config[name] = value;
  } catch (const Poco::NotFoundException & error) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not find config entry " << name << ".");
    return false;
  }

  loc_client_interface_->setConfigList(loc_client_config);
  return true;
}

#endif  // BOSCH_LOCATOR_BRIDGE__LOCATOR_BRIDGE_NODE_HPP_
