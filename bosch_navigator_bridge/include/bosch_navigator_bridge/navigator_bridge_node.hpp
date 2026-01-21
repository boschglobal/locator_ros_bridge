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

#ifndef BOSCH_NAVIGATOR_BRIDGE__NAVIGATOR_BRIDGE_NODE_HPP_
#define BOSCH_NAVIGATOR_BRIDGE__NAVIGATOR_BRIDGE_NODE_HPP_

#include <Poco/Thread.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>


#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navigator_interface.hpp"


// forward declarations
class SendingInterface;
class ReceivingInterface;
class ClientMotionCommandInterface;


/**
 * This is the main ROS node. It binds together the ROS interface and the Navigator API.
 */
class NavigatorBridgeNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructs a new NavigatorBridgeNode object
   * @param nodeName The name of the ROS node
   */
  explicit NavigatorBridgeNode(const std::string & nodeName);

  /**
   * @brief Destroys the NavigatorBridgeNode object
   *
   * Cleans up resources
   */
  ~NavigatorBridgeNode();

  /**
   * @brief Initializes the NavigatorBridgeNode.
   *
   * Sets up ROS publishers, subscribers, services, and initializes
   * the connection to the Navigator API. This method should be called
   * after construction
   */
  void init();

private:
  bool check_module_versions(
    const std::unordered_map<std::string, std::pair<int32_t,
    int32_t>> & module_versions);
  /**
 * @brief Retrieves a configuration entry
 *
 * This is a templated helper function to read various types of configuration
 * entries by their name
 *
 * @tparam T The type of the configuration entry value
 * @param name The name of the configuration entry
 * @param value A reference where the retrieved value will be stored
 * @return True if the configuration entry was found and retrieved successfully, false otherwise
 */
  template<typename T>
  bool get_config_entry(const std::string & name, T & value) const;
  /**
   * @brief Sets a configuration entry
   *
   * This is a templated helper function to write various types of configuration
   * entries by their name
   *
   * @tparam T The type of the configuration entry value
   * @param name The name of the configuration entry
   * @param value The value to set for the configuration entry
   * @return True if the configuration entry was found and set successfully, false otherwise
   */
  template<typename T>
  bool set_config_entry(const std::string & name, const T & value) const;

  /**
   * @brief Callback function for incoming odometry messages
   *
   * Processes the received odometry data and forwards it to the Navigator if enabled
   * @param msg A shared pointer to the received nav_msgs::msg::Odometry message
   */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /// read out ROS parameters and use them to update the Navigator config

  /**
   * @brief Reads ROS parameters and updates the Navigator's configuration
   *
   * This method ensures that the Navigator's internal configuration is synchronized
   * with the ROS parameters defined for this node
   */
  void syncConfig();

  /**
   * @brief Sets up the binary receiver interfaces for data streaming
   *
   * Initializes and starts the binary interface
   * @param host The hostname or IP address of the Navigator
   * @param binaryPortsStart The starting port number for the binary interfaces
   */
  void setupBinaryReceiverInterfaces(const std::string & host, const Poco::UInt16 binaryPortsStart);

  std::unique_ptr<NavigatorInterface> nav_client_interface_;

  rclcpp::TimerBase::SharedPtr session_refresh_timer_;

  rclcpp::CallbackGroup::SharedPtr callback_group_services_;


  // Value retrieved by the Navigator settings.
  bool odometry_pose_set_;


  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<SendingInterface> odom_sending_interface_;
  Poco::Thread odom_sending_interface_thread_;

  //! Binary interfaces and according threads
  std::unique_ptr<ClientMotionCommandInterface> client_motion_command_interface_;
  Poco::Thread client_motion_command_interface_thread_;

  size_t odom_num_ {0};
};

// Retrieves the value of a configuration entry by its name and stores it in the provided variable.
template<typename T>
bool NavigatorBridgeNode::get_config_entry(const std::string & name, T & value) const
{
  const auto & nav_client_config = nav_client_interface_->getConfigList();

  try {
    nav_client_config[name].convert(value);
  } catch (const Poco::NotFoundException & error) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not find config entry " << name << ".");
    return false;
  }

  return true;
}

// Sets the value of a configuration entry by its name to the provided value.
template<typename T>
bool NavigatorBridgeNode::set_config_entry(const std::string & name, const T & value) const
{
  auto nav_client_config = nav_client_interface_->getConfigList();

  try {
    nav_client_config[name] = value;
  } catch (const Poco::NotFoundException & error) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not find config entry " << name << ".");
    return false;
  }

  nav_client_interface_->setConfigList(nav_client_config);
  return true;
}

#endif  // BOSCH_NAVIGATOR_BRIDGE__NAVIGATOR_BRIDGE_NODE_HPP_
