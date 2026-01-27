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

#ifndef BOSCH_NAVIGATOR_BRIDGE__RECEIVING_INTERFACE_HPP_
#define BOSCH_NAVIGATOR_BRIDGE__RECEIVING_INTERFACE_HPP_

#include <Poco/Net/NetException.h>
#include <Poco/Net/SocketNotification.h>
#include <Poco/Net/SocketReactor.h>
#include <Poco/Net/StreamSocket.h>

#include <string>
#include <vector>


#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"


/**
 * @brief The ReceivingInterface class is the base class for all receiving interfaces, such as
 * ClientControlModeInterface, etc.
 */
class ReceivingInterface : public Poco::Runnable
{
public:
  /**
   * @brief Constructs a new ReceivingInterface object
   * @param hostadress The IP address of the Navigator to connect to
   * @param port The port number for the binary connection
   * @param node A shared pointer to the ROS 2 node
   */
  ReceivingInterface(
    const Poco::Net::IPAddress & hostadress, Poco::UInt16 port,
    rclcpp::Node::SharedPtr node);
  /**
   * @brief Destroys the ReceivingInterface object
   */
  virtual ~ReceivingInterface();

  /**
  * @brief Callback method invoked when data is available on the socket
  * @param notification A smart pointer to the ReadableNotification
  */
  virtual void onReadEvent(const Poco::AutoPtr<Poco::Net::ReadableNotification> & notification);

  /**
   * @brief The main execution loop for the receiving interface
   */
  void run();

protected:
  /**
   * @brief Actual function to be overwritten by child to handle data, e.g., convert to ros messages and
   * publish
   * @param datagram_buffer The data received via the binary connection socket
   * @return amount of bytes successfully parsed and can be removed from the buffer (0 if not parsing failed)
   */
  virtual size_t tryToParseData(
    const std::vector<char> & datagram_buffer,
    rclcpp::Node::SharedPtr node) = 0;

  //! Node
  rclcpp::Node::SharedPtr node_;

private:
  Poco::Net::StreamSocket ccm_socket_;
  Poco::Net::SocketReactor reactor_;
  // TODO(): use a better suited data structure (a deque?)
  std::vector<char> datagram_buffer_;
};

/**
 * @brief Specific receiving interface for Client Motion Commands
 *
 * This class inherits from ReceivingInterface and specializes in receiving
 * motion command data, which is then published as ROS geometry_msgs::msg::Twist messages
 */
class ClientMotionCommandInterface : public ReceivingInterface
{
public:
  /**
   * @brief Constructs a new ClientMotionCommandInterface object
   * @param hostadress The IP address of the Navigator
   * @param binaryClientNavigatorPort The specific port for Navigator motion commands
   * @param node A shared pointer to the ROS 2 node
   */
  ClientMotionCommandInterface(
    const Poco::Net::IPAddress & hostadress,
    const Poco::UInt16 binaryClientNavigatorPort,
    rclcpp::Node::SharedPtr node);
  /**
 * @brief Parses incoming raw motion command data
 * @param datagram The buffer containing raw binary motion command data
 * @param node A shared pointer to the ROS 2 node
 * @return The amount of bytes successfully parsed
 */
  size_t tryToParseData(
    const std::vector<char> & datagram,
    rclcpp::Node::SharedPtr node) override;

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>
  ::SharedPtr client_motion_command_pub_;
};


#endif  // BOSCH_NAVIGATOR_BRIDGE__RECEIVING_INTERFACE_HPP_
