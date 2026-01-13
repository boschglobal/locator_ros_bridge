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

#ifndef BOSCH_NAVIGATOR_BRIDGE__SENDING_INTERFACE_HPP_
#define BOSCH_NAVIGATOR_BRIDGE__SENDING_INTERFACE_HPP_

#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/StreamSocket.h>
#include <Poco/Buffer.h>

#include <atomic>
#include <mutex>
#include <vector>

#include "rclcpp/node.hpp"

/**
 * @brief Interface for sending binary data to "push" consumers (clients)
 *
 * This class facilitates communication with clients that actively connect
 * to receive data streams, such as sensor data (e.g., laser scans) or odometry
 * It operates as a server, accepting connections and sending data to all connected clients
 */
class SendingInterface : public Poco::Runnable
{
public:
  /**
   * @brief Constructs a new SendingInterface object
   * @param port The port number on which the server socket will listen for incoming client connections
   * @param node A shared pointer to the ROS 2 node, used for logging
   */
  SendingInterface(uint16_t port, rclcpp::Node::SharedPtr node);
  /**
  * @brief The main execution loop for the sending interface
  */
  void run();
  /**
   * @brief Destroys the SendingInterface object
   */
  virtual ~SendingInterface();

  /**
  * @brief Enumeration for the status of a data sending operation
  */
  enum class SendingStatus {SUCCESS, NO_CONNECTIONS, NOT_COMPLETED, RESET_EXCEPTION, IO_EXCEPTION};

  /**
   * @brief Sends the given data blob to all currently connected clients
   * @param data A pointer to the raw data buffer to be sent
   * @param size The size of the data buffer in bytes
   * @return A SendingStatus indicating the outcome of the send operation
   */
  SendingStatus sendData(void * data, size_t size);

  /**
   * @brief Stops the sending interface's operation
   */
  void stop();

private:
  std::mutex connections_mutex_;
  Poco::Net::ServerSocket socket_;
  std::atomic<bool> running_;
  std::vector<Poco::Net::StreamSocket> connections_;

  rclcpp::Node::SharedPtr node_;
};

#endif  // BOSCH_NAVIGATOR_BRIDGE__SENDING_INTERFACE_HPP_
