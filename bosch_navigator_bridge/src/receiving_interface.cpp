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

#include "bosch_navigator_bridge/receiving_interface.hpp"

#include <Poco/NObserver.h>

#include <string>
#include <vector>

#include "bosch_navigator_bridge/rosmsgs_datagram_converter.hpp"


ReceivingInterface::ReceivingInterface(
  const Poco::Net::IPAddress & hostadress, Poco::UInt16 port,
  rclcpp::Node::SharedPtr node)
: node_(node),
  ccm_socket_(Poco::Net::SocketAddress(hostadress, port))
{
  reactor_.addEventHandler(
    ccm_socket_, Poco::NObserver<ReceivingInterface, Poco::Net::ReadableNotification>(
      *this, &ReceivingInterface::onReadEvent));
}

ReceivingInterface::~ReceivingInterface()
{
  reactor_.stop();
  ccm_socket_.shutdown();
}

void ReceivingInterface::onReadEvent(
  const Poco::AutoPtr<Poco::Net::ReadableNotification> & /*notification*/)
{
  try {
    // Create buffer with size of available data
    const int bytes_available = ccm_socket_.available();
    std::vector<char> msg(bytes_available);
    int received_bytes = ccm_socket_.receiveBytes(&(msg[0]), bytes_available);
    if (received_bytes == 0) {
      std::cout << "received msg of length 0... Connection closed? \n";
    } else {
      datagram_buffer_.insert(datagram_buffer_.end(), msg.begin(), msg.end());

      size_t bytes_to_delete = 0;
      // Try to parse messages from the buffer until tryToParseData fails to parse a full message
      do {
        bytes_to_delete = tryToParseData(datagram_buffer_, node_);
        datagram_buffer_.erase(
          datagram_buffer_.begin(),
          datagram_buffer_.begin() + bytes_to_delete);
      } while (bytes_to_delete > 0);
    }
  } catch (const std::ios_base::failure & io_failure) {
    // catching this exception is actually no error:
    // the datagram is just not yet completely transmitted could not be
    // parsed because of that. Will automatically retry after more data is available.
  } catch (...) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Caught exception in ReceivingInterface!");
  }
}

void ReceivingInterface::run()
{
  reactor_.run();
}

ClientMotionCommandInterface::ClientMotionCommandInterface(
  const Poco::Net::IPAddress & hostadress,
  const Poco::UInt16 binaryClientControlModePort,
  rclcpp::Node::SharedPtr node)
: ReceivingInterface(hostadress, binaryClientControlModePort, node)
{
  // Setup publisher
  client_motion_command_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", 5);
}

size_t ClientMotionCommandInterface::tryToParseData(
  const std::vector<char> & datagram,
  rclcpp::Node::SharedPtr)
{
  // convert datagram to ros messages
  geometry_msgs::msg::Twist twist;

  const auto bytes_parsed = RosMsgsDatagramConverter::convertMotionCommand2Twist(
    datagram, twist);

  if (bytes_parsed > 0) {
    // publish
    client_motion_command_pub_->publish(twist);
  }
  return bytes_parsed;
}
