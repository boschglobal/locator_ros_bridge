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

#include "sending_interface.hpp"

#include <Poco/Net/NetException.h>

#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

SendingInterface::SendingInterface(uint16_t port, rclcpp::Node::SharedPtr node)
: socket_(port),
  running_(true),
  node_(node)
{
  // configure server socket same as binary interface example
  socket_.setKeepAlive(true);
  socket_.setReceiveTimeout(Poco::Timespan(30, 0));
}

void SendingInterface::run()
{
  Poco::Timespan timeout(2000000);
  while (running_) {
    if (socket_.poll(timeout, Poco::Net::Socket::SELECT_READ)) {
      try {
        Poco::Net::SocketAddress clientAddr;
        Poco::Net::StreamSocket sock = socket_.acceptConnection(clientAddr);
        RCLCPP_INFO_STREAM(
          node_->get_logger(),
          "accepted connection from " << clientAddr << " at " << sock.address().toString());
        sock.setNoDelay(true);
        {
          std::lock_guard<std::mutex> lock(connections_mutex_);
          connections_.push_back(sock);
        }
      } catch (const Poco::Exception & e) {
        RCLCPP_ERROR_STREAM(
          node_->get_logger(),
          "caught exception in SendingInterface: " << e.what());
      }
    }
  }
}

SendingInterface::~SendingInterface()
{
  stop();
}

void SendingInterface::sendData(void * data, size_t size)
{
  std::lock_guard<std::mutex> lock(connections_mutex_);
  std::vector<Poco::Net::StreamSocket> good_connections;
  if (connections_.size() == 0) {
    RCLCPP_INFO_STREAM_THROTTLE(
      node_->get_logger().get_child(std::to_string(size)), *node_->get_clock(), 10000,
      "Cannot send data of size " << size << " to any peer (no connections available)");
  }
  for (size_t i = 0; i < connections_.size(); i++) {
    try {
      size_t total_sent = 0;
      while (total_sent < size) {
        const auto sent = connections_[i].sendBytes(data, size);
        if (sent <= 0) {
          break;
        }
        total_sent += sent;
      }
      if (total_sent == size) {
        good_connections.push_back(connections_[i]);
        RCLCPP_INFO_STREAM_THROTTLE(
          node_->get_logger().get_child(std::to_string(size)), *node_->get_clock(), 10000,
          size << " bytes successfully sent via " << connections_[i].address());
      } else {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "could not sent datagram completely!");
      }
    } catch (const Poco::Net::ConnectionResetException & e) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "caught connection reset exception: " << e.name());
    } catch (const Poco::IOException & e) {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "caught io exception: " << e.name() << "  -  " << e.what());
    }
  }
  const auto discarded_connections = connections_.size() - good_connections.size();
  if (discarded_connections > 0) {
    RCLCPP_WARN_STREAM(
      node_->get_logger(), "discarding " << discarded_connections << " connections!");
  }
  std::swap(connections_, good_connections);
}

void SendingInterface::stop()
{
  running_.store(false);
}
