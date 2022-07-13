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

#include <ros/ros.h>

#include <Poco/Net/NetException.h>

SendingInterface::SendingInterface(uint16_t port) : socket_(port), running_(true)
{
  // configure server socket same as binary interface example
  socket_.setKeepAlive(true);
  socket_.setReceiveTimeout(Poco::Timespan(30, 0));
}

void SendingInterface::run()
{
  Poco::Timespan timeout(2000000);
  while (running_)
  {
    if (socket_.poll(timeout, Poco::Net::Socket::SELECT_READ))
    {
      try
      {
        Poco::Net::SocketAddress clientAddr;
        Poco::Net::StreamSocket sock = socket_.acceptConnection(clientAddr);
        ROS_INFO_STREAM("accepted connection from " << clientAddr << " at " << sock.address().toString());
        sock.setNoDelay(true);
        {
          std::lock_guard<std::mutex> lock(connections_mutex_);
          connections_.push_back(sock);
        }
      }
      catch (const Poco::Exception& e)
      {
        ROS_ERROR_STREAM("caught exception in SendingInterface: " << e.what());
      }
    }
  }
}

SendingInterface::~SendingInterface()
{
  stop();
}

SendingInterface::SendingStatus SendingInterface::sendData(void* data, size_t size)
{
  SendingStatus ret = SendingStatus::SUCCESS;

  std::lock_guard<std::mutex> lock(connections_mutex_);
  std::vector<Poco::Net::StreamSocket> good_connections;
  if (connections_.size() == 0)
  {
    ROS_INFO_STREAM_THROTTLE_NAMED(10, std::to_string(size),
                                   "Cannot send data of size " << size << " to any peer (no connections available)");
    ret = SendingStatus::NO_CONNECTIONS;
  }
  for (size_t i = 0; i < connections_.size(); i++)
  {
    try
    {
      size_t total_sent = 0;
      while (total_sent < size)
      {
        const auto sent = connections_[i].sendBytes(data, size);
        if (sent <= 0)
        {
          break;
        }
        total_sent += sent;
      }
      if (total_sent == size)
      {
        good_connections.push_back(connections_[i]);
        ROS_INFO_STREAM_THROTTLE_NAMED(10, std::to_string(size),
                                       size << " bytes successfully sent via " << connections_[i].address());
      }
      else
      {
        ROS_ERROR_STREAM("could not sent datagram completely!");
        ret = SendingStatus::NOT_COMPLETED;
      }
    }
    catch (const Poco::Net::ConnectionResetException& e)
    {
      ROS_ERROR_STREAM("caught connection reset exception: " << e.name());
    }
    catch (const Poco::IOException& e)
    {
      ROS_ERROR_STREAM("caught io exception: " << e.displayText());
      ret = SendingStatus::IO_EXCEPTION;
    }
  }
  const auto discarded_connections = connections_.size() - good_connections.size();
  if (discarded_connections > 0)
  {
    ROS_WARN_STREAM("discarding " << discarded_connections << " connections!");
  }
  std::swap(connections_, good_connections);

  return ret;
}

void SendingInterface::stop()
{
  running_.store(false);
}
