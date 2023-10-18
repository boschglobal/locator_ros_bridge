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

#ifndef BOSCH_LOCATOR_BRIDGE__RECEIVING_INTERFACE_HPP_
#define BOSCH_LOCATOR_BRIDGE__RECEIVING_INTERFACE_HPP_

#include <Poco/Net/SocketReactor.h>
#include <Poco/Net/StreamSocket.h>
#include <Poco/Net/SocketNotification.h>
#include <Poco/Net/NetException.h>

#include <string>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


/**
 * @brief The ReceivingInterface class is the base class for all receiving interfaces, such as
 * ClientControlModeInterface, etc.
 */
class ReceivingInterface : public Poco::Runnable
{
public:
  ReceivingInterface(
    const Poco::Net::IPAddress & hostadress,
    Poco::UInt16 port,
    ros::NodeHandle & nh);

  virtual ~ReceivingInterface();

  virtual void onReadEvent(const Poco::AutoPtr<Poco::Net::ReadableNotification> & notification);

  void run();

protected:
  /**
   * @brief Actual function to be overwritten by child to handle data,
   * e.g., convert to ros messages and publish
   * @param datagram_buffer The data received via the binary connection socket
   * @return amount of bytes successfully parsed and can be removed from the buffer
   * (0 if not parsing failed)
   */
  virtual size_t tryToParseData(const std::vector<char> & datagram_buffer) = 0;

  //! Publisher
  std::vector<ros::Publisher> publishers_;

  //! Node handle
  ros::NodeHandle nh_;

private:
  Poco::Net::StreamSocket ccm_socket_;
  Poco::Net::SocketReactor reactor_;
  // TODO(): use a better suited data structure (a deque?)
  std::vector<char> datagram_buffer_;
};

class ClientControlModeInterface : public ReceivingInterface
{
public:
  ClientControlModeInterface(
    const Poco::Net::IPAddress & hostadress,
    const Poco::UInt16 binaryClientControlModePort,
    ros::NodeHandle & nh);
  size_t tryToParseData(const std::vector<char> & datagram) override;
};

class ClientMapMapInterface : public ReceivingInterface
{
public:
  ClientMapMapInterface(
    const Poco::Net::IPAddress & hostadress,
    const Poco::UInt16 binaryClientMapMapPort,
    ros::NodeHandle & nh);
  size_t tryToParseData(const std::vector<char> & datagram) override;
};

class ClientMapVisualizationInterface : public ReceivingInterface
{
public:
  ClientMapVisualizationInterface(
    const Poco::Net::IPAddress & hostadress,
    const Poco::UInt16 binaryClientMapVisualizationPort,
    ros::NodeHandle & nh);
  size_t tryToParseData(const std::vector<char> & datagram) override;
};

class ClientRecordingMapInterface : public ReceivingInterface
{
public:
  ClientRecordingMapInterface(
    const Poco::Net::IPAddress & hostadress,
    const Poco::UInt16 binaryClientRecordingMapPort,
    ros::NodeHandle & nh);
  size_t tryToParseData(const std::vector<char> & datagram) override;
};

class ClientRecordingVisualizationInterface : public ReceivingInterface
{
public:
  ClientRecordingVisualizationInterface(
    const Poco::Net::IPAddress & hostadress,
    const Poco::UInt16 binaryClientRecordingVisualizationPort,
    ros::NodeHandle & nh);
  size_t tryToParseData(const std::vector<char> & datagram) override;
};

class ClientLocalizationMapInterface : public ReceivingInterface
{
public:
  ClientLocalizationMapInterface(
    const Poco::Net::IPAddress & hostadress,
    const Poco::UInt16 binaryClientLocalizationMapPort,
    ros::NodeHandle & nh);
  size_t tryToParseData(const std::vector<char> & datagram) override;
};

class ClientLocalizationVisualizationInterface : public ReceivingInterface
{
public:
  ClientLocalizationVisualizationInterface(
    const Poco::Net::IPAddress & hostadress,
    const Poco::UInt16 binaryClientLocalizationVisualizationPort,
    ros::NodeHandle & nh);
  size_t tryToParseData(const std::vector<char> & datagram) override;
};

class ClientLocalizationPoseInterface : public ReceivingInterface
{
public:
  ClientLocalizationPoseInterface(
    const Poco::Net::IPAddress & hostadress,
    const Poco::UInt16 binaryClientLocalizationPosePort,
    ros::NodeHandle & nh);
  size_t tryToParseData(const std::vector<char> & datagram) override;
};

class ClientGlobalAlignVisualizationInterface : public ReceivingInterface
{
public:
  ClientGlobalAlignVisualizationInterface(
    const Poco::Net::IPAddress & hostadress,
    const Poco::UInt16 binaryClientGlobalAlignVisualizationPort,
    ros::NodeHandle & nh);
  size_t tryToParseData(const std::vector<char> & datagram) override;
};

class ClientExpandMapVisualizationInterface : public ReceivingInterface
{
public:
  ClientExpandMapVisualizationInterface(
    const Poco::Net::IPAddress & hostadress,
    const Poco::UInt16 binaryClientExpandMapVisualizationPort,
    ros::NodeHandle & nh);
  size_t tryToParseData(const std::vector<char> & datagram) override;
};

class ClientExpandMapPriorMapInterface : public ReceivingInterface
{
public:
  ClientExpandMapPriorMapInterface(
    const Poco::Net::IPAddress & hostadress,
    const Poco::UInt16 binaryClientExpandMapPriorMapPort,
    ros::NodeHandle & nh);
  size_t tryToParseData(const std::vector<char> & datagram) override;
};

#endif  // BOSCH_LOCATOR_BRIDGE__RECEIVING_INTERFACE_HPP_
