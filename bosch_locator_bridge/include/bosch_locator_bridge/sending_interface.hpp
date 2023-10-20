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

#ifndef BOSCH_LOCATOR_BRIDGE__SENDING_INTERFACE_HPP_
#define BOSCH_LOCATOR_BRIDGE__SENDING_INTERFACE_HPP_

#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/StreamSocket.h>

#include <atomic>
#include <mutex>
#include <vector>

/**
 * for communicating with "push" consumer, e.g. ClientSensorLaser
 */
class SendingInterface : public Poco::Runnable
{
public:
  explicit SendingInterface(uint16_t port);
  void run();
  virtual ~SendingInterface();

  enum class SendingStatus {SUCCESS, NO_CONNECTIONS, NOT_COMPLETED, RESET_EXCEPTION, IO_EXCEPTION};

  /**
   * Send the given data blob to all connected clients.
   */
  SendingStatus sendData(void * data, size_t size);

  void stop();

private:
  std::mutex connections_mutex_;
  Poco::Net::ServerSocket socket_;
  std::atomic<bool> running_;
  std::vector<Poco::Net::StreamSocket> connections_;
};

#endif  // BOSCH_LOCATOR_BRIDGE__SENDING_INTERFACE_HPP_
