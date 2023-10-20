// Copyright (c) 2021 - for information on the respective copyright owner
// see the NOTICE file and/or the repository
// https://github.com/boschglobal/locator_ros_bridge.
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

#ifndef BOSCH_LOCATOR_BRIDGE__SERVER__SERVER_BRIDGE_NODE_HPP_
#define BOSCH_LOCATOR_BRIDGE__SERVER__SERVER_BRIDGE_NODE_HPP_

#include <Poco/Thread.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ros/ros.h"
#include "std_srvs/Empty.h"

#include "bosch_locator_bridge/ServerMapGetImageWithResolution.h"
#include "bosch_locator_bridge/ServerMapList.h"

// forward declarations
class LocatorRPCInterface;

/**
 * This is the main ROS node. It binds together the ROS interface and the
 * Locator API.
 */
class ServerBridgeNode
{
public:
  ServerBridgeNode();
  virtual ~ServerBridgeNode();

  void init();

private:
  bool check_module_versions(
    const std::unordered_map<std::string, std::pair<int32_t, int32_t>>
    & module_versions);
  template<typename T>
  std::vector<T> convert_value_array_to_vector(const XmlRpc::XmlRpcValue & array) const;

  bool serverMapListCb(
    bosch_locator_bridge::ServerMapList::Request & req,
    bosch_locator_bridge::ServerMapList::Response & res);

  bool serverMapGetImageWithResolutionCb(
    bosch_locator_bridge::ServerMapGetImageWithResolution::Request & req,
    bosch_locator_bridge::ServerMapGetImageWithResolution::Response & res);

  /// read out ROS parameters and use them to update the locator config
  void syncConfig();

  ros::NodeHandle nh_;
  std::unique_ptr<LocatorRPCInterface> server_interface_;

  ros::Timer session_refresh_timer_;

  std::vector<ros::ServiceServer> services_;
};

template<typename T>
std::vector<T> ServerBridgeNode::convert_value_array_to_vector(
  const XmlRpc::XmlRpcValue & array) const
{
  std::vector<T> vec;
  for (int i = 0; i != array.size(); ++i) {
    vec.push_back(static_cast<T>(array[i]));
  }

  return vec;
}

#endif  // BOSCH_LOCATOR_BRIDGE__SERVER__SERVER_BRIDGE_NODE_HPP_
