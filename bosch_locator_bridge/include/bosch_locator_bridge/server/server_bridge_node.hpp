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

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "bosch_locator_bridge/srv/server_map_get_image_with_resolution.hpp"
#include "bosch_locator_bridge/srv/server_map_list.hpp"

// forward declarations
class LocatorRPCInterface;

/**
 * This is the server bridge node. It binds together the ROS interface and the
 * Locator API of the map server.
 */
class ServerBridgeNode : public rclcpp::Node
{
public:
  explicit ServerBridgeNode(const std::string & nodeName);
  virtual ~ServerBridgeNode();

  void init();

private:
  bool check_module_versions(
    const std::unordered_map<std::string, std::pair<int32_t, int32_t>>
    & module_versions);

  bool serverMapListCb(
    const std::shared_ptr<bosch_locator_bridge::srv::ServerMapList::Request> req,
    std::shared_ptr<bosch_locator_bridge::srv::ServerMapList::Response> res);

  bool serverMapGetImageWithResolutionCb(
    const std::shared_ptr<bosch_locator_bridge::srv::ServerMapGetImageWithResolution::Request> req,
    std::shared_ptr<bosch_locator_bridge::srv::ServerMapGetImageWithResolution::Response> res);

  /// read out ROS parameters and use them to update the locator config
  void syncConfig();

  std::unique_ptr<LocatorRPCInterface> server_interface_;

  rclcpp::TimerBase::SharedPtr session_refresh_timer_;

  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
};

#endif  // BOSCH_LOCATOR_BRIDGE__SERVER__SERVER_BRIDGE_NODE_HPP_
