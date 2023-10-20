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

#include "server/server_bridge_node.hpp"

#include <fstream>
#include <string>
#include <unordered_map>
#include <utility>

#include "Poco/Base64Decoder.h"

#include "enums.hpp"
#include "locator_rpc_interface.hpp"

/// server module versions to check against. Format is name, { major_version,
/// minor_version }
static const std::unordered_map<std::string, std::pair<int32_t, int32_t>>
REQUIRED_MODULE_VERSIONS({
  {"AboutModules", {5, 0}},
  {"Session", {3, 1}},
  {"Diagnostic", {4, 1}},
  {"Licensing", {6, 1}},
  {"Config", {5, 1}},
  {"AboutBuild", {3, 0}},
  {"Certificate", {3, 0}},
  {"System", {3, 1}},
  {"ServerMap", {6, 0}},
  {"ServerUser", {4, 0}},
  {"ServerInternal", {2, 0}},
});

ServerBridgeNode::ServerBridgeNode()
: nh_("~") {}

ServerBridgeNode::~ServerBridgeNode() {}

void ServerBridgeNode::init()
{
  std::string host;
  nh_.getParam("server_host", host);

  std::string user, pwd;
  nh_.getParam("user_name", user);
  nh_.getParam("password", pwd);

  server_interface_.reset(new LocatorRPCInterface(host, 8082));
  server_interface_->login(user, pwd);
  session_refresh_timer_ =
    nh_.createTimer(
    ros::Duration(30.), [&](const ros::TimerEvent &) {
      ROS_INFO_STREAM("refreshing session!");
      server_interface_->refresh();
    });

  const auto module_versions = server_interface_->getAboutModules();
  if (!check_module_versions(module_versions)) {
    throw std::runtime_error("locator software incompatible with this bridge!");
  }

  syncConfig();
  services_.push_back(
    nh_.advertiseService(
      "list_server_maps", &ServerBridgeNode::serverMapListCb, this));
  services_.push_back(
    nh_.advertiseService(
      "get_map_with_resolution",
      &ServerBridgeNode::serverMapGetImageWithResolutionCb, this));

  ROS_INFO_STREAM("initialization done");
}

void ServerBridgeNode::syncConfig()
{
  ROS_INFO_STREAM("syncing config");
  XmlRpc::XmlRpcValue localization_server_rosconfig;
  nh_.getParam("localization_server_config", localization_server_rosconfig);

  auto server_config = server_interface_->getConfigList();
  // overwrite current locator config with ros params
  for (auto & iter : localization_server_rosconfig) {
    const auto & key = iter.first;
    auto & value = iter.second;
    if (!localization_server_rosconfig.hasMember(key)) {
      ROS_WARN_STREAM(
        "invalid locator rosparam found: " << key << ", " <<
          value);
      continue;
    }
    ROS_INFO_STREAM("setting value for " << key << ", " << value);
    switch (value.getType()) {
      case XmlRpc::XmlRpcValue::TypeBoolean:
        server_config[key] = static_cast<bool>(value);
        break;
      case XmlRpc::XmlRpcValue::TypeInt:
        server_config[key] = static_cast<int>(value);
        break;
      case XmlRpc::XmlRpcValue::TypeDouble:
        server_config[key] = static_cast<double>(value);
        break;
      case XmlRpc::XmlRpcValue::TypeString:
        server_config[key] = static_cast<std::string>(value);
        break;
      case XmlRpc::XmlRpcValue::TypeArray:
        if (value.size() == 0) {
          // for an empty array the type does not matter here, we arbitrarily choose double
          server_config[key] = convert_value_array_to_vector<double>(value);
        } else {
          switch (value[0].getType()) {
            case XmlRpc::XmlRpcValue::TypeBoolean:
              {
                server_config[key] = convert_value_array_to_vector<bool>(value);
                break;
              }
            case XmlRpc::XmlRpcValue::TypeInt:
              {
                server_config[key] = convert_value_array_to_vector<int>(value);
                break;
              }
            case XmlRpc::XmlRpcValue::TypeDouble:
              {
                server_config[key] = convert_value_array_to_vector<double>(value);
                break;
              }
            case XmlRpc::XmlRpcValue::TypeString:
              {
                server_config[key] = convert_value_array_to_vector<std::string>(value);
                break;
              }
            default:
              ROS_ERROR_STREAM("unknown element type for " << key);
              break;
          }
        }
        break;
      default:
        ROS_ERROR_STREAM("unknown config type for " << key);
        break;
    }
  }
  ROS_INFO_STREAM("new loc client config: " << server_config.toString());
  for (const auto & c : server_config) {
    ROS_INFO_STREAM("- " << c.first << ": " << c.second.toString());
  }

  server_interface_->setConfigList(server_config);
}

bool ServerBridgeNode::check_module_versions(
  const std::unordered_map<std::string, std::pair<int32_t, int32_t>>
  & module_versions)
{
  ROS_INFO("-----------------check_module_versions");
  for (const auto & required_pair : REQUIRED_MODULE_VERSIONS) {
    const auto & module_name = required_pair.first;
    const auto & required_version = required_pair.second;

    const auto & actual_version_iter = module_versions.find(module_name);
    if (actual_version_iter == module_versions.end()) {
      ROS_WARN_STREAM(
        "required server module " << module_name <<
          " not found!");
      return false;
    }
    const auto & actual_version = actual_version_iter->second;
    // major version number needs to match, minor version number equal or bigger
    if ((actual_version.first == required_version.first) &&
      (actual_version.second >= required_version.second))
    {
      ROS_DEBUG_STREAM("server module " << module_name << ": version ok!");
    } else {
      ROS_WARN_STREAM(
        "---------8 module: " <<
          module_name <<
          " required version: " << required_version.first << "." <<
          required_version.second <<
          " (actual version: " << actual_version.first << "." <<
          actual_version.second << ")");
      return false;
    }
  }
  return true;
}

bool ServerBridgeNode::serverMapListCb(
  bosch_locator_bridge::ServerMapList::Request & req,
  bosch_locator_bridge::ServerMapList::Response & res)
{
  auto query = server_interface_->getSessionQuery();
  auto response = server_interface_->call("serverMapList", query);
  if (response.has("serverMapNames")) {
    const auto entries = response.getArray("serverMapNames");
    for (size_t i = 0; i < entries->size(); i++) {
      res.names.push_back(entries->get(i).toString());
    }
  }
  return true;
}

bool ServerBridgeNode::serverMapGetImageWithResolutionCb(
  bosch_locator_bridge::ServerMapGetImageWithResolution::Request & req,
  bosch_locator_bridge::ServerMapGetImageWithResolution::Response & res)
{
  auto query = server_interface_->getSessionQuery();
  query.set("resolution", req.resolution);
  query.set("serverMapName", req.map_name);

  auto response =
    server_interface_->call("serverMapGetImageWithResolution", query);

  if (response.getValue<double>("responseCode") == CommonResponseCode::OK) {
    const auto & pose_2d = response.getObject("MAPimageOrigin");
    const double & x = pose_2d->getValue<double>("x");
    const double & y = pose_2d->getValue<double>("y");
    const double & a = pose_2d->getValue<double>("a");
    const double & width = response.getValue<double>("width");
    const double & height = response.getValue<double>("height");

    const std::string & encoded_png =
      response.getObject("image")->getValue<std::string>("content");

    std::istringstream png_stream(encoded_png);
    Poco::Base64Decoder decoder(png_stream);

    ROS_INFO_STREAM(
      "Received map with origin x " << x << "y " << y << "a " << a <<
        "\nheight " << height <<
        "width " << width);
    std::ofstream file;
    file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
    try {
      file.open(req.file_name + ".png", std::ios::binary);
      file << decoder.rdbuf();
      file.close();

      file.open(req.file_name + ".yaml");
      file << "image: " << req.file_name + ".png" <<
        "\nresolution: " << 1.0 / req.resolution << "\norigin: [" <<
        std::fixed << std::setprecision(6) << x << ", " << y << ", " << a <<
        "]\nnegate: "
        "0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n";
    } catch (std::ofstream::failure & ex) {
      ROS_ERROR_STREAM(
        "Exception occured while writing: " <<
          ex.what() << " with code: " << ex.code());
      return false;
    }

  } else {
    return false;
  }
  return true;
}
