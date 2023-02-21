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

#include <Poco/Base64Decoder.h>

#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "enums.hpp"
#include "locator_rpc_interface.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

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
//  {"User", {4, 0}},
  {"ServerInternal", {2, 0}},
});

ServerBridgeNode::ServerBridgeNode(const std::string & nodeName)
: Node(nodeName,
    rclcpp::NodeOptions().allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
{
}

ServerBridgeNode::~ServerBridgeNode()
{
}

void ServerBridgeNode::init()
{
  std::string host;
  get_parameter("server_host", host);

  std::string user, pwd;
  get_parameter("user_name", user);
  get_parameter("password", pwd);

  server_interface_.reset(new LocatorRPCInterface(host, 8082));
  server_interface_->login(user, pwd);
  session_refresh_timer_ = create_wall_timer(
    30s, [&]() {
      RCLCPP_INFO_STREAM(get_logger(), "refreshing session!");
      server_interface_->refresh();
    });

  const auto module_versions = server_interface_->getAboutModules();
  if (!check_module_versions(module_versions)) {
    throw std::runtime_error("locator software incompatible with this bridge!");
  }

  syncConfig();

  services_.push_back(
    create_service<bosch_locator_bridge::srv::ServerMapList>(
      "~/list_server_maps",
      std::bind(&ServerBridgeNode::serverMapListCb, this, _1, _2)));
  services_.push_back(
    create_service<bosch_locator_bridge::srv::ServerMapGetImageWithResolution>(
      "~/get_map_with_resolution",
      std::bind(&ServerBridgeNode::serverMapGetImageWithResolutionCb, this, _1, _2)));

  RCLCPP_INFO_STREAM(get_logger(), "initialization done");
}

void ServerBridgeNode::syncConfig()
{
  RCLCPP_INFO_STREAM(get_logger(), "syncing config");

  auto server_config = server_interface_->getConfigList();

  // overwrite current locator config with ros params

  std::map<std::string, rclcpp::Parameter> locator_parameters;
  get_node_parameters_interface()->get_parameters_by_prefix(
    "map_server_config",
    locator_parameters);
  std::for_each(
    locator_parameters.begin(), locator_parameters.end(), [&server_config,
    logger = get_logger()](const std::pair<std::string, rclcpp::Parameter> & param) {
      switch (param.second.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          server_config[param.first] = param.second.as_bool();
          break;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          server_config[param.first] = param.second.as_int();
          break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          server_config[param.first] = param.second.as_double();
          break;
        case rclcpp::ParameterType::PARAMETER_STRING:
          server_config[param.first] = param.second.as_string();
          break;
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
          server_config[param.first] = param.second.as_bool_array();
          break;
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
          server_config[param.first] = param.second.as_integer_array();
          break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
          server_config[param.first] = param.second.as_double_array();
          break;
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
          server_config[param.first] = param.second.as_string_array();
          break;
        default:
          RCLCPP_WARN(
            logger, "Parameter type %s is unsupported for Locator config!",
            param.second.get_type_name().c_str());
      }
    });

  RCLCPP_INFO_STREAM(get_logger(), "new server config: " << server_config.toString());
  for (const auto & c : server_config) {
    RCLCPP_INFO_STREAM(get_logger(), "- " << c.first << ": " << c.second.toString());
  }

  server_interface_->setConfigList(server_config);
}

bool ServerBridgeNode::check_module_versions(
  const std::unordered_map<std::string, std::pair<int32_t, int32_t>>
  & module_versions)
{
  RCLCPP_INFO(get_logger(), "-----------------check_module_versions");
  for (const auto & required_pair : REQUIRED_MODULE_VERSIONS) {
    const auto & module_name = required_pair.first;
    const auto & required_version = required_pair.second;

    const auto & actual_version_iter = module_versions.find(module_name);
    if (actual_version_iter == module_versions.end()) {
      RCLCPP_WARN_STREAM(get_logger(), "required server module " << module_name << " not found!");
      return false;
    }
    const auto & actual_version = actual_version_iter->second;
    // major version number needs to match, minor version number equal or bigger
    if ((actual_version.first == required_version.first) &&
      (actual_version.second >= required_version.second))
    {
      RCLCPP_DEBUG_STREAM(get_logger(), "server module " << module_name << ": version ok!");
    } else {
      RCLCPP_WARN_STREAM(
        get_logger(), "---------8 module: " <<
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
  const std::shared_ptr<bosch_locator_bridge::srv::ServerMapList::Request>/*req*/,
  std::shared_ptr<bosch_locator_bridge::srv::ServerMapList::Response> res)
{
  auto query = server_interface_->getSessionQuery();
  auto response = server_interface_->call("serverMapList", query);
  if (response.has("serverMapNames")) {
    const auto entries = response.getArray("serverMapNames");
    for (size_t i = 0; i < entries->size(); i++) {
      res->names.push_back(entries->get(i).toString());
    }
  }
  return true;
}

bool ServerBridgeNode::serverMapGetImageWithResolutionCb(
  const std::shared_ptr<bosch_locator_bridge::srv::ServerMapGetImageWithResolution::Request> req,
  std::shared_ptr<bosch_locator_bridge::srv::ServerMapGetImageWithResolution::Response>/*res*/)
{
  auto query = server_interface_->getSessionQuery();
  query.set("resolution", req->resolution);
  query.set("serverMapName", req->map_name);

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

    RCLCPP_INFO_STREAM(
      get_logger(), "Received map with origin x " << x << "y " << y << "a " << a <<
        "\nheight " << height <<
        "width " << width);
    std::ofstream file;
    file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
    try {
      file.open(req->file_name + ".png", std::ios::binary);
      file << decoder.rdbuf();
      file.close();

      file.open(req->file_name + ".yaml");
      file << "image: " << req->file_name + ".png" <<
        "\nresolution: " << 1.0 / req->resolution << "\norigin: [" <<
        std::fixed << std::setprecision(6) << x << ", " << y << ", " << a <<
        "]\nnegate: "
        "0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n";
    } catch (std::ofstream::failure & ex) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Exception occured while writing: " <<
          ex.what() << " with code: " << ex.code());
      return false;
    }

  } else {
    return false;
  }
  return true;
}
