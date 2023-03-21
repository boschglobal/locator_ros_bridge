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

#pragma once

#include <mutex>
#include <unordered_map>
#include <Poco/Net/HTTPClientSession.h>

#include <Poco/JSON/Object.h>

/**
 * Shared RPC interface for JSON RPC communication with localization client and map server.
 * See API documentation, chapter 8.
 */
class LocatorRPCInterface
{
public:
  LocatorRPCInterface(const std::string& host, uint16_t port);
  virtual ~LocatorRPCInterface();

  void login(const std::string& user, const std::string& password);
  void refresh();
  void logout();

  std::string getAboutBuildList();
  std::unordered_map<std::string, std::pair<int32_t, int32_t>> getAboutModules();

  Poco::DynamicStruct getConfigList();
  bool setConfigList(const Poco::DynamicStruct& config);

  Poco::JSON::Object getSessionQuery() const;
  Poco::JSON::Object call(const std::string& method, const Poco::JSON::Object& query_obj);

protected:
  Poco::JSON::Object json_rpc_call(Poco::Net::HTTPClientSession& session, const std::string& method,
                                   const Poco::JSON::Object& query_obj);
  std::mutex json_rpc_call_mutex_;
  Poco::Net::HTTPClientSession session_;
  std::string session_id_;
  size_t query_id_;
};
