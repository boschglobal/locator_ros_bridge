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

#include "locator_rpc_interface.hpp"

#include "enums.hpp"

#include <Poco/JSON/Parser.h>
#include <Poco/Net/HTTPRequest.h>
#include <Poco/Net/HTTPResponse.h>

#include <Poco/StreamCopier.h>

Poco::JSON::Object makeTimeInterval(bool valid, int64_t time, int64_t resolution)
{
  Poco::JSON::Object obj;
  obj.set("valid", valid);
  obj.set("time", time);
  obj.set("resolution", resolution);
  return obj;
}

Poco::JSON::Object makeSessionQueryMessage(const std::string& session_id)
{
  Poco::JSON::Object obj;
  obj.set("sessionId", session_id);
  return obj;
}

Poco::JSON::Object makeSessionLoginMessage(const std::string& username, const std::string& password,
                                           const Poco::JSON::Object& timeout)
{
  Poco::JSON::Object obj;
  obj.set("userName", username);
  obj.set("password", password);
  obj.set("timeout", timeout);
  return obj;
}

Poco::JSON::Object makeConfigEntry(const std::string& key, const Poco::Dynamic::Var& value)
{
  Poco::JSON::Object obj;
  obj.set("key", key);
  obj.set("value", value);
  return obj;
}

Poco::JSON::Object makeConfigEntryArrayMessage(const std::string& session_id, const Poco::DynamicStruct& config)
{
  Poco::JSON::Object obj = makeSessionQueryMessage(session_id);
  Poco::JSON::Array config_entries;
  for (const auto& e : config)
  {
    config_entries.add(makeConfigEntry(e.first, e.second));
  }
  obj.set("configEntries", config_entries);
  return obj;
}

LocatorRPCInterface::LocatorRPCInterface(const std::string& host, uint16_t port) : session_(host, port), query_id_(0)
{
}
LocatorRPCInterface::~LocatorRPCInterface()
{
  logout();
}

void LocatorRPCInterface::login(const std::string& user, const std::string& password)
{
  Poco::JSON::Object login_msg = makeSessionLoginMessage(user, password, makeTimeInterval(true, 600, 1));
  const auto resp = json_rpc_call(session_, "sessionLogin", login_msg);
  session_id_ = resp.getValue<std::string>("sessionId");
}

Poco::JSON::Object LocatorRPCInterface::getSessionQuery() const
{
  return makeSessionQueryMessage(session_id_);
}

void LocatorRPCInterface::refresh()
{
  const auto logout_resp = json_rpc_call(session_, "sessionRefresh", makeSessionQueryMessage(session_id_));
}

void LocatorRPCInterface::logout()
{
  const auto logout_resp = json_rpc_call(session_, "sessionLogout", makeSessionQueryMessage(session_id_));
}

std::string LocatorRPCInterface::getAboutBuildList()
{
  const auto about_build_resp = json_rpc_call(session_, "aboutBuildList", makeSessionQueryMessage(session_id_));
  return about_build_resp.getValue<std::string>("aboutString");
}

std::unordered_map<std::string, std::pair<int32_t, int32_t>> LocatorRPCInterface::getAboutModules()
{
  const auto about_modules_resp = json_rpc_call(session_, "aboutModulesList", Poco::JSON::Object());
  about_modules_resp.stringify(std::cout);

  std::unordered_map<std::string, std::pair<int32_t, int32_t>> result;
  if (about_modules_resp.has("modules"))
  {
    const auto modules = about_modules_resp.getArray("modules");
    for (size_t i = 0; i < modules->size(); i++)
    {
      const auto obj = modules->getObject(i);
      result.insert({ obj->getValue<std::string>("name"),
                      std::make_pair(obj->getValue<int32_t>("majorVersion"), obj->getValue<int32_t>("minorVersion")) });
    }
  }

  return result;
}

Poco::DynamicStruct LocatorRPCInterface::getConfigList()
{
  auto config_list_resp = json_rpc_call(session_, "configList", makeSessionQueryMessage(session_id_));
  Poco::DynamicStruct config;
  if (config_list_resp.has("configEntries"))
  {
    const auto entries = config_list_resp.getArray("configEntries");
    for (size_t i = 0; i < entries->size(); i++)
    {
      const auto obj = entries->getObject(i);
      config.insert(obj->getValue<std::string>("key"), obj->get("value"));
    }
  }
  return config;
}

void LocatorRPCInterface::setConfigList(const Poco::DynamicStruct& config)
{
  auto config_resp = json_rpc_call(session_, "configSet", makeConfigEntryArrayMessage(session_id_, config));
}

Poco::JSON::Object LocatorRPCInterface::call(const std::string& method, const Poco::JSON::Object& query_obj)
{
  Poco::JSON::Object resp;

  try
  {
    resp = json_rpc_call(session_, method, query_obj);
  }
  catch (const std::runtime_error& error)
  {
    std::cerr << error.what() << std::endl;
  }

  return resp;
}

Poco::JSON::Object LocatorRPCInterface::json_rpc_call(Poco::Net::HTTPClientSession& session, const std::string& method,
                                                      const Poco::JSON::Object& query_obj)
{
  std::lock_guard<std::mutex> lock(json_rpc_call_mutex_);  // just one call at a time

  using Poco::Net::HTTPRequest;
  using Poco::Net::HTTPResponse;

  Poco::JSON::Object obj;
  obj.set("jsonrpc", "2.0");
  obj.set("method", method);
  Poco::JSON::Object params_obj;
  params_obj.set("query", query_obj);
  obj.set("params", params_obj);
  const size_t request_id = query_id_++;
  obj.set("id", request_id);
  std::stringstream sstr;
  obj.stringify(sstr);
  const std::string json_str = sstr.str();

  HTTPRequest req(HTTPRequest::HTTP_POST, "/", "HTTP/1.1");
  req.setContentType("application/json");
  req.setContentLength(json_str.size());
  req.setKeepAlive(true);
  auto& outstream = session.sendRequest(req);
  outstream << json_str;

  HTTPResponse resp;
  std::istream& instream = session.receiveResponse(resp);
  std::string response_string;
  Poco::StreamCopier::copyToString(instream, response_string);
  if (resp.getStatus() != 200)
  {
    throw std::runtime_error(resp.getReason());
  }

  Poco::JSON::Parser json_parser;
  const auto& result = json_parser.parse(response_string);
  Poco::JSON::Object::Ptr response_obj = result.extract<Poco::JSON::Object::Ptr>();
  if (response_obj->getValue<size_t>("id") != request_id)
  {
    std::cout << "error: ids do not match\n";
  }
  if (!response_obj->has("result"))
  {
    const auto error_obj = response_obj->getObject("error");
    const auto error_code = error_obj->getValue<int>("code");
    const auto error_msg = error_obj->getValue<std::string>("message");
    std::stringstream sstr;
    sstr << "ERROR: JSON-RPC error occured! code: " << error_code << ", message: " << error_msg;
    throw std::runtime_error(sstr.str());
  }
  const auto response_code =
      response_obj->getObject("result")->getObject("response")->getValue<uint64_t>("responseCode");
  if (response_code != 0)
  {
    const auto error_str = stringifyCommonResponseCode(static_cast<CommonResponseCode>(response_code));
    const auto module_str = stringifyModuleId(static_cast<ModuleIdentifier>(response_code >> 48));
    std::stringstream sstr;
    sstr << "ERROR: response code != 0; response code 0x" << std::hex << response_code << "; module: " << module_str;
    if (!error_str.empty())
    {
      sstr << "; error: " << error_str;
    }
    throw std::runtime_error(sstr.str());
  }

  return *response_obj->getObject("result")->getObject("response");
}
