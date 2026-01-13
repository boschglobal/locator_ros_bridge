#ifndef BOSCH_NAVIGATOR_BRIDGE__NAVIGATOR_RPC_INTERFACE_HPP_
#define BOSCH_NAVIGATOR_BRIDGE__NAVIGATOR_RPC_INTERFACE_HPP_

#include <Poco/JSON/Object.h>
#include <Poco/Net/HTTPClientSession.h>

#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>


class NavigatorInterface
{
public:
  /**
   * @brief Constructs a new NavigatorInterface object
   * @param host The hostname or IP address of the Navigator
   * @param port The port number for the Navigator's RPC service
   */
  NavigatorInterface(const std::string & host, uint16_t port);
  /**
   * @brief Destroys the NavigatorInterface object
   */
  virtual ~NavigatorInterface();

   /**
   * @brief Logs into the Navigator API
   * @param user The username for login
   * @param password The password for login
   */
  void login(const std::string & user, const std::string & password);

  /**
   * @brief Refreshes the current session with the Navigator API
   */
  void refresh();

  /**
   * @brief Logs out from the Navigator API
   */
  void logout();

  /**
   * @brief Retrieves a string describing the Navigator's build version
   * @return A string containing build information
   */
  std::string getAboutBuildList();

  /**
   * @brief Retrieves version information for all Navigator modules
   * @return A map where keys are module names and values are pairs of major/minor version numbers
   */
  std::unordered_map<std::string, std::pair<int32_t, int32_t>> getAboutModules();

  /**
   * @brief Retrieves the current configuration of the Navigator
   * @return A Poco::DynamicStruct containing the configuration entries
   */
  Poco::DynamicStruct getConfigList();
  /**
   * @brief Sets the Navigator's configuration
   * @param config A Poco::DynamicStruct containing the configuration entries to set
   * @return True if the configuration was set successfully, false otherwise
   */
  bool setConfigList(const Poco::DynamicStruct & config);

  /**
   * @brief Retrieves a JSON object representing the current session query
   *
   * @return A Poco::JSON::Object with session query details
   */
  Poco::JSON::Object getSessionQuery() const;
  
  /**
   * @brief Calls a generic RPC method on the Navigator
   *
   * @param method The name of the RPC method to call
   * @param query_obj A Poco::JSON::Object containing the parameters for the method call
   * @return A Poco::JSON::Object containing the response from the RPC call
   */
  Poco::JSON::Object call(const std::string & method, const Poco::JSON::Object & query_obj);

protected:
 /**
   * @brief Performs a generic JSON-RPC call using an HTTP client session
   * @param session The HTTPClientSession to use for the call
   * @param method The name of the RPC method to call
   * @param query_obj A Poco::JSON::Object containing the parameters for the method call
   * @return A Poco::JSON::Object containing the response from the RPC call
   */
  Poco::JSON::Object json_rpc_call(
    Poco::Net::HTTPClientSession & session, const std::string & method,
    const Poco::JSON::Object & query_obj);
  std::mutex json_rpc_call_mutex_;
  Poco::Net::HTTPClientSession session_;
  std::string session_id_;
  size_t query_id_;
};

#endif  // BOSCH_Navigator_BRIDGE__Navigator_RPC_INTERFACE_HPP_
