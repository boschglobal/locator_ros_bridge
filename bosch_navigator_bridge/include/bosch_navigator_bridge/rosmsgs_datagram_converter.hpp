// Copyright (c) 2021 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/boschglobal/rokit_ros_bridge.
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

#ifndef BOSCH_NAVIGATOR_BRIDGE__ROSMSGS_DATAGRAM_CONVERTER_HPP_
#define BOSCH_NAVIGATOR_BRIDGE__ROSMSGS_DATAGRAM_CONVERTER_HPP_

#include <Poco/BinaryReader.h>
#include <Poco/JSON/Object.h>


#include <iostream>
#include <vector>
#include <algorithm>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"


/**
 * Class with static function to convert ros messages to Navigator's datagrams.
 */
class RosMsgsDatagramConverter
{
public:
  /** @brief convertOdometry2FeedbackDataGram Converts velocity and Odometry from ros and converts
   * it to the datagram structure required for the binary interface of the navigator
   *
   * @param msg The current odomerty [INPUT]
   * @return The data shaped into the datagram structure required by the navigator
   */
  static Poco::Buffer<char> convertOdometry2FeedbackDataGram(
    const nav_msgs::msg::Odometry::SharedPtr msg, size_t odom_num_, bool odometrySet,
    rclcpp::Node::SharedPtr node);


  /** @brief convertMotionCommand2Twist Converts Command from navigator binary interface into ros message Twist
   *  @param twist Twist message [OUTPUT]
   *  @param datagram The binary data input datagram [INPUT]
   *  @return number of bytes parsed successfully
   */
  static size_t  convertMotionCommand2Twist(
    const std::vector<char> & datagram,
    geometry_msgs::msg::Twist & twist);
};

#endif  // BOSCH_NAVIGATOR_BRIDGE__ROSMSGS_DATAGRAM_CONVERTER_HPP_
