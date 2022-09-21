// Copyright (c) 2022 - for information on the respective copyright owner
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

#ifndef BOSCH_LOCATOR_BRIDGE_UTILS__MAP_SERVER_HPP_
#define BOSCH_LOCATOR_BRIDGE_UTILS__MAP_SERVER_HPP_

#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace bosch_locator_bridge_utils
{
// The MapServer class subscribes to a point cloud map,
// converts it into a gridmap and publishes it again.
class MapServer : public nav2_util::LifecycleNode
{
public:
  explicit MapServer(const std::string & node_name);

protected:
  nav2_util::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state) override;

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void convertCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

  std::string frame_id_, topic_name_cloud_;
  nav_msgs::msg::OccupancyGrid grid_msg_;
  float grid_resolution_ = 0.0f;
  int measurement_contribution_ = 0;
};
}  // namespace bosch_locator_bridge_utils

#endif  // BOSCH_LOCATOR_BRIDGE_UTILS__MAP_SERVER_HPP_
