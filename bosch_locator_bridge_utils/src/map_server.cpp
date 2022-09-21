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

#include "bosch_locator_bridge_utils/map_server.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <utility>

namespace bosch_locator_bridge_utils
{
MapServer::MapServer(const std::string & node_name)
: nav2_util::LifecycleNode(node_name)
{
  declare_parameter("frame_id", "map");
  declare_parameter("grid_resolution", 0.02f);
  declare_parameter("measurement_contribution", 100);
  declare_parameter("topic_name_cloud", "/bridge_node/client_localization_map");
  declare_parameter("topic_name_grid", "map");
}

nav2_util::CallbackReturn MapServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  frame_id_ = get_parameter("frame_id").as_string();
  grid_resolution_ = static_cast<float>(get_parameter("grid_resolution").as_double());
  measurement_contribution_ = get_parameter("measurement_contribution").as_int();
  topic_name_cloud_ = get_parameter("topic_name_cloud").as_string();
  const std::string topic_name_grid = get_parameter("topic_name_grid").as_string();

  grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name_grid,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MapServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  grid_pub_->on_activate();

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_name_cloud_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&MapServer::cloudCallback, this, std::placeholders::_1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MapServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  grid_pub_->on_deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MapServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  grid_pub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MapServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  return nav2_util::CallbackReturn::SUCCESS;
}

void MapServer::cloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  convertCloud(msg);

  // update msg header
  // msg_.info.map_load_time = now();
  grid_msg_.header.frame_id = frame_id_;
  // msg_.header.stamp = now();

  auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(grid_msg_);
  grid_pub_->publish(std::move(occ_grid));
}

void MapServer::convertCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  pcl::fromROSMsg(*cloud, point_cloud);

  // Compute bounding box
  float xMin = FLT_MAX;
  float xMax = -FLT_MAX;
  float yMin = xMin;
  float yMax = xMax;

  for (size_t i = 0; i != point_cloud.points.size(); ++i) {
    const auto & pt = point_cloud.points[i];

    if (pt.x < xMin) {
      xMin = pt.x;
    }
    if (pt.x > xMax) {
      xMax = pt.x;
    }
    if (pt.y < yMin) {
      yMin = pt.y;
    }
    if (pt.y > yMax) {
      yMax = pt.y;
    }
  }

  // Create gridmap

  grid_msg_.info.resolution = grid_resolution_;
  const float x1 = floor(xMin / grid_msg_.info.resolution) * grid_msg_.info.resolution;
  const float x2 = ceil(xMax / grid_msg_.info.resolution + 0.5f) * grid_msg_.info.resolution;
  const float y1 = floor(yMin / grid_msg_.info.resolution) * grid_msg_.info.resolution;
  const float y2 = ceil(yMax / grid_msg_.info.resolution + 0.5f) * grid_msg_.info.resolution;
  grid_msg_.info.width = ceil((x2 - x1) / grid_msg_.info.resolution);
  grid_msg_.info.height = ceil((y2 - y1) / grid_msg_.info.resolution);
  grid_msg_.header = cloud->header;
  grid_msg_.info.map_load_time = grid_msg_.header.stamp;
  grid_msg_.info.origin.position.x = x1;
  grid_msg_.info.origin.position.y = y1;
  grid_msg_.data.clear();
  grid_msg_.data.resize(grid_msg_.info.width * grid_msg_.info.height, static_cast<int8_t>(0));

  for (size_t i = 0; i != point_cloud.points.size(); ++i) {
    float dx = point_cloud.points[i].x - x1;
    float dy = point_cloud.points[i].y - y1;
    int jx = floor(dx / grid_msg_.info.resolution);
    int jy = floor(dy / grid_msg_.info.resolution);
    int j = jy * grid_msg_.info.width + jx;

    if (0 <= j && j < static_cast<int>(grid_msg_.data.size())) {
      int occ = static_cast<int>(grid_msg_.data[j]) + measurement_contribution_;
      if (occ > 100) {
        occ = 100;
      }
      grid_msg_.data[j] = static_cast<int8_t>(occ);
    }
  }
}

}  // namespace bosch_locator_bridge_utils
