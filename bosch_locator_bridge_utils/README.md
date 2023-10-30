General information about this repository, including legal information and build instructions are given in [README.md](../README.md) in the repository root.

# bosch_locator_bridge_utils

## Overview

This package provides utilities to use `bosch_locator_bridge` directly with [Nav2].
It provides the ROKIT Locator map as a grid map, and calculates the corresponding transformation from the localization poses.

The package has been tested under [ROS 2] Humble and Ubuntu 22.04.

## Quick Start

This section describes how to use this package to setup the localization part of Nav2.

#### Start the ROKIT Locator and the ROS Bridge

First of all, the ROKIT Locator and the corresponding ROS bridge must be started. Please see the [README.md](../bosch_locator_bridge/README.md) of the `bosch_locator_bridge` package for details.

    ros2 launch bosch_locator_bridge bridge.launch.xml

#### Provide interface to Nav2

Then, start the `locator_node` and the `locator_map_server` nodes with

    ros2 launch bosch_locator_bridge_utils localization_launch.py

From the localization poses, the `locator_node` node calculates and broadcasts the transformation between the `map` and the `odom` frame, which is required for a complete transformation tree for Nav2.
The `map_server` node converts the point cloud map from the ROKIT Locator to a grid map, also needed by Nav2.

To see the map and the transformation tree, start RViz with

    ros2 run rviz2 rviz2 -d $(ros2 pkg prefix bosch_locator_bridge_utils)/share/bosch_locator_bridge_utils/config/localization.rviz

Now everything is ready in terms of localization to start navigating...

[Nav2]: https://navigation.ros.org/
[ROS 2]: https://docs.ros.org/en/humble
