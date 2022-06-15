[![License](https://img.shields.io/badge/License-Apache%202-blue.svg)](LICENSE)
[![Build status](http://build.ros.org/job/Ndev__locator_ros_bridge__ubuntu_focal_amd64/badge/icon?subject=Build%20farm%3A%20Noetic)](http://build.ros.org/job/Ndev__locator_ros_bridge__ubuntu_focal_amd64/)
[![Build status](http://build.ros2.org/job/Fdev__locator_ros_bridge__ubuntu_focal_amd64/badge/icon?subject=Build%20farm%3A%20Foxy)](http://build.ros2.org/job/Fdev__locator_ros_bridge__ubuntu_focal_amd64/)
[![Build status](http://build.ros2.org/job/Rdev__locator_ros_bridge__ubuntu_jammy_amd64/badge/icon?subject=Build%20farm%3A%20Rolling)](http://build.ros2.org/job/Rdev__locator_ros_bridge__ubuntu_jammy_amd64/)
[![Build action: Noetic](https://github.com/boschglobal/locator_ros_bridge/actions/workflows/build_noetic.yml/badge.svg?branch=noetic)](https://github.com/boschglobal/locator_ros_bridge/actions/workflows/build_noetic.yml)
[![Build action: Foxy](https://github.com/boschglobal/locator_ros_bridge/actions/workflows/build_foxy.yml/badge.svg?branch=foxy)](https://github.com/boschglobal/locator_ros_bridge/actions/workflows/build_foxy.yml)
[![Build action: Galactic](https://github.com/boschglobal/locator_ros_bridge/actions/workflows/build_galactic.yml/badge.svg?branch=main)](https://github.com/boschglobal/locator_ros_bridge/actions/workflows/build_galactic.yml)
[![Build action: Humble](https://github.com/boschglobal/locator_ros_bridge/actions/workflows/build_humble.yml/badge.svg?branch=main)](https://github.com/boschglobal/locator_ros_bridge/actions/workflows/build_humble.yml)
[![Build action: Rolling](https://github.com/boschglobal/locator_ros_bridge/actions/workflows/build_rolling.yml/badge.svg?branch=main)](https://github.com/boschglobal/locator_ros_bridge/actions/workflows/build_rolling.yml)

---
**Level Up Your Mobile Robots. Rexroth ROKIT Locator – Your Easy-to-Use Laser Localization Software**

---

# locator_ros_bridge

This repository contains the [bosch_locator_bridge](bosch_locator_bridge) package, which provides a [ROS] interface to the [Rexroth ROKIT Locator].
It translates ROS messages to the ROKIT Locator API (as described in the ROKIT Locator API documentation) and vice versa.
It also allows to control the ROKIT Locator via ROS service calls.

There are versions for the following ROS 1 and ROS 2 distributions:
* ROS 1: Noetic (this branch, will likely also work on Melodic)
* ROS 2: Foxy (branch [foxy](../../tree/foxy)), Rolling (branch [main](../../tree/main), will likely also work on Galactic)

The following video (click on image) gives more information about the ROKIT Locator.
[![Rexroth ROKIT Locator](https://dc-mkt-prod.cloud.bosch.tech/xrm/media/global/product_group_1/components_for_mobile_robotics/rokit/landingpage-stage-bild-keyvisual-locator-gruppe-a.jpg)](https://www.youtube.com/watch?v=g6SIUlXn9Bk)

## Installation

### Installing from Debian Package

You can install the `bosch_locator_bridge` package directly:

    sudo apt install ros-noetic-bosch-locator-bridge

Note that the installed package may contain an older software version, which corresponds to the latest tag 1.0.x here: [tags].
Since the release of a package can take a while, the installed package may even be from an earlier tag.
To be sure, check the version of the installed package as follows:

    apt show ros-noetic-bosch-locator-bridge

### Building from Source

#### Dependencies

- [ROS]
- [Poco] C++ library (Should be installed automatically with ROS, otherwise try: ```sudo apt install libpoco-dev```)

#### Building

To build from source, make sure your catkin workspace is set up correctly. Then clone the latest version from this repository into your catkin workspace and compile the package using

    cd catkin_ws
    rosdep install --from-paths . --ignore-src
    catkin_make


## How to Get Started

To get started, take a look at the [README.md](bosch_locator_bridge/README.md) of the bosch_locator_bridge package.

## License

locator_ros_bridge is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.


[ROS]: https://www.ros.org/
[Poco]: https://pocoproject.org/
[Rexroth ROKIT Locator]: https://www.boschrexroth.com/en/xc/products/product-groups/components-for-mobile-robotics/index
[tags]: https://github.com/boschglobal/locator_ros_bridge/tags
